#include "hmi_bms.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace hmi_bms {

static const char *const TAG = "hmi_bms";

void HMIBMS::setup() {
}

void HMIBMS::update() {
  if (this->address_ == 0) {
    ESP_LOGD(TAG, "No device address assigned yet, skipping register read");
    return;
  }

  std::vector<uint8_t> payload;
  payload.push_back(HMI_MSG_READ_REGISTERS);
  payload.push_back(this->address_);
  
  auto add_reg = [&](uint16_t reg) {
    payload.push_back(reg & 0xFF); // Lo byte first (Little Endian)
    payload.push_back(reg >> 8);   // Hi byte
  };

  if (this->soc_sensor_) add_reg(HMI_REG_SOC);
  if (this->current_sensor_) add_reg(HMI_REG_CURRENT);
  if (this->battery_voltage_sensor_) add_reg(HMI_REG_BATTERY_VOLTAGE);
  if (this->temperature_min_sensor_) add_reg(HMI_REG_TEMPERATURE_MIN);
  if (this->temperature_max_sensor_) add_reg(HMI_REG_TEMPERATURE_MAX);
  if (this->cell_voltage_min_sensor_) add_reg(HMI_REG_CELL_VOLTAGE_MIN);
  if (this->cell_voltage_max_sensor_) add_reg(HMI_REG_CELL_VOLTAGE_MAX);

  if (payload.size() > 2) {
    this->send_packet_(payload);
  }
}

void HMIBMS::send_packet_(const std::vector<uint8_t> &payload) {
  if (payload.empty()) return;
  uint8_t len_minus_1 = payload.size() - 1;
  std::vector<uint8_t> frame;
  frame.push_back(0xFF); // Sync byte included in CRC
  frame.push_back(len_minus_1);
  frame.insert(frame.end(), payload.begin(), payload.end());
  
  // Outgoing CRC includes the 0xFF sync byte and length byte
  uint16_t crc = this->crc16_modbus_(frame.data(), frame.size());
  
  frame.push_back(crc & 0xFF);
  frame.push_back(crc >> 8);

  ESP_LOGD(TAG, "Sending packet: %s (payload_len=%zu)", 
           format_hex_pretty(frame).c_str(), payload.size());

  this->write_array(frame);
}

void HMIBMS::loop() {
  static std::vector<uint8_t> dump_buffer;
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);

    if (this->dump_raw_) {
      dump_buffer.push_back(byte);
      if (dump_buffer.size() >= 32) {
        ESP_LOGD(TAG, "Raw Rx: %s", format_hex_pretty(dump_buffer).c_str());
        dump_buffer.clear();
      }
    }

    if (this->rx_buffer_.empty()) {
      if (byte == 0xFF) {
        this->rx_buffer_.push_back(byte);
      }
    } else if (this->rx_buffer_.size() == 1) {
      // Length byte
      this->rx_buffer_.push_back(byte);
    } else if (this->rx_buffer_.size() == 2) {
      // Message Type byte - check if it's a known BMS message type
      // We'll be more permissive here to catch error responses (e.g. 0x80 + type)
      this->rx_buffer_.push_back(byte);
    } else {
      this->rx_buffer_.push_back(byte);
      uint8_t payload_len_minus_1 = this->rx_buffer_[1];
      size_t payload_len = (size_t)payload_len_minus_1 + 1;
      
      // Packet is: 0xFF, len-1, payload[payload_len], crc[2]
      if (this->rx_buffer_.size() == payload_len + 4) {
        // Complete packet
        uint16_t received_crc = (this->rx_buffer_[payload_len + 3] << 8) | this->rx_buffer_[payload_len + 2];
        // Calculate Modbus CRC on the entire frame [FF, Len-1, Payload...]
        uint16_t computed_crc = this->crc16_modbus_(this->rx_buffer_.data(), payload_len + 2);
        
        if (received_crc == computed_crc || this->bypass_crc_) {
          if (received_crc != computed_crc) {
            ESP_LOGV(TAG, "CRC mismatch (bypassed): 0x%04X != 0x%04X (len=%d)", received_crc, computed_crc, payload_len);
          } else {
            ESP_LOGV(TAG, "CRC match: 0x%04X", computed_crc);
          }
          this->handle_packet_(this->rx_buffer_.data() + 2, payload_len);
          this->rx_buffer_.clear();
        } else {
          // Try Big Endian CRC or alternate check
          received_crc = (this->rx_buffer_[payload_len + 2] << 8) | this->rx_buffer_[payload_len + 3];
          if (received_crc == computed_crc) {
             this->handle_packet_(this->rx_buffer_.data() + 2, payload_len);
             this->rx_buffer_.clear();
          } else {
             ESP_LOGW(TAG, "CRC mismatch: Type=0x%02X LenField=0x%02X Calc=0x%04X Msg=0x%04X", 
                      this->rx_buffer_[2], payload_len_minus_1, computed_crc, received_crc);
             ESP_LOGD(TAG, "Raw packet: %s", format_hex_pretty(this->rx_buffer_).c_str());
             // Robust framing: discard the first byte and try to find a new 0xFF
             this->rx_buffer_.erase(this->rx_buffer_.begin());
             while (!this->rx_buffer_.empty() && this->rx_buffer_[0] != 0xFF) {
               this->rx_buffer_.erase(this->rx_buffer_.begin());
             }
          }
        }
      }
    }
    
    if (this->rx_buffer_.size() > 512) {
      ESP_LOGV(TAG, "Buffer overflow, clearing...");
      this->rx_buffer_.clear();
    }
  }
}

void HMIBMS::handle_packet_(const uint8_t *payload, size_t length) {
  size_t offset = 0;
  while (offset < length) {
    uint8_t msg_type = payload[offset];
    size_t msg_len = 0;

    // Determine message length based on type
    switch (msg_type) {
      case HMI_MSG_READ_REGISTERS_RESPONSE:
        // This message type has variable length based on the registers included
        this->handle_read_registers_response_(payload + offset, length - offset);
        return; // Assume variable length takes rest of packet for now
      case HMI_MSG_ANNOUNCE_DEVICE:
        if (offset + 11 <= length) {
          uint8_t device_type = payload[offset + 1];
          uint8_t address = payload[offset + 2];
          // Serial number is the 8 bytes starting at payload[offset + 3]
          uint64_t serial = 0;
          for (int i = 0; i < 8; i++) {
            serial |= (uint64_t)payload[offset + 3 + i] << (i * 8);
          }
          ESP_LOGI(TAG, "Device announced: Type=0x%02X, Addr=0x%02X, Serial=%llu", device_type, address, serial);
          
          this->serial_ = serial;
          if (address == 0 || address == 1) {
            static uint32_t last_assign_time = 0;
            static bool address_claimed = false;
            uint32_t now = millis();
            
            // Only assign/claim if we haven't done it recently, or if it's still at 0
            if ((address == 0 && now - last_assign_time > 2000) || (address == 1 && !address_claimed)) {
              last_assign_time = now;
              address_claimed = (address == 1);
              
              ESP_LOGI(TAG, "Claiming/Assigning address 0x01 for device %llu", serial);
              std::vector<uint8_t> set_addr_payload;
              set_addr_payload.push_back(HMI_MSG_SET_DEVICE_ADDRESS);
              set_addr_payload.push_back(address); // Current address (0 or 1)
              set_addr_payload.push_back(0x01);    // Target address
              // Send serial in the same order it was received
              for (int i = 0; i < 8; i++) {
                set_addr_payload.push_back(payload[offset + 3 + i]);
              }
              this->send_packet_(set_addr_payload);
            }
          }
          
          if (address != 0) {
            if (this->address_ == 0) {
              ESP_LOGI(TAG, "BMS Address confirmed at 0x%02X. Triggering initial poll...", address);
              this->address_ = address;
              this->update(); // Poll immediately now that we have an address
            }
            this->address_ = address;
          }
          msg_len = 11;
        } else {
          msg_len = length - offset;
        }
        break;
      default:
        ESP_LOGD(TAG, "Unknown message type: 0x%02X at offset %zu of %zu", msg_type, offset, length);
        msg_len = length - offset;
        break;
    }
    offset += msg_len;
  }
}

void HMIBMS::handle_read_registers_response_(const uint8_t *data, size_t length) {
  if (length < 2) return;
  uint8_t address = data[1];
  ESP_LOGD(TAG, "Read registers response from 0x%02X, len=%zu", address, length);
  size_t offset = 2;
  
  while (offset + 3 <= length) {
    // Register ID is 2 bytes, Little Endian (per BMS source)
    uint16_t reg_id = data[offset] | (data[offset + 1] << 8);
    uint8_t reg_type = data[offset + 2];
    offset += 3;
    
    // Low 4 bits of type indicate size in bytes (per BMS source hmi_get_type_size)
    size_t val_size = reg_type & 0x0F;
    
    if (val_size == 0 || offset + val_size > length) {
      ESP_LOGW(TAG, "Invalid register size %d for type 0x%02X at reg 0x%04X", val_size, reg_type, reg_id);
      return;
    }
    
    const uint8_t *v = data + offset;
    
    // Parse values as Little Endian
    if (reg_id == HMI_REG_SOC && this->soc_sensor_ != nullptr) {
      uint32_t val = (uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24);
      this->soc_sensor_->publish_state(val / 100.0f); 
    } else if (reg_id == HMI_REG_CURRENT && this->current_sensor_ != nullptr) {
      int32_t val = (int32_t)((uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24));
      this->current_sensor_->publish_state(val / 1000.0f); // mA to A
    } else if (reg_id == HMI_REG_BATTERY_VOLTAGE && this->battery_voltage_sensor_ != nullptr) {
      int32_t val = (int32_t)((uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24));
      this->battery_voltage_sensor_->publish_state(val / 1000.0f); // mV to V
    } else if (reg_id == HMI_REG_TEMPERATURE_MIN && this->temperature_min_sensor_ != nullptr) {
      int16_t val = (int16_t)(v[0] | (v[1] << 8));
      this->temperature_min_sensor_->publish_state(val / 10.0f); // 0.1C to C
    } else if (reg_id == HMI_REG_TEMPERATURE_MAX && this->temperature_max_sensor_ != nullptr) {
      int16_t val = (int16_t)(v[0] | (v[1] << 8));
      this->temperature_max_sensor_->publish_state(val / 10.0f); // 0.1C to C
    } else if (reg_id == HMI_REG_CELL_VOLTAGE_MIN && this->cell_voltage_min_sensor_ != nullptr) {
      int16_t val = (int16_t)(v[0] | (v[1] << 8));
      this->cell_voltage_min_sensor_->publish_state(val / 1000.0f); // mV to V
    } else if (reg_id == HMI_REG_CELL_VOLTAGE_MAX && this->cell_voltage_max_sensor_ != nullptr) {
      int16_t val = (int16_t)(v[0] | (v[1] << 8));
      this->cell_voltage_max_sensor_->publish_state(val / 1000.0f); // mV to V
    }
    
    offset += val_size;
  }
}

uint16_t HMIBMS::crc16_modbus_(const uint8_t *data, size_t len) {
  // Modbus CRC16 (poly 0x8005 -> reflected 0xA001), init 0xFFFF
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void HMIBMS::dump_config() {
  ESP_LOGCONFIG(TAG, "HMI BMS:");
  this->check_uart_settings(this->baud_rate_);
}

}  // namespace hmi_bms
}  // namespace esphome
