#include "hmi_bms.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace hmi_bms {

static const char *const TAG = "hmi_bms";

void HMIBMS::setup() {
}

void HMIBMS::update() {
  std::vector<uint8_t> payload;
  payload.push_back(HMI_MSG_READ_REGISTERS);
  payload.push_back(0x00); // Address 0 (broadcast or unassigned)
  
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
  uint8_t len = payload.size();
  std::vector<uint8_t> frame;
  frame.push_back(len);
  frame.insert(frame.end(), payload.begin(), payload.end());
  
  uint16_t crc = this->crc16_(frame.data(), frame.size());
  
  ESP_LOGD(TAG, "Sending packet: FF %02X %s %02X %02X", 
           len, format_hex_pretty(payload).c_str(), crc & 0xFF, crc >> 8);

  this->write_byte(0xFF);
  this->write_byte(len);
  this->write_array(payload);
  // Send Little Endian CRC
  this->write_byte(crc & 0xFF);
  this->write_byte(crc >> 8);
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
      // Valid types are 0x81 (Announce), 0x84 (Read Response), etc.
      if (byte == 0x81 || byte == 0x84 || byte == 0x01 || byte == 0x03 || byte == 0x04) {
        this->rx_buffer_.push_back(byte);
      } else {
        // Not a valid message type right after sync+len, likely noise.
        // Discard and look for next 0xFF
        this->rx_buffer_.clear();
        if (byte == 0xFF) this->rx_buffer_.push_back(byte);
      }
    } else {
      this->rx_buffer_.push_back(byte);
      uint8_t payload_len = this->rx_buffer_[1];
      // Packet is: 0xFF, len, payload[len], crc[2]
      if (this->rx_buffer_.size() == (size_t)payload_len + 4) {
        // Complete packet
        // Try Little Endian CRC first
        uint16_t received_crc = (this->rx_buffer_[payload_len + 3] << 8) | this->rx_buffer_[payload_len + 2];
        // Calculate CRC starting from the length byte
        uint16_t computed_crc = this->crc16_(this->rx_buffer_.data() + 1, payload_len + 1);
        
        if (received_crc == computed_crc || this->bypass_crc_) {
          if (received_crc != computed_crc) {
            ESP_LOGW(TAG, "CRC mismatch (bypassed): 0x%04X != 0x%04X (len=%d)", received_crc, computed_crc, payload_len);
            ESP_LOGD(TAG, "Raw payload: %s", format_hex_pretty(this->rx_buffer_.data() + 2, payload_len).c_str());
          }
          this->handle_packet_(this->rx_buffer_.data() + 2, payload_len);
          this->rx_buffer_.clear();
        } else {
          // Try Big Endian CRC and including length
          received_crc = (this->rx_buffer_[payload_len + 2] << 8) | this->rx_buffer_[payload_len + 3];
          if (received_crc == computed_crc) {
             this->handle_packet_(this->rx_buffer_.data() + 2, payload_len);
             this->rx_buffer_.clear();
          } else {
             ESP_LOGW(TAG, "CRC mismatch: 0x%04X != 0x%04X (len=%d)", received_crc, computed_crc, payload_len);
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
          ESP_LOGI(TAG, "Device announced: Type=0x%02X, Addr=0x%02X", device_type, address);
          msg_len = 11;
        } else {
          msg_len = length - offset; // Error
        }
        break;
      default:
        ESP_LOGD(TAG, "Unknown message type: 0x%02X at offset %d", msg_type, offset);
        return;
    }
    offset += msg_len;
  }
}

void HMIBMS::handle_read_registers_response_(const uint8_t *data, size_t length) {
  if (length < 2) return;
  uint8_t address = data[1];
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
      this->soc_sensor_->publish_state(val); 
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

uint16_t HMIBMS::crc16_(const uint8_t *data, size_t len) {
  // Common CRC16-CCITT (0x1021)
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
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
