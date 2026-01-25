#include "hmi_bms.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace hmi_bms {

static const char *const TAG = "hmi_bms";

static const char* EVENT_LEVELS[] = {
    "NONE",
    "INFO",
    "WARNING",
    "CRITICAL",
    "FATAL"
};

static const char* EVENT_TYPES[] = {
    "POS STUCK OPEN",
    "POS STUCK CLOSED",
    "NEG STUCK OPEN",
    "NEG STUCK CLOSED",
    "PRE STUCK OPEN",
    "PRE STUCK CLOSED",
    "TEST FAIL POS",
    "TEST FAIL NEG",
    "TEST FAIL PRE",
    "PRECHARGE FAIL",
    "CURRENT STALE",
    "SUPPLY STALE",
    "BMB STALE",
    "TEMP STALE",
    "SOC STALE",
    "BMB MISSING",
    "BMB FAULT",
    "BMB VOLTAGE MISMATCH",
    "BMB TEMP MISMATCH",
    "BMB SPI ERROR",
    "BMB UART ERROR",
    "BMB CAN ERROR",
    "CELL VOLTAGE LOW",
    "CELL VOLTAGE HIGH",
    "CELL VOLTAGE CRITICAL LOW",
    "CELL VOLTAGE CRITICAL HIGH",
    "CELL VOLTAGE DELTA",
    "PACK VOLTAGE LOW",
    "PACK VOLTAGE HIGH",
    "PACK VOLTAGE CRITICAL LOW",
    "PACK VOLTAGE CRITICAL HIGH",
    "CURRENT HIGH CHARGE",
    "CURRENT HIGH DISCHARGE",
    "CURRENT CRITICAL HIGH CHARGE",
    "CURRENT CRITICAL HIGH DISCHARGE",
    "TEMP LOW CHARGE",
    "TEMP LOW DISCHARGE",
    "TEMP HIGH CHARGE",
    "TEMP HIGH DISCHARGE",
    "TEMP CRITICAL LOW",
    "TEMP CRITICAL HIGH",
    "ESTOP PRESSED",
    "BOOT NORMAL",
    "BOOT WATCHDOG",
    "BOOT BROWNOUT",
    "BOOT SOFTWARE",
    "BOOT OTHER"
};

void HMIBMS::setup() {
}

void HMIBMS::update() {
  if (this->address_ == 0) {
    ESP_LOGD(TAG, "No device address assigned yet, skipping register read");
    return;
  }

  this->update_count_++;
  bool slow_poll = (this->update_count_ % 10 == 0);

  bool use_cv_cmd = !this->cell_voltage_sensors_.empty() && slow_poll;

  std::vector<uint8_t> payload;
  payload.push_back(HMI_MSG_READ_REGISTERS);
  payload.push_back(this->address_);
  
  auto add_reg = [&](uint16_t reg) {
    payload.push_back(reg & 0xFF); // Lo byte first (Little Endian)
    payload.push_back(reg >> 8);   // Hi byte
  };

  // 1Hz Slow Poll Registers
  if (slow_poll) {
    if (this->millis_sensor_) add_reg(HMI_REG_MILLIS);
    if (this->serial_sensor_) add_reg(HMI_REG_SERIAL);
    if (this->system_request_sensor_) add_reg(HMI_REG_SYSTEM_REQUEST);
    if (this->soc_sensor_) add_reg(HMI_REG_SOC);
    if (this->charge_raw_sensor_) add_reg(HMI_REG_CHARGE);
    if (this->temperature_min_dC_sensor_) add_reg(HMI_REG_TEMPERATURE_MIN);
    if (this->temperature_max_dC_sensor_) add_reg(HMI_REG_TEMPERATURE_MAX);
    if (this->cell_voltage_min_mV_sensor_) add_reg(HMI_REG_CELL_VOLTAGE_MIN);
    if (this->cell_voltage_max_mV_sensor_) add_reg(HMI_REG_CELL_VOLTAGE_MAX);
    if (this->system_sm_sensor_) add_reg(HMI_REG_SYSTEM_STATE);
    if (this->soc_voltage_based_sensor_) add_reg(HMI_REG_SOC_VOLTAGE_BASED);
    if (this->soc_basic_count_sensor_) add_reg(HMI_REG_SOC_BASIC_COUNT);
    if (this->soc_ekf_sensor_) add_reg(HMI_REG_SOC_EKF);
    if (this->capacity_mC_sensor_) add_reg(HMI_REG_CAPACITY);
    if (this->supply_voltage_3V3_mV_sensor_) add_reg(HMI_REG_SUPPLY_VOLTAGE_3V3);
    if (this->supply_voltage_5V_mV_sensor_) add_reg(HMI_REG_SUPPLY_VOLTAGE_5V);
    if (this->supply_voltage_12V_mV_sensor_) add_reg(HMI_REG_SUPPLY_VOLTAGE_12V);
    if (this->supply_voltage_contactor_mV_sensor_) add_reg(HMI_REG_SUPPLY_VOLTAGE_CTR);
    if (this->pos_contactor_voltage_mV_sensor_) add_reg(HMI_REG_POS_CONTACTOR_VOLTAGE);
    if (this->neg_contactor_voltage_mV_sensor_) add_reg(HMI_REG_NEG_CONTACTOR_VOLTAGE);
    
    // Module Temperatures (0x200 - 0x207) - always poll these (8 * 7 bytes = 56 bytes)
    for (uint16_t i = 0; i < this->module_temperature_sensors_.size() && i < 8; i++) {
      add_reg(HMI_REG_MODULE_TEMPS_START + i);
    }

    // Raw Temperatures (0x208 - 0x238) - Poll in chunks of 8 to stay under 256-byte limit
    // 8 registers * 7 bytes/reg = 56 bytes per batch.
    uint16_t num_raw = this->raw_temperature_sensors_.size();
    if (num_raw > 0) {
      if (num_raw > 48) num_raw = 48;
      for (uint16_t i = 0; i < 8; i++) {
        uint16_t idx = (this->temp_poll_index_ + i) % num_raw;
        add_reg(HMI_REG_RAW_TEMPS_START + idx);
      }
      this->temp_poll_index_ = (this->temp_poll_index_ + 8) % num_raw;
    }
  }

  // 10Hz Fast Poll Registers (V, A, Contactor State)
  if (this->current_mA_sensor_) add_reg(HMI_REG_CURRENT);
  if (this->battery_voltage_mV_sensor_) add_reg(HMI_REG_BATTERY_VOLTAGE);
  if (this->output_voltage_mV_sensor_) add_reg(HMI_REG_OUTPUT_VOLTAGE);
  if (this->contactor_sm_sensor_) add_reg(HMI_REG_CONTACTORS_STATE);

  if (payload.size() > 2) {
    this->send_packet_(payload);
  }

  if (use_cv_cmd) {
    std::vector<uint8_t> cv_payload;
    cv_payload.push_back(HMI_MSG_READ_CELL_VOLTAGES);
    cv_payload.push_back(this->address_);
    this->send_packet_(cv_payload);
  }

  if (slow_poll) {
    std::vector<uint8_t> event_payload;
    event_payload.push_back(HMI_MSG_READ_EVENTS);
    event_payload.push_back(this->address_);
    event_payload.push_back(0x00); // Start index Lo
    event_payload.push_back(0x00); // Start index Hi
    this->send_packet_(event_payload);
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

void HMIBMS::send_system_request(uint8_t request) {
  if (this->address_ == 0) {
    ESP_LOGW(TAG, "Cannot send system request: No device address assigned");
    return;
  }
  ESP_LOGI(TAG, "Sending system request: %u", request);
  std::vector<uint8_t> payload;
  payload.push_back(HMI_MSG_WRITE_REGISTERS);
  payload.push_back(this->address_);
  payload.push_back(HMI_REG_SYSTEM_REQUEST & 0xFF); // Reg Lo
  payload.push_back(HMI_REG_SYSTEM_REQUEST >> 8);   // Reg Hi
  payload.push_back(HMI_TYPE_UINT8);
  payload.push_back(request);
  this->send_packet_(payload);
  
  if (this->system_request_sensor_) {
    this->publish_queue_.push_back({this->system_request_sensor_, (float)request});
  }
}

void HMIBMS::loop() {
  // 1. Process all sensor updates from the queue
  while (!this->publish_queue_.empty()) {
    auto update = this->publish_queue_.front();
    this->publish_queue_.erase(this->publish_queue_.begin());
    if (update.sensor != nullptr) {
      update.sensor->publish_state(update.state);
    }
  }

  static std::vector<uint8_t> dump_buffer;
  uint32_t bytes_read = 0;
  
  // 2. Limit number of bytes read per loop to avoid UART saturation
  while (this->available() && bytes_read < 128) {
    uint8_t byte;
    this->read_byte(&byte);
    bytes_read++;

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
          // Stop processing more bytes this loop to give the system air
          return;
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
      case HMI_MSG_READ_CELL_VOLTAGES_RESPONSE:
        this->handle_read_cell_voltages_response_(payload + offset, length - offset);
        return;
      case HMI_MSG_READ_EVENTS_RESPONSE:
        this->handle_read_events_response_(payload + offset, length - offset);
        return;
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
  
  float t_min = NAN;
  float t_max = NAN;
  float v_min = NAN;
  float v_max = NAN;

  while (offset + 3 <= length) {
    uint16_t reg_id = data[offset] | (data[offset + 1] << 8);
    uint8_t reg_type = data[offset + 2];
    offset += 3;
    size_t val_size = reg_type & 0x0F;
    if (val_size == 0 || offset + val_size > length) return;
    const uint8_t *v = data + offset;
    
    if (reg_id == HMI_REG_MILLIS && this->millis_sensor_ != nullptr) {
      uint64_t val = (uint64_t)v[0] | ((uint64_t)v[1] << 8) | ((uint64_t)v[2] << 16) | ((uint64_t)v[3] << 24) |
                     ((uint64_t)v[4] << 32) | ((uint64_t)v[5] << 40) | ((uint64_t)v[6] << 48) | ((uint64_t)v[7] << 56);
      this->publish_queue_.push_back({this->millis_sensor_, (float)val});
    } else if (reg_id == HMI_REG_SERIAL && this->serial_sensor_ != nullptr) {
      uint64_t val = (uint64_t)v[0] | ((uint64_t)v[1] << 8) | ((uint64_t)v[2] << 16) | ((uint64_t)v[3] << 24) |
                     ((uint64_t)v[4] << 32) | ((uint64_t)v[5] << 40) | ((uint64_t)v[6] << 48) | ((uint64_t)v[7] << 56);
      this->publish_queue_.push_back({this->serial_sensor_, (float)val});
    } else if (reg_id == HMI_REG_SOC && this->soc_sensor_ != nullptr) {
      uint32_t val = (reg_type == HMI_TYPE_UINT16) ? (v[0] | (v[1] << 8)) : ((uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24));
      this->publish_queue_.push_back({this->soc_sensor_, val / 100.0f});
    } else if (reg_id == HMI_REG_CURRENT && this->current_mA_sensor_ != nullptr) {
      int32_t val = (int32_t)((uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24));
      this->publish_queue_.push_back({this->current_mA_sensor_, val / 1000.0f});
    } else if (reg_id == HMI_REG_CHARGE && this->charge_raw_sensor_ != nullptr) {
      int64_t val;
      if (reg_type == HMI_TYPE_INT64) {
        val = (int64_t)v[0] | ((int64_t)v[1] << 8) | ((int64_t)v[2] << 16) | ((int64_t)v[3] << 24) |
              ((int64_t)v[4] << 32) | ((int64_t)v[5] << 40) | ((int64_t)v[6] << 48) | ((int64_t)v[7] << 56);
      } else {
        val = (int32_t)((uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24));
      }
      this->publish_queue_.push_back({this->charge_raw_sensor_, (float)(val / 3600000.0f)}); // Convert mC to Ah
    } else if (reg_id == HMI_REG_BATTERY_VOLTAGE && this->battery_voltage_mV_sensor_ != nullptr) {
      int32_t val = (int32_t)((uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24));
      this->publish_queue_.push_back({this->battery_voltage_mV_sensor_, val / 1000.0f});
    } else if (reg_id == HMI_REG_OUTPUT_VOLTAGE && this->output_voltage_mV_sensor_ != nullptr) {
      int32_t val = (int32_t)((uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24));
      this->publish_queue_.push_back({this->output_voltage_mV_sensor_, val / 1000.0f});
    } else if (reg_id == HMI_REG_POS_CONTACTOR_VOLTAGE && this->pos_contactor_voltage_mV_sensor_ != nullptr) {
      int32_t val = (int32_t)((uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24));
      this->publish_queue_.push_back({this->pos_contactor_voltage_mV_sensor_, val / 1000.0f});
    } else if (reg_id == HMI_REG_NEG_CONTACTOR_VOLTAGE && this->neg_contactor_voltage_mV_sensor_ != nullptr) {
      int32_t val = (int32_t)((uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24));
      this->publish_queue_.push_back({this->neg_contactor_voltage_mV_sensor_, val / 1000.0f});
    } else if (reg_id == HMI_REG_TEMPERATURE_MIN) {
      int16_t val = (int16_t)(v[0] | (v[1] << 8));
      t_min = val / 10.0f;
      if (this->temperature_min_dC_sensor_) this->publish_queue_.push_back({this->temperature_min_dC_sensor_, t_min});
    } else if (reg_id == HMI_REG_TEMPERATURE_MAX) {
      int16_t val = (int16_t)(v[0] | (v[1] << 8));
      t_max = val / 10.0f;
      if (this->temperature_max_dC_sensor_) this->publish_queue_.push_back({this->temperature_max_dC_sensor_, t_max});
    } else if (reg_id == HMI_REG_CELL_VOLTAGE_MIN) {
      int16_t val = (int16_t)(v[0] | (v[1] << 8));
      v_min = val / 1000.0f;
      if (this->cell_voltage_min_mV_sensor_) this->publish_queue_.push_back({this->cell_voltage_min_mV_sensor_, v_min});
    } else if (reg_id == HMI_REG_CELL_VOLTAGE_MAX) {
      int16_t val = (int16_t)(v[0] | (v[1] << 8));
      v_max = val / 1000.0f;
      if (this->cell_voltage_max_mV_sensor_) this->publish_queue_.push_back({this->cell_voltage_max_mV_sensor_, v_max});
    } else if (reg_id == HMI_REG_SYSTEM_REQUEST && this->system_request_sensor_ != nullptr) {
      this->publish_queue_.push_back({this->system_request_sensor_, (float)v[0]});
    } else if (reg_id == HMI_REG_SYSTEM_STATE && this->system_sm_sensor_ != nullptr) {
      this->publish_queue_.push_back({this->system_sm_sensor_, (float)v[0]});
    } else if (reg_id == HMI_REG_CONTACTORS_STATE && this->contactor_sm_sensor_ != nullptr) {
      this->publish_queue_.push_back({this->contactor_sm_sensor_, (float)v[0]});
    } else if (reg_id == HMI_REG_SOC_VOLTAGE_BASED && this->soc_voltage_based_sensor_ != nullptr) {
      uint16_t val = v[0] | (v[1] << 8);
      this->publish_queue_.push_back({this->soc_voltage_based_sensor_, val / 100.0f});
    } else if (reg_id == HMI_REG_SOC_BASIC_COUNT && this->soc_basic_count_sensor_ != nullptr) {
      uint16_t val = v[0] | (v[1] << 8);
      this->publish_queue_.push_back({this->soc_basic_count_sensor_, val / 100.0f});
    } else if (reg_id == HMI_REG_SOC_EKF && this->soc_ekf_sensor_ != nullptr) {
      uint16_t val = v[0] | (v[1] << 8);
      this->publish_queue_.push_back({this->soc_ekf_sensor_, val / 100.0f});
    } else if (reg_id == HMI_REG_CAPACITY && this->capacity_mC_sensor_ != nullptr) {
      uint32_t val = (uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24);
      this->publish_queue_.push_back({this->capacity_mC_sensor_, (float)(val / 3600000.0f)}); // Convert mC to Ah
    } else if (reg_id == HMI_REG_SUPPLY_VOLTAGE_3V3 && this->supply_voltage_3V3_mV_sensor_ != nullptr) {
      uint16_t val = v[0] | (v[1] << 8);
      this->publish_queue_.push_back({this->supply_voltage_3V3_mV_sensor_, val / 1000.0f});
    } else if (reg_id == HMI_REG_SUPPLY_VOLTAGE_5V && this->supply_voltage_5V_mV_sensor_ != nullptr) {
      uint16_t val = v[0] | (v[1] << 8);
      this->publish_queue_.push_back({this->supply_voltage_5V_mV_sensor_, val / 1000.0f});
    } else if (reg_id == HMI_REG_SUPPLY_VOLTAGE_12V && this->supply_voltage_12V_mV_sensor_ != nullptr) {
      uint16_t val = v[0] | (v[1] << 8);
      this->publish_queue_.push_back({this->supply_voltage_12V_mV_sensor_, val / 1000.0f});
    } else if (reg_id == HMI_REG_SUPPLY_VOLTAGE_CTR && this->supply_voltage_contactor_mV_sensor_ != nullptr) {
      uint16_t val = v[0] | (v[1] << 8);
      this->publish_queue_.push_back({this->supply_voltage_contactor_mV_sensor_, val / 1000.0f});
    } else if (reg_id >= HMI_REG_MODULE_TEMPS_START && reg_id <= HMI_REG_MODULE_TEMPS_END) {
      uint16_t idx = reg_id - HMI_REG_MODULE_TEMPS_START;
      if (idx < this->module_temperature_sensors_.size() && this->module_temperature_sensors_[idx] != nullptr) {
        int16_t val = (int16_t)(v[0] | (v[1] << 8));
        this->publish_queue_.push_back({this->module_temperature_sensors_[idx], val / 10.0f});
      }
    } else if (reg_id >= HMI_REG_RAW_TEMPS_START && reg_id <= HMI_REG_RAW_TEMPS_END) {
      uint16_t idx = reg_id - HMI_REG_RAW_TEMPS_START;
      if (idx < this->raw_temperature_sensors_.size() && this->raw_temperature_sensors_[idx] != nullptr) {
        int16_t val = (int16_t)(v[0] | (v[1] << 8));
        this->publish_queue_.push_back({this->raw_temperature_sensors_[idx], val / 10.0f});
      }
    }
    offset += val_size;
  }

  // Calculate and publish deltas using cached values or existing state
  if (this->temperature_delta_sensor_) {
    float min_val = std::isnan(t_min) ? (this->temperature_min_dC_sensor_ ? this->temperature_min_dC_sensor_->state : NAN) : t_min;
    float max_val = std::isnan(t_max) ? (this->temperature_max_dC_sensor_ ? this->temperature_max_dC_sensor_->state : NAN) : t_max;
    if (!std::isnan(min_val) && !std::isnan(max_val)) {
      this->publish_queue_.push_back({this->temperature_delta_sensor_, max_val - min_val});
    }
  }
  if (this->cell_voltage_delta_sensor_) {
    float min_val = std::isnan(v_min) ? (this->cell_voltage_min_mV_sensor_ ? this->cell_voltage_min_mV_sensor_->state : NAN) : v_min;
    float max_val = std::isnan(v_max) ? (this->cell_voltage_max_mV_sensor_ ? this->cell_voltage_max_mV_sensor_->state : NAN) : v_max;
    if (!std::isnan(min_val) && !std::isnan(max_val)) {
      this->publish_queue_.push_back({this->cell_voltage_delta_sensor_, max_val - min_val});
    }
  }
}

void HMIBMS::handle_read_cell_voltages_response_(const uint8_t *data, size_t length) {
  if (length < 3) return;
  uint8_t address = data[1];
  uint8_t num_cells = data[2];
  
  size_t offset = 3;
  int16_t last_cell_voltage = 0;
  uint16_t cells_found = 0;
  
  for (int i = 0; i < num_cells && offset < length; i++) {
    int16_t cell_voltage;
    if (data[offset] & 0x80) {
      // Absolute voltage (2 bytes, big-endian)
      if (offset + 1 >= length) break;
      uint16_t val = (((uint16_t)data[offset] & 0x7F) << 8) | data[offset + 1];
      // Sign extend 15-bit to 16-bit
      if (val & 0x4000) {
        cell_voltage = (int16_t)(val | 0x8000);
      } else {
        cell_voltage = (int16_t)val;
      }
      offset += 2;
    } else {
      // Delta voltage (1 byte, signed 7-bit)
      uint8_t val = data[offset] & 0x7F;
      int8_t delta;
      if (val & 0x40) {
        delta = (int8_t)(val | 0x80);
      } else {
        delta = (int8_t)val;
      }
      cell_voltage = last_cell_voltage + delta;
      offset += 1;
    }
    
    if (i < this->cell_voltage_sensors_.size() && this->cell_voltage_sensors_[i] != nullptr) {
      if (cell_voltage == -1) {
        this->publish_queue_.push_back({this->cell_voltage_sensors_[i], NAN});
      } else {
        this->publish_queue_.push_back({this->cell_voltage_sensors_[i], cell_voltage / 1000.0f});
        cells_found++;
      }
    }
    last_cell_voltage = cell_voltage;
  }
  ESP_LOGD(TAG, "Read %u cell voltages from 0x%02X (queued)", cells_found, address);
}

void HMIBMS::handle_read_events_response_(const uint8_t *data, size_t length) {
  if (length < 6) return;
  uint8_t address = data[1];
  uint16_t next_index = data[2] | (data[3] << 8);
  uint16_t count = data[4] | (data[5] << 8);
  
  ESP_LOGD(TAG, "Read events response from 0x%02X, count=%u", address, count);
  
  size_t offset = 6;
  uint16_t highest_level = 0;
  uint16_t latest_event_type = 0xFFFF;
  std::string history = "";
  uint32_t now_millis = millis();

  for (uint16_t i = 0; i < count && offset + 22 <= length; i++) {
    uint16_t type = data[offset] | (data[offset + 1] << 8);
    uint16_t level = data[offset + 2] | (data[offset + 3] << 8);
    uint16_t event_count = data[offset + 4] | (data[offset + 5] << 8);
    
    uint64_t timestamp = 0;
    for (int j = 0; j < 8; j++) timestamp |= (uint64_t)data[offset + 6 + j] << (j * 8);
    
    uint64_t event_data = 0;
    for (int j = 0; j < 8; j++) event_data |= (uint64_t)data[offset + 14 + j] << (j * 8);

    if (level > highest_level) {
      highest_level = level;
    }
    latest_event_type = type;

    // Build history string
    char buf[128];
    const char* type_str = (type < sizeof(EVENT_TYPES)/sizeof(EVENT_TYPES[0])) ? EVENT_TYPES[type] : "UNKNOWN";
    const char* level_str = (level < sizeof(EVENT_LEVELS)/sizeof(EVENT_LEVELS[0])) ? EVENT_LEVELS[level] : "??";
    
    // We don't have absolute wall time, so we show relative to BMS uptime or just raw ID
    snprintf(buf, sizeof(buf), "[%s] %u %s (x%u)\n", level_str, type, type_str, event_count);
    history += buf;

    offset += 22;
  }

  if (this->highest_event_level_sensor_) {
    this->publish_queue_.push_back({this->highest_event_level_sensor_, (float)highest_level});
  }
  if (this->last_event_sensor_ && latest_event_type != 0xFFFF) {
    this->publish_queue_.push_back({this->last_event_sensor_, (float)latest_event_type});
  }
  if (this->event_history_text_sensor_ && !history.empty()) {
    this->event_history_text_sensor_->publish_state(history);
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
