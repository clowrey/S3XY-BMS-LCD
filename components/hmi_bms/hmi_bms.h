#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace hmi_bms {

enum HMIMsgType {
  HMI_MSG_ANNOUNCE_DEVICE = 0x81,
  HMI_MSG_SET_DEVICE_ADDRESS = 0x01,
  HMI_MSG_WRITE_REGISTERS = 0x03,
  HMI_MSG_READ_REGISTERS = 0x04,
  HMI_MSG_READ_REGISTERS_RESPONSE = 0x84,
};

enum HMIType {
  HMI_TYPE_UINT8 = 0x11,
  HMI_TYPE_INT8 = 0x21,
  HMI_TYPE_UINT16 = 0x12,
  HMI_TYPE_INT16 = 0x22,
  HMI_TYPE_UINT32 = 0x14,
  HMI_TYPE_INT32 = 0x24,
  HMI_TYPE_UINT64 = 0x18,
  HMI_TYPE_INT64 = 0x28,
  HMI_TYPE_FLOAT = 0x34,
  HMI_TYPE_DOUBLE = 0x38,
};

enum HMIRegister {
  HMI_REG_SERIAL = 1,
  HMI_REG_MILLIS = 2,
  HMI_REG_SOC = 3,
  HMI_REG_CURRENT = 4,
  HMI_REG_CHARGE = 5,
  HMI_REG_BATTERY_VOLTAGE = 6,
  HMI_REG_OUTPUT_VOLTAGE = 7,
  HMI_REG_POS_CONTACTOR_VOLTAGE = 8,
  HMI_REG_NEG_CONTACTOR_VOLTAGE = 9,
  HMI_REG_TEMPERATURE_MIN = 10,
  HMI_REG_TEMPERATURE_MAX = 11,
  HMI_REG_CELL_VOLTAGE_MIN = 12,
  HMI_REG_CELL_VOLTAGE_MAX = 13,
  HMI_REG_CELL_VOLTAGES_START = 0x100,
  HMI_REG_CELL_VOLTAGES_END = 0x1FF,
};

class HMIBMS : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_soc_sensor(sensor::Sensor *s) { soc_sensor_ = s; }
  void set_current_sensor(sensor::Sensor *s) { current_sensor_ = s; }
  void set_battery_voltage_sensor(sensor::Sensor *s) { battery_voltage_sensor_ = s; }
  void set_temperature_min_sensor(sensor::Sensor *s) { temperature_min_sensor_ = s; }
  void set_temperature_max_sensor(sensor::Sensor *s) { temperature_max_sensor_ = s; }
  void set_cell_voltage_min_sensor(sensor::Sensor *s) { cell_voltage_min_sensor_ = s; }
  void set_cell_voltage_max_sensor(sensor::Sensor *s) { cell_voltage_max_sensor_ = s; }
  void set_bypass_crc(bool bypass) { bypass_crc_ = bypass; }
  void set_baud_rate(uint32_t baud) { baud_rate_ = baud; }
  void set_dump_raw(bool dump) { dump_raw_ = dump; }

 protected:
  void handle_packet_(const uint8_t *payload, size_t length);
  void handle_message_(const uint8_t *msg, size_t length);
  void handle_read_registers_response_(const uint8_t *data, size_t length);
  
  void send_packet_(const std::vector<uint8_t> &payload);
  
  uint16_t crc16_(const uint8_t *data, size_t len);

  std::vector<uint8_t> rx_buffer_;
  
  sensor::Sensor *soc_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *temperature_min_sensor_{nullptr};
  sensor::Sensor *temperature_max_sensor_{nullptr};
  sensor::Sensor *cell_voltage_min_sensor_{nullptr};
  sensor::Sensor *cell_voltage_max_sensor_{nullptr};
  bool bypass_crc_{false};
  uint32_t baud_rate_{460800};
  bool dump_raw_{false};
};

}  // namespace hmi_bms
}  // namespace esphome
