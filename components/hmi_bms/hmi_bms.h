#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace hmi_bms {

enum HMIMsgType {
  HMI_MSG_ANNOUNCE_DEVICE = 0x81,
  HMI_MSG_SET_DEVICE_ADDRESS = 0x01,
  HMI_MSG_WRITE_REGISTERS = 0x03,
  HMI_MSG_READ_REGISTERS = 0x04,
  HMI_MSG_READ_REGISTERS_RESPONSE = 0x84,
  HMI_MSG_READ_CELL_VOLTAGES = 0x05,
  HMI_MSG_READ_CELL_VOLTAGES_RESPONSE = 0x85,
  HMI_MSG_READ_EVENTS = 0x06,
  HMI_MSG_READ_EVENTS_RESPONSE = 0x86,
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
  HMI_REG_SYSTEM_REQUEST = 14,
  HMI_REG_SYSTEM_STATE = 15,
  HMI_REG_CONTACTORS_STATE = 16,
  HMI_REG_SOC_VOLTAGE_BASED = 17,
  HMI_REG_SOC_BASIC_COUNT = 18,
  HMI_REG_SOC_EKF = 19,
  HMI_REG_CAPACITY = 20,
  HMI_REG_SUPPLY_VOLTAGE_3V3 = 21,
  HMI_REG_SUPPLY_VOLTAGE_5V = 22,
  HMI_REG_SUPPLY_VOLTAGE_12V = 23,
  HMI_REG_SUPPLY_VOLTAGE_CTR = 24,
  HMI_REG_CELL_VOLTAGES_START = 0x100,
  HMI_REG_CELL_VOLTAGES_END = 0x1FF,
  HMI_REG_MODULE_TEMPS_START = 0x200,
  HMI_REG_MODULE_TEMPS_END = 0x207,
  HMI_REG_RAW_TEMPS_START = 0x208,
  HMI_REG_RAW_TEMPS_END = 0x238,
};

class HMIBMS : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_millis_sensor(sensor::Sensor *s) { millis_sensor_ = s; }
  void set_serial_sensor(sensor::Sensor *s) { serial_sensor_ = s; }
  void set_system_request_sensor(sensor::Sensor *s) { system_request_sensor_ = s; }
  void set_soc_sensor(sensor::Sensor *s) { soc_sensor_ = s; }
  void set_current_mA_sensor(sensor::Sensor *s) { current_mA_sensor_ = s; }
  void set_charge_raw_sensor(sensor::Sensor *s) { charge_raw_sensor_ = s; }
  void set_battery_voltage_mV_sensor(sensor::Sensor *s) { battery_voltage_mV_sensor_ = s; }
  void set_output_voltage_mV_sensor(sensor::Sensor *s) { output_voltage_mV_sensor_ = s; }
  void set_pos_contactor_voltage_mV_sensor(sensor::Sensor *s) { pos_contactor_voltage_mV_sensor_ = s; }
  void set_neg_contactor_voltage_mV_sensor(sensor::Sensor *s) { neg_contactor_voltage_mV_sensor_ = s; }
  void set_temperature_min_dC_sensor(sensor::Sensor *s) { temperature_min_dC_sensor_ = s; }
  void set_temperature_max_dC_sensor(sensor::Sensor *s) { temperature_max_dC_sensor_ = s; }
  void set_temperature_delta_sensor(sensor::Sensor *s) { temperature_delta_sensor_ = s; }
  void set_cell_voltage_min_mV_sensor(sensor::Sensor *s) { cell_voltage_min_mV_sensor_ = s; }
  void set_cell_voltage_max_mV_sensor(sensor::Sensor *s) { cell_voltage_max_mV_sensor_ = s; }
  void set_cell_voltage_delta_sensor(sensor::Sensor *s) { cell_voltage_delta_sensor_ = s; }
  void set_system_sm_sensor(sensor::Sensor *s) { system_sm_sensor_ = s; }
  void set_contactor_sm_sensor(sensor::Sensor *s) { contactor_sm_sensor_ = s; }
  void set_soc_voltage_based_sensor(sensor::Sensor *s) { soc_voltage_based_sensor_ = s; }
  void set_soc_basic_count_sensor(sensor::Sensor *s) { soc_basic_count_sensor_ = s; }
  void set_soc_ekf_sensor(sensor::Sensor *s) { soc_ekf_sensor_ = s; }
  void set_capacity_mC_sensor(sensor::Sensor *s) { capacity_mC_sensor_ = s; }
  void set_supply_voltage_3V3_mV_sensor(sensor::Sensor *s) { supply_voltage_3V3_mV_sensor_ = s; }
  void set_supply_voltage_5V_mV_sensor(sensor::Sensor *s) { supply_voltage_5V_mV_sensor_ = s; }
  void set_supply_voltage_12V_mV_sensor(sensor::Sensor *s) { supply_voltage_12V_mV_sensor_ = s; }
  void set_supply_voltage_contactor_mV_sensor(sensor::Sensor *s) { supply_voltage_contactor_mV_sensor_ = s; }
  void set_last_event_sensor(sensor::Sensor *s) { last_event_sensor_ = s; }
  void set_highest_event_level_sensor(sensor::Sensor *s) { highest_event_level_sensor_ = s; }
  void set_event_history_text_sensor(text_sensor::TextSensor *s) { event_history_text_sensor_ = s; }
  void add_cell_voltage_sensor(sensor::Sensor *s) { cell_voltage_sensors_.push_back(s); }
  void add_module_temperature_sensor(sensor::Sensor *s) { module_temperature_sensors_.push_back(s); }
  void add_raw_temperature_sensor(sensor::Sensor *s) { raw_temperature_sensors_.push_back(s); }
  const std::vector<sensor::Sensor *> &get_cell_voltage_sensors() const { return cell_voltage_sensors_; }
  void set_bypass_crc(bool bypass) { bypass_crc_ = bypass; }
  void set_baud_rate(uint32_t baud) { baud_rate_ = baud; }
  void set_dump_raw(bool dump) { dump_raw_ = dump; }
  void send_system_request(uint8_t request);

 protected:
  void handle_packet_(const uint8_t *payload, size_t length);
  void handle_message_(const uint8_t *msg, size_t length);
  void handle_read_registers_response_(const uint8_t *data, size_t length);
  void handle_read_cell_voltages_response_(const uint8_t *data, size_t length);
  void handle_read_events_response_(const uint8_t *data, size_t length);
  
  void send_packet_(const std::vector<uint8_t> &payload);
  
  uint16_t crc16_modbus_(const uint8_t *data, size_t len);

  uint8_t address_{0x00};
  uint64_t serial_{0};
  uint32_t update_count_{0};
  uint8_t temp_poll_index_{0};
  std::vector<uint8_t> rx_buffer_;
  
  sensor::Sensor *millis_sensor_{nullptr};
  sensor::Sensor *serial_sensor_{nullptr};
  sensor::Sensor *system_request_sensor_{nullptr};
  sensor::Sensor *soc_sensor_{nullptr};
  sensor::Sensor *current_mA_sensor_{nullptr};
  sensor::Sensor *charge_raw_sensor_{nullptr};
  sensor::Sensor *battery_voltage_mV_sensor_{nullptr};
  sensor::Sensor *output_voltage_mV_sensor_{nullptr};
  sensor::Sensor *pos_contactor_voltage_mV_sensor_{nullptr};
  sensor::Sensor *neg_contactor_voltage_mV_sensor_{nullptr};
  sensor::Sensor *temperature_min_dC_sensor_{nullptr};
  sensor::Sensor *temperature_max_dC_sensor_{nullptr};
  sensor::Sensor *temperature_delta_sensor_{nullptr};
  sensor::Sensor *cell_voltage_min_mV_sensor_{nullptr};
  sensor::Sensor *cell_voltage_max_mV_sensor_{nullptr};
  sensor::Sensor *cell_voltage_delta_sensor_{nullptr};
  sensor::Sensor *system_sm_sensor_{nullptr};
  sensor::Sensor *contactor_sm_sensor_{nullptr};
  sensor::Sensor *soc_voltage_based_sensor_{nullptr};
  sensor::Sensor *soc_basic_count_sensor_{nullptr};
  sensor::Sensor *soc_ekf_sensor_{nullptr};
  sensor::Sensor *capacity_mC_sensor_{nullptr};
  sensor::Sensor *supply_voltage_3V3_mV_sensor_{nullptr};
  sensor::Sensor *supply_voltage_5V_mV_sensor_{nullptr};
  sensor::Sensor *supply_voltage_12V_mV_sensor_{nullptr};
  sensor::Sensor *supply_voltage_contactor_mV_sensor_{nullptr};
  sensor::Sensor *last_event_sensor_{nullptr};
  sensor::Sensor *highest_event_level_sensor_{nullptr};
  text_sensor::TextSensor *event_history_text_sensor_{nullptr};
  std::vector<sensor::Sensor *> cell_voltage_sensors_;
  std::vector<sensor::Sensor *> module_temperature_sensors_;
  std::vector<sensor::Sensor *> raw_temperature_sensors_;
  bool bypass_crc_{false};
  uint32_t baud_rate_{460800};
  bool dump_raw_{false};

  struct SensorUpdate {
    sensor::Sensor *sensor;
    float state;
  };
  std::vector<SensorUpdate> publish_queue_;
};

}  // namespace hmi_bms
}  // namespace esphome
