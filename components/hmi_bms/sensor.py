import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    UNIT_AMPERE,
    UNIT_VOLT,
    UNIT_CELSIUS,
    ICON_EMPTY,
)
from . import HMIBMS, hmi_bms_ns

CONF_HMI_BMS_ID = "hmi_bms_id"
CONF_MILLIS = "millis"
CONF_SERIAL = "serial"
CONF_SYSTEM_REQUEST = "system_request"
CONF_SOC = "soc"
CONF_CURRENT_MA = "current_mA"
CONF_CHARGE_RAW = "charge_raw"
CONF_BATTERY_VOLTAGE_MV = "battery_voltage_mV"
CONF_OUTPUT_VOLTAGE_MV = "output_voltage_mV"
CONF_POS_CONTACTOR_VOLTAGE_MV = "pos_contactor_voltage_mV"
CONF_NEG_CONTACTOR_VOLTAGE_MV = "neg_contactor_voltage_mV"
CONF_TEMPERATURE_MIN_DC = "temperature_min_dC"
CONF_TEMPERATURE_MAX_DC = "temperature_max_dC"
CONF_TEMPERATURE_DELTA = "temperature_delta"
CONF_CELL_VOLTAGE_MIN_MV = "cell_voltage_min_mV"
CONF_CELL_VOLTAGE_MAX_MV = "cell_voltage_max_mV"
CONF_CELL_VOLTAGE_DELTA = "cell_voltage_delta"
CONF_CELL_VOLTAGES = "cell_voltages"
CONF_SYSTEM_SM = "system_sm"
CONF_CONTACTOR_SM = "contactor_sm"
CONF_SOC_VOLTAGE_BASED = "soc_voltage_based"
CONF_SOC_BASIC_COUNT = "soc_basic_count"
CONF_SOC_EKF = "soc_ekf"
CONF_CAPACITY_MC = "capacity_mC"
CONF_SUPPLY_VOLTAGE_3V3_MV = "supply_voltage_3V3_mV"
CONF_SUPPLY_VOLTAGE_5V_MV = "supply_voltage_5V_mV"
CONF_SUPPLY_VOLTAGE_12V_MV = "supply_voltage_12V_mV"
CONF_SUPPLY_VOLTAGE_CONTACTOR_MV = "supply_voltage_contactor_mV"
CONF_LAST_EVENT = "last_event"
CONF_HIGHEST_EVENT_LEVEL = "highest_event_level"
CONF_EVENT_HISTORY = "event_history"
CONF_MODULE_TEMPERATURES = "module_temperatures"
CONF_RAW_TEMPERATURES = "raw_temperatures"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_HMI_BMS_ID): cv.use_id(HMIBMS),
    cv.Optional(CONF_MILLIS): sensor.sensor_schema(
        unit_of_measurement="ms",
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_SERIAL): sensor.sensor_schema(
        accuracy_decimals=0,
    ),
    cv.Optional(CONF_SYSTEM_REQUEST): sensor.sensor_schema(
        accuracy_decimals=0,
    ),
    cv.Optional(CONF_SOC): sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_BATTERY,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_CURRENT_MA): sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_CURRENT,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_CHARGE_RAW): sensor.sensor_schema(
        unit_of_measurement="Ah",
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_BATTERY_VOLTAGE_MV): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_OUTPUT_VOLTAGE_MV): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_POS_CONTACTOR_VOLTAGE_MV): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_NEG_CONTACTOR_VOLTAGE_MV): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_TEMPERATURE_MIN_DC): sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_TEMPERATURE_MAX_DC): sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_TEMPERATURE_DELTA): sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_CELL_VOLTAGE_MIN_MV): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_CELL_VOLTAGE_MAX_MV): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_CELL_VOLTAGE_DELTA): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_CELL_VOLTAGES): cv.ensure_list(sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    )),
    cv.Optional(CONF_SYSTEM_SM): sensor.sensor_schema(
        accuracy_decimals=0,
    ),
    cv.Optional(CONF_CONTACTOR_SM): sensor.sensor_schema(
        accuracy_decimals=0,
    ),
    cv.Optional(CONF_SOC_VOLTAGE_BASED): sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_BATTERY,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_SOC_BASIC_COUNT): sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_BATTERY,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_SOC_EKF): sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_BATTERY,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_CAPACITY_MC): sensor.sensor_schema(
        unit_of_measurement="Ah",
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_SUPPLY_VOLTAGE_3V3_MV): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_SUPPLY_VOLTAGE_5V_MV): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_SUPPLY_VOLTAGE_12V_MV): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_SUPPLY_VOLTAGE_CONTACTOR_MV): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_LAST_EVENT): sensor.sensor_schema(
        accuracy_decimals=0,
    ),
    cv.Optional(CONF_HIGHEST_EVENT_LEVEL): sensor.sensor_schema(
        accuracy_decimals=0,
    ),
    cv.Optional(CONF_EVENT_HISTORY): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_MODULE_TEMPERATURES): cv.ensure_list(sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    )),
    cv.Optional(CONF_RAW_TEMPERATURES): cv.ensure_list(sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    )),
})

async def to_code(config):
    parent = await cg.get_variable(config[CONF_HMI_BMS_ID])

    if CONF_MILLIS in config:
        sens = await sensor.new_sensor(config[CONF_MILLIS])
        cg.add(parent.set_millis_sensor(sens))
    if CONF_SERIAL in config:
        sens = await sensor.new_sensor(config[CONF_SERIAL])
        cg.add(parent.set_serial_sensor(sens))
    if CONF_SYSTEM_REQUEST in config:
        sens = await sensor.new_sensor(config[CONF_SYSTEM_REQUEST])
        cg.add(parent.set_system_request_sensor(sens))
    if CONF_SOC in config:
        sens = await sensor.new_sensor(config[CONF_SOC])
        cg.add(parent.set_soc_sensor(sens))
    if CONF_CURRENT_MA in config:
        sens = await sensor.new_sensor(config[CONF_CURRENT_MA])
        cg.add(parent.set_current_mA_sensor(sens))
    if CONF_CHARGE_RAW in config:
        sens = await sensor.new_sensor(config[CONF_CHARGE_RAW])
        cg.add(parent.set_charge_raw_sensor(sens))
    if CONF_BATTERY_VOLTAGE_MV in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_VOLTAGE_MV])
        cg.add(parent.set_battery_voltage_mV_sensor(sens))
    if CONF_OUTPUT_VOLTAGE_MV in config:
        sens = await sensor.new_sensor(config[CONF_OUTPUT_VOLTAGE_MV])
        cg.add(parent.set_output_voltage_mV_sensor(sens))
    if CONF_POS_CONTACTOR_VOLTAGE_MV in config:
        sens = await sensor.new_sensor(config[CONF_POS_CONTACTOR_VOLTAGE_MV])
        cg.add(parent.set_pos_contactor_voltage_mV_sensor(sens))
    if CONF_NEG_CONTACTOR_VOLTAGE_MV in config:
        sens = await sensor.new_sensor(config[CONF_NEG_CONTACTOR_VOLTAGE_MV])
        cg.add(parent.set_neg_contactor_voltage_mV_sensor(sens))
    if CONF_TEMPERATURE_MIN_DC in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE_MIN_DC])
        cg.add(parent.set_temperature_min_dC_sensor(sens))
    if CONF_TEMPERATURE_MAX_DC in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE_MAX_DC])
        cg.add(parent.set_temperature_max_dC_sensor(sens))
    if CONF_TEMPERATURE_DELTA in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE_DELTA])
        cg.add(parent.set_temperature_delta_sensor(sens))
    if CONF_CELL_VOLTAGE_MIN_MV in config:
        sens = await sensor.new_sensor(config[CONF_CELL_VOLTAGE_MIN_MV])
        cg.add(parent.set_cell_voltage_min_mV_sensor(sens))
    if CONF_CELL_VOLTAGE_MAX_MV in config:
        sens = await sensor.new_sensor(config[CONF_CELL_VOLTAGE_MAX_MV])
        cg.add(parent.set_cell_voltage_max_mV_sensor(sens))
    if CONF_CELL_VOLTAGE_DELTA in config:
        sens = await sensor.new_sensor(config[CONF_CELL_VOLTAGE_DELTA])
        cg.add(parent.set_cell_voltage_delta_sensor(sens))
    
    if CONF_CELL_VOLTAGES in config:
        for conf in config[CONF_CELL_VOLTAGES]:
            sens = await sensor.new_sensor(conf)
            cg.add(parent.add_cell_voltage_sensor(sens))

    if CONF_SYSTEM_SM in config:
        sens = await sensor.new_sensor(config[CONF_SYSTEM_SM])
        cg.add(parent.set_system_sm_sensor(sens))
    if CONF_CONTACTOR_SM in config:
        sens = await sensor.new_sensor(config[CONF_CONTACTOR_SM])
        cg.add(parent.set_contactor_sm_sensor(sens))
    if CONF_SOC_VOLTAGE_BASED in config:
        sens = await sensor.new_sensor(config[CONF_SOC_VOLTAGE_BASED])
        cg.add(parent.set_soc_voltage_based_sensor(sens))
    if CONF_SOC_BASIC_COUNT in config:
        sens = await sensor.new_sensor(config[CONF_SOC_BASIC_COUNT])
        cg.add(parent.set_soc_basic_count_sensor(sens))
    if CONF_SOC_EKF in config:
        sens = await sensor.new_sensor(config[CONF_SOC_EKF])
        cg.add(parent.set_soc_ekf_sensor(sens))
    if CONF_CAPACITY_MC in config:
        sens = await sensor.new_sensor(config[CONF_CAPACITY_MC])
        cg.add(parent.set_capacity_mC_sensor(sens))
    if CONF_SUPPLY_VOLTAGE_3V3_MV in config:
        sens = await sensor.new_sensor(config[CONF_SUPPLY_VOLTAGE_3V3_MV])
        cg.add(parent.set_supply_voltage_3V3_mV_sensor(sens))
    if CONF_SUPPLY_VOLTAGE_5V_MV in config:
        sens = await sensor.new_sensor(config[CONF_SUPPLY_VOLTAGE_5V_MV])
        cg.add(parent.set_supply_voltage_5V_mV_sensor(sens))
    if CONF_SUPPLY_VOLTAGE_12V_MV in config:
        sens = await sensor.new_sensor(config[CONF_SUPPLY_VOLTAGE_12V_MV])
        cg.add(parent.set_supply_voltage_12V_mV_sensor(sens))
    if CONF_SUPPLY_VOLTAGE_CONTACTOR_MV in config:
        sens = await sensor.new_sensor(config[CONF_SUPPLY_VOLTAGE_CONTACTOR_MV])
        cg.add(parent.set_supply_voltage_contactor_mV_sensor(sens))
    if CONF_LAST_EVENT in config:
        sens = await sensor.new_sensor(config[CONF_LAST_EVENT])
        cg.add(parent.set_last_event_sensor(sens))
    if CONF_HIGHEST_EVENT_LEVEL in config:
        sens = await sensor.new_sensor(config[CONF_HIGHEST_EVENT_LEVEL])
        cg.add(parent.set_highest_event_level_sensor(sens))
    if CONF_EVENT_HISTORY in config:
        sens = await text_sensor.new_text_sensor(config[CONF_EVENT_HISTORY])
        cg.add(parent.set_event_history_text_sensor(sens))
    
    if CONF_MODULE_TEMPERATURES in config:
        for conf in config[CONF_MODULE_TEMPERATURES]:
            sens = await sensor.new_sensor(conf)
            cg.add(parent.add_module_temperature_sensor(sens))
    
    if CONF_RAW_TEMPERATURES in config:
        for conf in config[CONF_RAW_TEMPERATURES]:
            sens = await sensor.new_sensor(conf)
            cg.add(parent.add_raw_temperature_sensor(sens))
