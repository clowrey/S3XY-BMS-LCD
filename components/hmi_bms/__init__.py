import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

hmi_bms_ns = cg.esphome_ns.namespace("hmi_bms")
HMIBMS = hmi_bms_ns.class_("HMIBMS", cg.PollingComponent, uart.UARTDevice)

CONF_BYPASS_CRC = "bypass_crc"
CONF_BAUD_RATE = "baud_rate"
CONF_DUMP_RAW = "dump_raw"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HMIBMS),
    cv.Optional(CONF_BYPASS_CRC, default=False): cv.boolean,
    cv.Optional(CONF_BAUD_RATE, default=460800): cv.positive_int,
    cv.Optional(CONF_DUMP_RAW, default=False): cv.boolean,
}).extend(cv.polling_component_schema("60s")).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_bypass_crc(config[CONF_BYPASS_CRC]))
    cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))
    cg.add(var.set_dump_raw(config[CONF_DUMP_RAW]))
