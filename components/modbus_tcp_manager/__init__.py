import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

# Configuration constants
CONF_HOST = "host"
CONF_PORT = "port"
CONF_UNIT_ID = "unit_id"
CONF_WATCHDOG_REGISTER = "watchdog_register"
CONF_WATCHDOG_INTERVAL = "watchdog_interval"
CONF_SAFE_MODE_REGISTERS = "safe_mode_registers"

# Shared constants used by sensor.py
CONF_MODBUS_TCP_ID = "modbus_tcp_id"
CONF_REGISTER_ADDRESS = "register_address"
CONF_FUNCTION_CODE = "function_code"

# Namespace
modbus_tcp_ns = cg.esphome_ns.namespace("modbus_tcp")
ModbusTCPManager = modbus_tcp_ns.class_("ModbusTCPManager", cg.Component)

DEPENDENCIES = ["network"]
CODEOWNERS = ["@Gucioo"]

SAFE_MODE_REGISTER_SCHEMA = cv.Schema({
    cv.Required("register"): cv.positive_int,
    cv.Required("value"): cv.int_range(min=-32768, max=32767),
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ModbusTCPManager),
    cv.Required(CONF_HOST): cv.string,
    cv.Optional(CONF_PORT, default=502): cv.port,
    cv.Optional(CONF_UNIT_ID, default=1): cv.int_range(min=1, max=255),
    cv.Optional(CONF_WATCHDOG_REGISTER): cv.positive_int,
    cv.Optional(CONF_WATCHDOG_INTERVAL): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_SAFE_MODE_REGISTERS, default=[]): cv.All(cv.ensure_list(SAFE_MODE_REGISTER_SCHEMA)),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_HOST],
        config[CONF_PORT],
        config[CONF_UNIT_ID]
    )

    if CONF_WATCHDOG_REGISTER in config:
        cg.add(var.set_watchdog_register(config[CONF_WATCHDOG_REGISTER]))
        if CONF_WATCHDOG_INTERVAL in config:
            cg.add(var.set_watchdog_interval(config[CONF_WATCHDOG_INTERVAL]))

    for safe_reg in config[CONF_SAFE_MODE_REGISTERS]:
        cg.add(var.add_safe_mode_register(safe_reg["register"], safe_reg["value"]))

    await cg.register_component(var, config)
