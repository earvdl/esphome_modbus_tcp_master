import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_OFFSET,
    CONF_REGISTER_ADDRESS,
    CONF_UPDATE_INTERVAL,
)

from . import modbus_tcp_ns, ModbusTCPManager

# Local config keys for this component
CONF_MODBUS_TCP_ID = "modbus_tcp_id"
CONF_FUNCTION_CODE = "function_code"
CONF_SCALE = "scale"
CONF_VALUE_TYPE = "value_type"

ModbusTCPAdvancedSensor = modbus_tcp_ns.class_(
    "ModbusTCPAdvancedSensor", sensor.Sensor, cg.PollingComponent
)

ModbusValueType = modbus_tcp_ns.enum("ModbusValueType")

VALUE_TYPE_MAP = {
    "u16": ModbusValueType.U16,
    "s16": ModbusValueType.S16,
    "u32_be": ModbusValueType.U32_BE,
    "s32_be": ModbusValueType.S32_BE,
    "u32_le": ModbusValueType.U32_LE,
    "s32_le": ModbusValueType.S32_LE,
}

CONFIG_SCHEMA = (
    sensor.sensor_schema(ModbusTCPAdvancedSensor)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(ModbusTCPAdvancedSensor),
            cv.GenerateID(CONF_MODBUS_TCP_ID): cv.use_id(ModbusTCPManager),
            cv.Required(CONF_REGISTER_ADDRESS): cv.int_range(min=0, max=0xFFFF),
            cv.Optional(CONF_FUNCTION_CODE, default=3): cv.one_of(3, 4, int=True),
            cv.Optional(CONF_SCALE, default=1.0): cv.float_,
            cv.Optional(CONF_OFFSET, default=0.0): cv.float_,
            cv.Optional(CONF_VALUE_TYPE, default="s16"): cv.enum(VALUE_TYPE_MAP, lower=True),
            cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
        }
    )
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MODBUS_TCP_ID])

    var = cg.new_Pvariable(
        config[CONF_ID],
        parent,
        config[CONF_REGISTER_ADDRESS],
        config[CONF_FUNCTION_CODE],
        config[CONF_SCALE],
        config[CONF_OFFSET],
        config[CONF_UPDATE_INTERVAL].total_milliseconds,  # uint32_t expected by C++
        config[CONF_VALUE_TYPE],
    )

    await sensor.register_sensor(var, config)
    await cg.register_component(var, config)

