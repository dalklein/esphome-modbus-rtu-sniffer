import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ADDRESS, CONF_ID

from . import ModbusRtuSniffer, sniffer_ns

DEPENDENCIES = ["modbus_rtu_sniffer"]

CONF_MODBUS_RTU_SNIFFER_ID = "modbus_rtu_sniffer_id"
CONF_REGISTER = "register"
CONF_VALUE_TYPE = "value_type"

# Names match modbus_controller's value_type so configs read the same way.
VALUE_TYPES = {"U_WORD": False, "S_WORD": True}

CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(CONF_MODBUS_RTU_SNIFFER_ID): cv.use_id(ModbusRtuSniffer),
        cv.Required(CONF_ADDRESS): cv.hex_uint8_t,
        cv.Required(CONF_REGISTER): cv.uint16_t,
        cv.Optional(CONF_VALUE_TYPE, default="U_WORD"): cv.enum(VALUE_TYPES, upper=True),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MODBUS_RTU_SNIFFER_ID])
    var = await sensor.new_sensor(config)
    cg.add(
        hub.add_sensor(
            config[CONF_ADDRESS],
            config[CONF_REGISTER],
            VALUE_TYPES[config[CONF_VALUE_TYPE]],
            var,
        )
    )
