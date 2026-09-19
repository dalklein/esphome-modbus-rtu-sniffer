import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@dalklein"]
DEPENDENCIES = ["uart"]
# AUTO_LOAD, not DEPENDENCIES: this uses modbus's frame-geometry helpers
# (modbus_helpers.h) but needs no `modbus:` block of its own, and a sniffer user should
# not have to configure one. The helpers are mostly header-inline, and unused modbus code
# is dropped by --gc-sections.
AUTO_LOAD = ["modbus"]

sniffer_ns = cg.esphome_ns.namespace("modbus_rtu_sniffer")
ModbusRtuSniffer = sniffer_ns.class_("ModbusRtuSniffer", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ModbusRtuSniffer),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
