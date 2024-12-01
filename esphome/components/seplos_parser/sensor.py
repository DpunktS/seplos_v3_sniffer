import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, uart
from esphome.const import CONF_ID
import logging
_LOGGER = logging.getLogger("seplos_parser")

# Benutzerdefinierte Konfigurationsoptionen
CONF_BMS_COUNT = "bms_count"
CONF_UART_ID = "uart_id"
CONF_THROTTLE_INTERVAL = "throttle_interval"

DEPENDENCIES = ["uart"]

seplos_parser_ns = cg.esphome_ns.namespace("seplos_parser")
SeplosParser = seplos_parser_ns.class_(
    "SeplosParser", cg.Component, uart.UARTDevice
)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SeplosParser),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_BMS_COUNT): cv.int_range(min=1, max=16),
    cv.Required(CONF_THROTTLE_INTERVAL): cv.positive_int,
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    _LOGGER.info("Creating SeplosParser instance...")
    try:
        # Erstellt die Instanz des SeplosParser
        var = cg.new_Pvariable(
            config[CONF_ID],
            await cg.get_variable(config[CONF_UART_ID]),
            config[CONF_BMS_COUNT],
            config[CONF_THROTTLE_INTERVAL]
        )
        cg.add(var)
        _LOGGER.info("SeplosParser instance created")
        await cg.register_component(var, config)
        _LOGGER.info("SeplosParser registered as component")
        await uart.register_uart_device(var, config[CONF_UART_ID])
        _LOGGER.info("SeplosParser linked to UART")
    except Exception as e:
        cg.esphome_ns.logger.error(f"Error in SeplosParser registration: {e}")

    for i in range(config[CONF_BMS_COUNT]):
        name_prefix = f"BMS {i + 1} "

        # Beispiel: Pack Voltage für jedes BMS
        if "pack_voltage" in config:
            sens = await sensor.new_sensor({
                "name": name_prefix + config["pack_voltage"]["name"]
            })
            cg.add(var.add_pack_voltage_sensor(sens, i))
