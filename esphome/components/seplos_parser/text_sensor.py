import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import HUB_CHILD_SCHEMA, CONF_SEPLOS_PARSER_ID

DEPENDENCIES = ["seplos_parser"]

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema()
    .extend(HUB_CHILD_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    var = await text_sensor.new_text_sensor(config)

    cg.add(paren.register_text_sensor(var))
