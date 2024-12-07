import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import UNIT_VOLT, UNIT_AMPERE, ICON_EMPTY
from . import HUB_CHILD_SCHEMA, CONF_SEPLOS_PARSER_ID

DEPENDENCIES = ["seplos_parser"]

CONFIG_SCHEMA = (
    sensor.sensor_schema(
      unit_of_measurement=UNIT_VOLT, icon=ICON_EMPTY, accuracy_decimals=2
    )
    .extend(HUB_CHILD_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    var = await sensor.new_sensor(config)

    cg.add(paren.register_sensor(var))
