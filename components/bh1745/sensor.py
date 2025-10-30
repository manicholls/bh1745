import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_RED,
    CONF_GREEN,
    CONF_BLUE,
    CONF_ILLUMINANCE,
    UNIT_LUX,
    ICON_BRIGHTNESS_5,
    CONF_ID
)

from . import BH1745, bh1745_ns

DEPENDENCIES = ['i2c']
TYPES = {
    CONF_RED: "set_red_sensor",
    CONF_GREEN: "set_green_sensor",
    CONF_BLUE: "set_blue_sensor",
    CONF_ILLUMINANCE: "set_illuminance_sensor",
}

BH1745_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_LUX,
    icon=ICON_BRIGHTNESS_5,
    accuracy_decimals=1,
)

CONFIG_SCHEMA = cv.Schema({
    # FIX: Require the ID to link this sensor platform to the base component
    cv.Required(CONF_ID): cv.use_id(BH1745), 

    cv.Optional(CONF_RED): BH1745_SENSOR_SCHEMA,
    cv.Optional(CONF_GREEN): BH1745_SENSOR_SCHEMA,
    cv.Optional(CONF_BLUE): BH1745_SENSOR_SCHEMA,
    cv.Optional(CONF_ILLUMINANCE): BH1745_SENSOR_SCHEMA,
}).extend(cv.COMPONENT_SCHEMA)


def to_code(config):
    # Retrieve the main BH1745 component object using the configured ID
    paren = yield cg.get_variable(config[CONF_ID])

    for type, setter in TYPES.items():
        if type in config:
            conf = config[type]
            sens = yield sensor.new_sensor(conf)
            cg.add(getattr(paren, setter)(sens))