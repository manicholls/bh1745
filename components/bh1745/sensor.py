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

# Import the parent component class from __init__.py
from . import BH1745, bh1745_ns

DEPENDENCIES = ['i2c']
TYPES = {
    CONF_RED: "set_red_sensor",
    CONF_GREEN: "set_green_sensor",
    CONF_BLUE: "set_blue_sensor",
    CONF_ILLUMINANCE: "set_illuminance_sensor",
}

# Standard sensor schema for all channels (using LUX unit, 1 decimal place)
BH1745_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_LUX,
    icon=ICON_BRIGHTNESS_5,
    accuracy_decimals=1,
)

# Combined schema for all sensors, linked to the main component ID
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(BH1745), # Reference the main BH1745 component instance

    cv.Optional(CONF_RED): BH1745_SENSOR_SCHEMA,
    cv.Optional(CONF_GREEN): BH1745_SENSOR_SCHEMA,
    cv.Optional(CONF_BLUE): BH1745_SENSOR_SCHEMA,
    
    # Illuminance uses the same schema, but is the calculated Lux value
    cv.Optional(CONF_ILLUMINANCE): BH1745_SENSOR_SCHEMA,
}).extend(cv.COMPONENT_SCHEMA)


def to_code(config):
    # Retrieve the main BH1745 component object
    paren = yield cg.get_variable(config[CONF_ID])

    # Iterate over the defined sensor types and register each one
    for type, setter in TYPES.items():
        if type in config:
            conf = config[type]
            # Create a new sensor object
            sens = yield sensor.new_sensor(conf)
            # Call the corresponding setter method on the BH1745 component instance
            cg.add(getattr(paren, setter)(sens))