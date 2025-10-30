import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import CONF_ID, CONF_RED, CONF_GREEN, CONF_BLUE, CONF_ILLUMINANCE, UNIT_LUX, ICON_BRIGHTNESS_5, CONF_ADDRESS

# Define the C++ namespace for your component
bh1745_ns = cg.esphome_ns.namespace('bh1745_color')
BH1745Color = bh1745_ns.class_('BH1745Color', cg.Component, i2c.I2CDevice)

# Define configuration keys for the YAML file
CONF_INTEGRATION_TIME = "integration_time"
CONF_GAIN = "gain"

# Schema for the RGB and Illuminance sensors
SENSOR_SCHEMA = sensor.sensor_schema(UNIT_LUX, ICON_BRIGHTNESS_5, 1)

# Main component configuration schema
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BH1745Color),
    cv.Optional(CONF_INTEGRATION_TIME, default='160ms'): cv.All(cv.TimePeriod.from_milliseconds, cv.Range(min=160, max=1280)),
    cv.Optional(CONF_GAIN, default='1x'): cv.one_of('1x', '32x'),
    cv.Optional(CONF_RED): SENSOR_SCHEMA,
    cv.Optional(CONF_GREEN): SENSOR_SCHEMA,
    cv.Optional(CONF_BLUE): SENSOR_SCHEMA,
    cv.Optional(CONF_ILLUMINANCE): SENSOR_SCHEMA,
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x38)) # Default I2C address is 0x38

def to_code(config):
    # Instantiate the C++ class
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    # The i2c.register_i2c_device handles setting the address and bus in the C++ object
    yield i2c.register_i2c_device(var, config)

    # Convert TimePeriod to milliseconds integer for C++
    cg.add(var.set_integration_time(config[CONF_INTEGRATION_TIME]))

    # Set gain
    if config[CONF_GAIN] == '32x':
        cg.add(var.set_gain(32))
    else:
        cg.add(var.set_gain(1))

    # Generate code for each sensor and set it in the C++ component
    if CONF_RED in config:
        sens = yield sensor.new_sensor(config[CONF_RED])
        cg.add(var.set_red_sensor(sens))
    if CONF_GREEN in config:
        sens = yield sensor.new_sensor(config[CONF_GREEN])
        cg.add(var.set_green_sensor(sens))
    if CONF_BLUE in config:
        sens = yield sensor.new_sensor(config[CONF_BLUE])
        cg.add(var.set_blue_sensor(sens))
    if CONF_ILLUMINANCE in config:
        sens = yield sensor.new_sensor(config[CONF_ILLUMINANCE])
        cg.add(var.set_illuminance_sensor(sens))
