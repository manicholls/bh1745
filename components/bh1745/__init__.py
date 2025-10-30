import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID

bh1745_ns = cg.esphome_ns.namespace('bh1745')
BH1745 = bh1745_ns.class_('BH1745', cg.Component, i2c.I2CDevice)

CONF_INTEGRATION_TIME = "integration_time"
CONF_GAIN = "gain"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BH1745),
    
    cv.Optional(CONF_INTEGRATION_TIME, default='160ms'): 
        # ✅ FIX: Use lambda to ensure the integer millisecond value is used for cv.Range
        cv.All(cv.positive_time_period_milliseconds, 
               lambda value: value.total_milliseconds, 
               cv.Range(min=160, max=1280)),
    
    cv.Optional(CONF_GAIN, default='1x'): cv.one_of('1x', '32x'),
    
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x38))


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)

    integration_time_ms = config[CONF_INTEGRATION_TIME]
    cg.add(var.set_integration_time(integration_time_ms))

    if config[CONF_GAIN] == '32x':
        cg.add(var.set_gain(32))
    else:
        cg.add(var.set_gain(1))