#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace bh1745 {

class BH1745 : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_red_sensor(sensor::Sensor *s) { this->red_sensor_ = s; }
  void set_green_sensor(sensor::Sensor *s) { this->green_sensor_ = s; }
  void set_blue_sensor(sensor::Sensor *s) { this->blue_sensor_ = s; }
  void set_illuminance_sensor(sensor::Sensor *s) { this->illuminance_sensor_ = s; }
  
  // ✅ FIX: Use simple setters to avoid the TEMPLATABLE_VALUE macro issue
  void set_integration_time(uint32_t integration_time_ms) { this->integration_time_ms_ = integration_time_ms; }
  void set_gain(uint8_t gain) { this->gain_ = gain; }

  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

 protected:
  bool read_sensor_data_();
  float calculate_lux_(uint16_t r, uint16_t g, uint16_t b, uint16_t clear);

  uint32_t integration_time_ms_; // Now stored as a simple variable
  uint8_t gain_;                 // Now stored as a simple variable
  uint16_t red_value_{0};
  uint16_t green_value_{0};
  uint16_t blue_value_{0};
  uint16_t clear_value_{0};

  sensor::Sensor *red_sensor_{nullptr};
  sensor::Sensor *green_sensor_{nullptr};
  sensor::Sensor *blue_sensor_{nullptr};
  sensor::Sensor *illuminance_sensor_{nullptr};
};

}  // namespace bh1745
}  // namespace esphome