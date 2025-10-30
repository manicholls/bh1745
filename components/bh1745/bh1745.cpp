#include "bh1745.h" 
#include "esphome/core/log.h"
#include "esphome/core/helpers.h" // Keep this one, it's needed for other macros like LOG_SENSOR

namespace esphome {
namespace bh1745 {

static const char *const TAG = "bh1745"; 
static const uint8_t SYSTEM_CONTROL_ADDR = 0x40;
static const uint8_t MODE_CONTROL1_ADDR = 0x41; // <-- This was the missing constant!
static const uint8_t MODE_CONTROL2_ADDR = 0x42;
static const uint8_t RED_DATA_LSB_ADDR = 0x50;

// Configuration bits/masks
static const uint8_t SC_RESET_MASK = 0b10000000;
static const uint8_t MC2_MEASURE_BIT = 0b10000000;
static const uint8_t MC2_RGBC_EN_BIT = 0b00010000;
// ... register definitions ...

void BH1745::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BH1745 Color Sensor...");
  // ... i2c check ...
  delay(10); 

  // ✅ FIX: Access the simple member variable
  uint8_t it_reg_value = 0x00;
  if (this->integration_time_ms_ == 320) it_reg_value = 0x01;
  else if (this->integration_time_ms_ == 640) it_reg_value = 0x02;
  else if (this->integration_time_ms_ == 1280) it_reg_value = 0x03;

  if (!this->write_byte(MODE_CONTROL1_ADDR, it_reg_value)) {
    this->mark_failed();
    return;
  }

  // ✅ FIX: Access the simple member variable
  uint8_t gain_reg_value = (this->gain_ == 32) ? 0b00000010 : 0b00000000;
  uint8_t mc2_value = gain_reg_value | MC2_RGBC_EN_BIT | MC2_MEASURE_BIT;
  // ... rest of setup ...
}

void BH1745::dump_config() {
  ESP_LOGCONFIG(TAG, "BH1745 Color Sensor:");
  
  // LOG_I2C_DEVICE still requires only the component pointer for your version
  LOG_I2C_DEVICE(this); 
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with BH1745 failed!");
  }
  
  // Access the simple member variables
  ESP_LOGCONFIG(TAG, "  Integration Time: %u ms", this->integration_time_ms_);
  ESP_LOGCONFIG(TAG, "  Gain: %ux", this->gain_);
  
  // ✅ FINAL, DEFINITIVE FIX: Use the simplest two-argument structure that does not clash with the level's integer type.
  //    The descriptive string acts as the first argument, and the sensor pointer is the second.
  LOG_SENSOR("  Red", this->red_sensor_);
  LOG_SENSOR("  Green", this->green_sensor_);
  LOG_SENSOR("  Blue", this->blue_sensor_);
  LOG_SENSOR("  Illuminance (Lux)", this->illuminance_sensor_);
}

void BH1745::update() {
  // ... read data check ...

  float lux = this->calculate_lux_(this->red_value_, this->green_value_, this->blue_value_, this->clear_value_);
  // ... publish illuminance ...
  
  // ✅ FIX: Access the simple member variable
  float normalization_factor = (float)this->integration_time_ms_ * (float)this->gain_;
  // ... publish RGB sensors ...
}

float BH1745::calculate_lux_(uint16_t r, uint16_t g, uint16_t b, uint16_t clear) {
  
  // ✅ FIX: Access the simple member variable
  float normalization_factor = (float)this->integration_time_ms_ * (float)this->gain_;
  // ... rest of calculation ...
}

}  // namespace bh1745
}  // namespace esphome