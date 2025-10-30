#include "bh1745.h" 
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace bh1745 {

static const char *const TAG = "bh1745"; 

// BH1745 Register Addresses
static const uint8_t SYSTEM_CONTROL_ADDR = 0x40;
static const uint8_t MODE_CONTROL1_ADDR = 0x41;
static const uint8_t MODE_CONTROL2_ADDR = 0x42;
static const uint8_t RED_DATA_LSB_ADDR = 0x50;

// Configuration bits/masks
static const uint8_t SC_RESET_MASK = 0b10000000;
static const uint8_t MC2_MEASURE_BIT = 0b10000000;
static const uint8_t MC2_RGBC_EN_BIT = 0b00010000;

void BH1745::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BH1745 Color Sensor...");

  if (!this->write_byte(SYSTEM_CONTROL_ADDR, SC_RESET_MASK)) {
    this->mark_failed();
    return;
  }
  delay(10); 

  // ✅ FIX: Get the value from the TEMPLATABLE_VALUE object
  uint32_t integration_time_ms = this->integration_time_.value_or(160); 

  // Set Integration Time
  uint8_t it_reg_value = 0x00;
  if (integration_time_ms == 320) it_reg_value = 0x01;
  else if (integration_time_ms == 640) it_reg_value = 0x02;
  else if (integration_time_ms == 1280) it_reg_value = 0x03;

  if (!this->write_byte(MODE_CONTROL1_ADDR, it_reg_value)) {
    this->mark_failed();
    return;
  }

  // ✅ FIX: Get the value from the TEMPLATABLE_VALUE object
  uint8_t gain_value = this->gain_.value_or(1);
  
  // Set Gain (1x or 32x) and Enable RGBC measurement
  uint8_t gain_reg_value = (gain_value == 32) ? 0b00000010 : 0b00000000;
  uint8_t mc2_value = gain_reg_value | MC2_RGBC_EN_BIT | MC2_MEASURE_BIT;
  if (!this->write_byte(MODE_CONTROL2_ADDR, mc2_value)) {
    this->mark_failed();
    return;
  }
}

void BH1745::dump_config() {
  ESP_LOGCONFIG(TAG, "BH1745 Color Sensor:");
  
  LOG_I2C_DEVICE(LOG_LEVEL_CONFIG, this); 
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with BH1745 failed!");
  }
  
  // Use .value_or() to dump config values
  ESP_LOGCONFIG(TAG, "  Integration Time: %u ms", this->integration_time_.value_or(0));
  ESP_LOGCONFIG(TAG, "  Gain: %ux", this->gain_.value_or(0));
  
  // LOG_SENSOR requires LOG_LEVEL_CONFIG
  LOG_SENSOR(LOG_LEVEL_CONFIG, "  Red", this->red_sensor_);
  LOG_SENSOR(LOG_LEVEL_CONFIG, "  Green", this->green_sensor_);
  LOG_SENSOR(LOG_LEVEL_CONFIG, "  Blue", this->blue_sensor_);
  LOG_SENSOR(LOG_LEVEL_CONFIG, "  Illuminance (Lux)", this->illuminance_sensor_);
}

void BH1745::update() {
  if (!this->read_sensor_data_()) {
    return;
  }

  uint32_t integration_time_ms = this->integration_time_.value_or(160);
  uint8_t gain_value = this->gain_.value_or(1);

  float lux = this->calculate_lux_(this->red_value_, this->green_value_, this->blue_value_, this->clear_value_);
  
  if (this->illuminance_sensor_ != nullptr) {
    this->illuminance_sensor_->publish_state(lux);
  }
  
  float normalization_factor = (float)integration_time_ms * (float)gain_value;

  if (this->red_sensor_ != nullptr) {
    this->red_sensor_->publish_state((float)this->red_value_ / normalization_factor);
  }
  if (this->green_sensor_ != nullptr) {
    this->green_sensor_->publish_state((float)this->green_value_ / normalization_factor);
  }
  if (this->blue_sensor_ != nullptr) {
    this->blue_sensor_->publish_state((float)this->blue_value_ / normalization_factor);
  }
}

bool BH1745::read_sensor_data_() {
  uint8_t data[8];
  
  if (!this->read_bytes(RED_DATA_LSB_ADDR, data, 8)) {
    ESP_LOGW(TAG, "Read data failed!");
    return false;
  }

  this->red_value_ = data[0] | (data[1] << 8);
  this->green_value_ = data[2] | (data[3] << 8);
  this->blue_value_ = data[4] | (data[5] << 8);
  this->clear_value_ = data[6] | (data[7] << 8);

  ESP_LOGD(TAG, "Raw R:%u G:%u B:%u Clear:%u", this->red_value_, this->green_value_, this->blue_value_, this->clear_value_);

  return true;
}

float BH1745::calculate_lux_(uint16_t r, uint16_t g, uint16_t b, uint16_t clear) {
  
  // ✅ FIX: Get the values needed for normalization
  float normalization_factor = (float)this->integration_time_.value_or(160) * (float)this->gain_.value_or(1);
  if (normalization_factor == 0.0f) return 0.0f;

  float norm_r = (float)r / normalization_factor;
  float norm_g = (float)g / normalization_factor;
  float norm_b = (float)b / normalization_factor;
  float norm_c = (float)clear / normalization_factor;
  
  const float COEFFICIENT_A = 0.500f; 
  const float COEFFICIENT_B = 1.000f; 
  const float COEFFICIENT_C = -0.500f; 
  const float COEFFICIENT_D = 0.58f; 
  const float LUX_SCALE = 10000.0f; 

  float compensated_lux = (COEFFICIENT_A * norm_r) + 
                          (COEFFICIENT_B * norm_g) + 
                          (COEFFICIENT_C * norm_b) +
                          (COEFFICIENT_D * norm_c);

  float lux = compensated_lux * LUX_SCALE;
  
  return std::max(0.0f, lux);
}

}  // namespace bh1745
}  // namespace esphome