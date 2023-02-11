#include "adafruit_soilmoisture.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#define SEESAW_HW_ID_CODE 0x55  ///< seesaw HW ID code

namespace esphome {
namespace adafruit_soilmoisture {

using namespace esphome::i2c;

/** Module Base Addreses
 *  The module base addresses for different seesaw modules.
 */
enum {
  SEESAW_STATUS_BASE = 0x00,
  SEESAW_TOUCH_BASE = 0x0F,
};

/** status module function address registers
 */
enum {
  SEESAW_STATUS_HW_ID = 0x01,
  SEESAW_STATUS_VERSION = 0x02,
  SEESAW_STATUS_OPTIONS = 0x03,
  SEESAW_STATUS_TEMP = 0x04,
  SEESAW_STATUS_SWRST = 0x7F,
};

enum {
  SEESAW_TOUCH_CHANNEL_OFFSET = 0x10,
};

static const char *const TAG = "adafruit_soilmoisture.sensor";

void AdafruitSoilMoistureComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Adafruit Soil Moisture Sensor...");

  // Mark as not failed before initializing. Some devices will turn off sensors to save on batteries
  // and when they come back on, the COMPONENT_STATE_FAILED bit must be unset on the component.
  if ((this->component_state_ & COMPONENT_STATE_MASK) == COMPONENT_STATE_FAILED) {
    this->component_state_ &= ~COMPONENT_STATE_MASK;
    this->component_state_ |= COMPONENT_STATE_CONSTRUCTION;
  }

  this->reset();

  delay(500);

  uint8_t c = 0;
  ErrorCode err = this->read_seesaw(0x00, 0x01, &c, 1);
  if (err != ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read hardware id. Error: 0x%x", err);
    this->mark_failed();
  }
  if (c != SEESAW_HW_ID_CODE) {
    ESP_LOGE(TAG, "Unknown hardware type 0x%x. Expected: 0x%x", c, SEESAW_HW_ID_CODE);
    this->mark_failed();
  }
}
void AdafruitSoilMoistureComponent::dump_config() {}
float AdafruitSoilMoistureComponent::get_setup_priority() const { return setup_priority::AFTER_CONNECTION; }

void AdafruitSoilMoistureComponent::reset() {
  uint8_t message[3];
  message[0] = SEESAW_STATUS_BASE;
  message[1] = SEESAW_STATUS_SWRST;
  message[2] = 0xff;

  this->write(message, 3);

  delay(500);
}

void AdafruitSoilMoistureComponent::update() {
  // Enable sensor
  if (this->temperature_sensor_ != nullptr) {
    uint8_t buf[4];
    ErrorCode err = this->read_seesaw(0x00, 0x04, buf, 4);
    if (err != ERROR_OK) {
      ESP_LOGE(TAG, "Failed to read temp sensor. code: %d", err);
    }
    int32_t ret = ((uint32_t) buf[0] << 24) | ((uint32_t) buf[1] << 16) | ((uint32_t) buf[2] << 8) | (uint32_t) buf[3];
    float value = (1.0 / (1UL << 16)) * ret;
    this->temperature_sensor_->publish_state(value);
  }
  if (this->moisture_sensor_ != nullptr) {
    uint16_t ret = this->getCapacitance();
    this->moisture_sensor_->publish_state(ret);
  }
}

uint16_t AdafruitSoilMoistureComponent::getCapacitance() {
  uint8_t buf[2];
  uint16_t ret = 65535;
  uint16_t retry = 0;
  for (; retry < 5; retry++) {
    if (ERROR_OK == this->read_seesaw(SEESAW_TOUCH_BASE, SEESAW_TOUCH_CHANNEL_OFFSET, buf, 2, 3000 + retry * 1000)) {
      ret = ((uint16_t) buf[0] << 8) | buf[1];
      break;
    }
  }
  ESP_LOGI(TAG, "Took %d reads to get a value", retry);
  return ret;
}

ErrorCode AdafruitSoilMoistureComponent::read_seesaw(uint8_t regHigh, uint8_t regLow, uint8_t *output, uint8_t num,
                                                     uint16_t delay) {
  uint8_t outMessage[2];
  outMessage[0] = regHigh;
  outMessage[1] = regLow;
  esphome::i2c::ErrorCode err = this->write(outMessage, 2);
  if (err != ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write message. Code: %d", err);
    return err;
  }
  delayMicroseconds(delay);

  return this->read(output, num);
}

}  // namespace adafruit_soilmoisture
}  // namespace esphome
