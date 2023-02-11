#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/i2c/i2c_bus.h"

namespace esphome {
namespace adafruit_soilmoisture {

/// This class implements support for the BME280 Temperature+Pressure+moisture i2c sensor.
class AdafruitSoilMoistureComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_moisture_sensor(sensor::Sensor *moisture_sensor) { moisture_sensor_ = moisture_sensor; }

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

 protected:
  void reset();
  uint16_t getCapacitance();
  esphome::i2c::ErrorCode read_seesaw(uint8_t regHigh, uint8_t regLow, uint8_t *output, uint8_t num,
                                      uint16_t delay = 100);

  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *moisture_sensor_{nullptr};
};

}  // namespace adafruit_soilmoisture
}  // namespace esphome
