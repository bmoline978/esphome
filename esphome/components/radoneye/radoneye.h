#pragma once

#ifdef USE_ESP32

#include <esp_gattc_api.h>
#include <algorithm>
#include <iterator>
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace esphome {
namespace radoneye {

static const char *const SERVICE_UUID = "00001523-1212-efde-1523-785feabcd123";
static const char *const CHARACTERISTIC_UUID = "00001525-1212-efde-1523-785feabcd123";
static const char *const CHARACTERISTIC24_UUID = "00001524-1212-efde-1523-785feabcd123";

class RadonEye : public PollingComponent, public ble_client::BLEClientNode {
 public:
  RadonEye();

  void dump_config() override;
  void update() override;

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  void set_radon(sensor::Sensor *radon) { radon_sensor_ = radon; }
  void set_radon_day(sensor::Sensor *radon_day) { radon_day_sensor_ = radon_day; }
  void set_radon_long_term(sensor::Sensor *radon_long_term) { radon_long_term_sensor_ = radon_long_term; }

 protected:
  bool is_valid_radon_value_(uint16_t radon);

  void read_sensors_(uint8_t *value, uint16_t value_len);
  void request_read_values_();
  void trigger_reading_();

  sensor::Sensor *radon_sensor_{nullptr};
  sensor::Sensor *radon_day_sensor_{nullptr};
  sensor::Sensor *radon_long_term_sensor_{nullptr};

  uint16_t handle_;
  uint16_t handle24_;
  esp32_ble_tracker::ESPBTUUID service_uuid_;
  esp32_ble_tracker::ESPBTUUID sensors_data_characteristic_uuid_;
  esp32_ble_tracker::ESPBTUUID sensors_data_characteristic24_uuid_;

// In order to ensure that radon readings are in the correct byte position, the structure must be packed
#pragma pack(push, 1)
  struct RadonEyeReadings {
    uint16_t unknown;
    float radon;
    float radon_day;
    float radon_month;
    };
#pragma pack(pop)
};

}  // namespace radoneye
}  // namespace esphome

#endif  // USE_ESP32
