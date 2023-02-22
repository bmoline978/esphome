#include "radoneye_listener.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

namespace esphome {
namespace radoneye_ble {

static const char *const TAG = "radoneye_ble";

bool RadoneyeListener::parse_device(const esp32_ble_tracker::ESPBTDevice &device) {
  ESP_LOGD(TAG, "Found device %s", device.get_name().c_str());

  if (device.get_name().rfind("FR:R20") == 0) {
    ESP_LOGD(TAG, "Found RadonEye device %s,  (MAC: %s)", device.get_name().c_str(), device.address_str().c_str());
    return true;
  }

  return false;
}

}  // namespace radoneye_ble
}  // namespace esphome

#endif
