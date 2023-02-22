#include "radoneye.h"

#ifdef USE_ESP32

namespace esphome {
namespace radoneye {

static const char *const TAG = "radoneye";

void RadonEye::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                            esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "Connected successfully!");
      }
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "Disconnected!");
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      this->handle_ = 0;
      auto chr = this->parent()->get_characteristic(service_uuid_, sensors_data_characteristic_uuid_);
      if (chr == nullptr) {
        ESP_LOGW(TAG, "No sensor characteristic found at service %s char %s", service_uuid_.to_string().c_str(),
                 sensors_data_characteristic_uuid_.to_string().c_str());
        break;
      }
      this->handle_ = chr->handle;

      this->handle24_ = 0;
      auto chr24 = this->parent()->get_characteristic(service_uuid_, sensors_data_characteristic24_uuid_);
      if (chr24 == nullptr) {
        ESP_LOGW(TAG, "No sensor characteristic found at service %s char %s", service_uuid_.to_string().c_str(),
                 sensors_data_characteristic24_uuid_.to_string().c_str());
        break;
      }
      this->handle24_ = chr24->handle;

      this->node_state = esp32_ble_tracker::ClientState::ESTABLISHED;

      request_read_values_();
      break;
    }

    case ESP_GATTC_READ_CHAR_EVT: {
      if (param->read.conn_id != this->parent()->conn_id)
        break;
      if (param->read.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error reading char at handle %d, status=%d", param->read.handle, param->read.status);
        break;
      }
      if (param->read.handle == this->handle_) {
        read_sensors_(param->read.value, param->read.value_len);
      }
      break;
    }
    default:
      break;
  }
}



void RadonEye::read_sensors_(uint8_t *raw_value, uint16_t value_len) {
    auto value = (RadonEyeReadings *) raw_value;

    if (sizeof(RadonEyeReadings) <= value_len) {
      if ((this->radon_sensor_ != nullptr) && is_valid_radon_value_(value->radon))
        this->radon_sensor_->publish_state(value->radon);

      if ((this->radon_day_sensor_ != nullptr) && is_valid_radon_value_(value->radon_day))
        this->radon_day_sensor_->publish_state(value->radon_day);

      if ((this->radon_long_term_sensor_ != nullptr) && is_valid_radon_value_(value->radon_month))
        this->radon_long_term_sensor_->publish_state(value->radon_month);
    }
   
    // This instance must not stay connected
    // so other clients can connect to it (e.g. the
    // mobile app).
    parent()->set_enabled(false);
}

bool RadonEye::is_valid_radon_value_(uint16_t radon) { return 0 <= radon && radon <= 16383; }


void RadonEye::update() {
  if (this->node_state != esp32_ble_tracker::ClientState::ESTABLISHED) {
    if (!parent()->enabled) {
      ESP_LOGW(TAG, "Reconnecting to device");
      parent()->set_enabled(true);
      parent()->connect();
    } else {
      ESP_LOGW(TAG, "Connection in progress");
    }
  }
  else
    request_read_values_();
}

void RadonEye::request_read_values_() {
  unsigned char c = 0x50;
  auto status1 =
      esp_ble_gattc_write_char(this->parent()->gattc_if, this->parent()->conn_id, this->handle24_, 1, &c, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);            
  auto status =
      esp_ble_gattc_read_char(this->parent()->gattc_if, this->parent()->conn_id, this->handle_, ESP_GATT_AUTH_REQ_NONE);
  if (status) {
    ESP_LOGW(TAG, "Error sending read request for sensor, status=%d", status);
  }
}

void RadonEye::dump_config() {
  LOG_SENSOR("  ", "Radon", this->radon_sensor_);
  LOG_SENSOR("  ", "Radon Day", this->radon_day_sensor_);
  LOG_SENSOR("  ", "Radon Long Term", this->radon_long_term_sensor_);
}

RadonEye::RadonEye()
    : PollingComponent(10000),
      service_uuid_(esp32_ble_tracker::ESPBTUUID::from_raw(SERVICE_UUID)),
      sensors_data_characteristic_uuid_(esp32_ble_tracker::ESPBTUUID::from_raw(CHARACTERISTIC_UUID)),
      sensors_data_characteristic24_uuid_(esp32_ble_tracker::ESPBTUUID::from_raw(CHARACTERISTIC24_UUID))
       {}
}  // namespace radoneye
}  // namespace esphome

#endif  // USE_ESP32
