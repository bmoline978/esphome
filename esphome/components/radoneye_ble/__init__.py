import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32_ble_tracker
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32_ble_tracker"]
CODEOWNERS = ["@bmoline_978"]

radoneye_ble_ns = cg.esphome_ns.namespace("radoneye_ble")
RadoneyeListener = radoneye_ble_ns.class_(
    "RadoneyeListener", esp32_ble_tracker.ESPBTDeviceListener
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RadoneyeListener),
    }
).extend(esp32_ble_tracker.ESP_BLE_DEVICE_SCHEMA)


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield esp32_ble_tracker.register_ble_device(var, config)
