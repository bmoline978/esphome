import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, ble_client

from esphome.const import (
    STATE_CLASS_MEASUREMENT,
    UNIT_HECTOPASCAL,
    ICON_RADIOACTIVE,
    CONF_ID,
    CONF_RADON,
    CONF_RADON_DAY,
    CONF_RADON_LONG_TERM,
    UNIT_PICOCURIES_PER_LITER,
)

DEPENDENCIES = ["ble_client"]

radoneye_ns = cg.esphome_ns.namespace("radoneye")
RadonEye = radoneye_ns.class_(
    "RadonEye", cg.PollingComponent, ble_client.BLEClientNode
)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RadonEye),
            cv.Optional(CONF_RADON): sensor.sensor_schema(
                unit_of_measurement=UNIT_PICOCURIES_PER_LITER,
                icon=ICON_RADIOACTIVE,
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_RADON_DAY): sensor.sensor_schema(
                unit_of_measurement=UNIT_PICOCURIES_PER_LITER,
                icon=ICON_RADIOACTIVE,
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_RADON_LONG_TERM): sensor.sensor_schema(
                unit_of_measurement=UNIT_PICOCURIES_PER_LITER,
                icon=ICON_RADIOACTIVE,
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
            ),        }
    )
    .extend(cv.polling_component_schema("5min"))
    .extend(ble_client.BLE_CLIENT_SCHEMA),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    await ble_client.register_ble_node(var, config)

    if CONF_RADON in config:
        sens = await sensor.new_sensor(config[CONF_RADON])
        cg.add(var.set_radon(sens))
    if CONF_RADON_DAY in config:
        sens = await sensor.new_sensor(config[CONF_RADON_DAY])
        cg.add(var.set_radon_day(sens))
    if CONF_RADON_LONG_TERM in config:
        sens = await sensor.new_sensor(config[CONF_RADON_LONG_TERM])
        cg.add(var.set_radon_long_term(sens))
