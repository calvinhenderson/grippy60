#define DT_DRV_COMPAT zmk_behavior_imu_mouse

#include <zmk-imu-mouse/config.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(DT_DRV_COMPAT, CONFIG_ZMK_LOG_LEVEL);

#define BEHAVIOR_IMU_MOUSE_INACTIVE 0x01
#define BEHAVIOR_IMU_MOUSE_MOVING 0x02
#define BEHAVIOR_IMU_MOUSE_SCROLLING 0x04

struct behavior_imu_mouse_config {
    struct device *sensor;
    struct sensor_trigger *trigger;

    // Configuration
    uint8_t status;
    uint8_t sensor_axis[2];
    int32_t scale_multiplier;
    int32_t scale_divisor;
};

static void sensor_callback_handler(const struct device *sensor,
                                    const struct sensor_trigger *trigger) {
    const struct behavior_imu_mouse_config *config =
        CONTAINER_OF(trigger, struct behavior_imu_mouse_config, trigger);
    const struct device *dev = CONTAINER_OF(config, struct device, config);

    if (config->trigger) {
        LOG_DBG("Sensor triggered for %s", dev->name);
    } else {
        LOG_WRN("Trigger callback not resolved for %s", dev->name);
    }
}

static int behavior_imu_mouse_init(const struct device *dev) {
    const struct behavior_imu_mouse_config *config = dev->config;
    struct sensor_trigger *trig = config->trigger;

    if (!device_is_ready(config->sensor)) {
        LOG_ERR("Sensor device %s is not ready", config->sensor->name);
        return -ENODEV;
    }

    config->trigger = (struct sensor_trigger *)k_malloc(sizeof(struct sensor_trigger));
    if (!config->trigger) {
        LOG_ERR("Failed to initialize sensor trigger for %s on %s", config->sensor->name,
                dev->name);
        return -ENOMEM;
    }

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_IMU_MOUSE_ENABLE_TIMER)
    config->trigger->type = SENSOR_TRIG_TIMER;
#else
    config->trigger->type = SENSOR_TRIG_DATA_READY;
#endif

    config->trigger->chan = SENSOR_CHAN_ALL;

    if (sensor_trigger_set(config->sensor, &config->trigger, sensor_callback_handler) < 0) {
        LOG_ERR("Failed to set sensor trigger for %s on %s", config->sensor->name, dev->name);
        return -EIO;
    }

    return 0;
}

static int behavior_imu_mouse_pressed(struct zmk_behavior_binding *binding) {
    const struct device *dev = binding->behavior->dev;
    LOG_DBG("Behavior %s pressed", dev->name);
    dev->config->status &= ~BEHAVIOR_IMU_MOUSE_INACTIVE;
    return 0;
}

static int behavior_imu_mouse_released(struct zmk_behavior_binding *binding) {
    const struct device *dev = binding->behavior->dev;
    LOG_DBG("Behavior %s released", dev->name);
    dev->config->status |= BEHAVIOR_IMU_MOUSE_INACTIVE;
    return 0;
}

const struct behavior_driver_api behavior_imu_mouse_driver_api = {
    .init = behavior_imu_mouse_init,
    .pressed = behavior_imu_mouse_pressed,
    .released = behavior_imu_mouse_released,
};

#define DT_INST(inst)                                                                              \
    static const struct behavior_imu_mouse_config behavior_imu_mouse_config_##inst = {             \
        .sensor = DEVICE_DT_GET(DT_INST_PROP(inst, sensor)),                                       \
        .sensor_axis = {(uint8_t)(DT_INST_PROP(inst, sensor_axis)[0] & 0xff),                      \
                        (uint8_t)(DT_INST_PROP(inst, sensor_axis)[1] & 0xff)},                     \
        .scale_multiplier = DT_INST_PROP_OR(inst, scale_multiplier, 1),                            \
        .scale_divisor = DT_INST_PROP_OR(inst, scale_divisor, 1),                                  \
        .status = (BEHAVIOR_IMU_MOUSE_INACTIVE |                                                   \
                   (DT_INST_PROP_OR(inst, perform_scrolling, 0) ? BEHAVIOR_IMU_MOUSE_MOVING        \
                                                                : BEHAVIOR_IMU_MOUSE_SCROLLING)),  \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, &behavior_imu_mouse_init, NULL, NULL,                              \
                          &behavior_imu_mouse_config_##inst, POST_KERNEL,                          \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(DT_DRV_COMPAT, DT_INST)
