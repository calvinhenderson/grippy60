#define DT_DRV_COMPAT zmk_behavior_imu_mouse

#include <zmk-imu-mouse.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <math.h>

LOG_MODULE_REGISTER(DT_DRV_COMPAT, CONFIG_ZMK_LOG_LEVEL);

struct behavior_imu_mouse_config {
    const struct device *behavior;
    const struct device *sensor;
    uint8_t sensor_axis[2];
    int32_t sensitivity_multiplier;
    int32_t sensitivity_divisor;
};

struct behavior_imu_mouse_data {
    const struct device *behavior;
    struct sensor_trigger trigger;
    uint8_t status;
    double last_pitch, last_roll;
};

static void sensor_callback_handler(const struct device *sensor,
                                    const struct sensor_trigger *trigger) {
    struct behavior_imu_mouse_data *state =
        CONTAINER_OF(trigger, struct behavior_imu_mouse_data, trigger);
    const struct device *behavior = state->behavior;
    const struct behavior_imu_mouse_config *config = behavior->config;

    if (state->status & IMU_SCROLL || state->status & IMU_MOVE) {
        LOG_DBG("Processing sensor packet");
    } else {
        LOG_DBG("Skipping sensor packet..");
        return;
    }

    double accel[3];
    double gyro[3];

    int accel_channels[3] = {SENSOR_CHAN_ACCEL_X, SENSOR_CHAN_ACCEL_Y, SENSOR_CHAN_ACCEL_Z};
    int gyro_channels[3] = {SENSOR_CHAN_GYRO_X, SENSOR_CHAN_GYRO_Y, SENSOR_CHAN_GYRO_Z};

    if (0 != sensor_sample_fetch_chan(sensor, SENSOR_CHAN_ALL)) {
        LOG_ERR("Failed to fetch sensor channels.");
        return;
    } else {
        LOG_DBG("Fetched sensor channels");
    }

    for (int i = 0; i < 3; i++) {
        struct sensor_value raw;
        sensor_channel_get(sensor, accel_channels[i], &raw);
        accel[i] = sensor_value_to_double(&raw);
        sensor_channel_get(sensor, gyro_channels[i], &raw);
        gyro[i] = sensor_value_to_double(&raw);
    }

    LOG_DBG("Calculating movement deltas");
    float pitchFromAccel = 0;
    float rollFromAccel = 0;

    pitchFromAccel = atan(-accel[0] / sqrt(pow(accel[1], 2) + pow(accel[2], 2)));
    rollFromAccel = atan(accel[1] / sqrt(pow(accel[0], 2) + pow(accel[2], 2)));

    float _pitchGyroFavoring = 0.98;
    float _rollGyroFavoring = 0.98;
    float _filterUpdateRate = 1.00;

    // Complimentary Filter
    float pitch =
        (_pitchGyroFavoring) * (state->last_pitch + (gyro[1] * (1.00 / _filterUpdateRate))) +
        (1.00 - _pitchGyroFavoring) * (pitchFromAccel);

    float roll = (_rollGyroFavoring) * (state->last_roll + (gyro[0] * (1.00 / _filterUpdateRate))) +
                 (1.00 - _rollGyroFavoring) * (rollFromAccel);

    state->last_pitch = pitch;
    state->last_roll = roll;

    int16_t dx = pitch * config->sensitivity_multiplier / config->sensitivity_divisor;
    int16_t dy = roll * config->sensitivity_multiplier / config->sensitivity_divisor;

    LOG_DBG("pitch: %.2f, roll: %.2f", pitch, roll);
}

static int behavior_imu_mouse_init(const struct device *behavior) {
    const struct behavior_imu_mouse_config *config = behavior->config;
    struct behavior_imu_mouse_data *state = behavior->data;

    // Reset the instance state
    memset(state, 0, sizeof(struct behavior_imu_mouse_data));

    if (!device_is_ready(config->sensor)) {
        LOG_ERR("Sensor device %s is not ready", config->sensor->name);
        return -ENODEV;
    }

    state->trigger.type = SENSOR_TRIG_DATA_READY;
    state->trigger.chan = SENSOR_CHAN_ALL;

    if (sensor_trigger_set(config->sensor, &state->trigger, sensor_callback_handler) < 0) {
        LOG_ERR("Failed to set sensor trigger for %s on %s", config->sensor->name, behavior->name);
        return -EIO;
    } else {
        LOG_DBG("Set sensor trigger for %s on %s", config->sensor->name, behavior->name);
    }

    return 0;
}

static int on_behavior_imu_mouse_pressed(struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event) {
    const struct device *behavior = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_imu_mouse_data *state = behavior->data;
    const struct behavior_imu_mouse_config *config = behavior->config;

    switch (binding->param1) {
    case IMU_MOVE:
        LOG_DBG("Started moving for %s", behavior->name);
        break;
    case IMU_SCROLL:
        LOG_DBG("Started scrolling for %s", behavior->name);
        break;
    default:
        LOG_DBG("Ignoring unsupported action %d for device %s.", binding->param1, behavior->name);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    // Set the status bit
    state->status |= (binding->param1 & 0xf);

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_behavior_imu_mouse_released(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    const struct device *behavior = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_imu_mouse_data *state = behavior->data;
    const struct behavior_imu_mouse_config *config = behavior->config;

    switch (binding->param1) {
    case IMU_MOVE:
        LOG_DBG("Stopped moving for %s", behavior->name);
        break;
    case IMU_SCROLL:
        LOG_DBG("Stopped scrolling for %s", behavior->name);
        break;
    default:
        LOG_DBG("Ignoring unsupported action %d for device %s.", binding->param1, behavior->name);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    // Unset the status bit
    state->status &= ~(binding->param1 & 0xf);

    return ZMK_BEHAVIOR_OPAQUE;
}

const struct behavior_driver_api behavior_imu_mouse_driver_api = {
    .binding_pressed = on_behavior_imu_mouse_pressed,
    .binding_released = on_behavior_imu_mouse_released,
};

#define IMU_MOUSE_INST(inst)                                                                       \
    static const struct behavior_imu_mouse_config behavior_imu_mouse_config_##inst = {             \
        .behavior = DEVICE_DT_INST_GET(inst),                                                      \
        .sensor = DEVICE_DT_GET(DT_INST_PROP(inst, sensor)),                                       \
        .sensor_axis = {DT_INST_PROP_BY_IDX(inst, sensor_axis, 0) & 0xff,                          \
                        DT_INST_PROP_BY_IDX(inst, sensor_axis, 1) & 0xff},                         \
        .sensitivity_multiplier = DT_INST_PROP_OR(inst, sensitivity_multiplier, 1),                \
        .sensitivity_divisor = DT_INST_PROP_OR(inst, sensitivity_divisor, 1),                      \
    };                                                                                             \
                                                                                                   \
    static struct behavior_imu_mouse_data behavior_imu_mouse_data_##inst = {                       \
        .behavior = DEVICE_DT_INST_GET(inst),                                                      \
        .status = 0,                                                                               \
    };                                                                                             \
                                                                                                   \
    BEHAVIOR_DT_INST_DEFINE(inst, &behavior_imu_mouse_init, NULL, &behavior_imu_mouse_data_##inst, \
                            &behavior_imu_mouse_config_##inst, APPLICATION,                        \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_imu_mouse_driver_api);

DT_INST_FOREACH_STATUS_OKAY(IMU_MOUSE_INST)
