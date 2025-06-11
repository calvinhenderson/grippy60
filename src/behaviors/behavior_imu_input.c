#include "behavior_imu_input.h"

#define DT_DRV_COMPAT zmk_behavior_imu_input

#define DEGREE_PER_RADIAN 57.29577
#define DATA_RATE CONFIG_ZMK_BEHAVIOR_IMU_INPUT_DATA_RATE
#define CALIBRATION_PERIOD CONFIG_ZMK_BEHAVIOR_IMU_INPUT_CALIBRATION_PERIOD

#define EWMA(alpha, new, prev) ((alpha * new) + ((1.0 - alpha) * prev))

LOG_MODULE_REGISTER(DT_DRV_COMPAT, CONFIG_ZMK_LOG_LEVEL);

static inline void value_for_axis(const struct device *sensor, uint8_t axis, float *accel,
                                  float *gyro) {
    int8_t invert = (axis & IMU_INVERT) ? -1 : 1;

    struct sensor_value val;

    switch (axis & (IMU_AXIS_X | IMU_AXIS_Y | IMU_AXIS_Z)) {
    case IMU_AXIS_X:
        sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_X, &val);
        *accel = sensor_value_to_float(&val) * invert;
        sensor_channel_get(sensor, SENSOR_CHAN_GYRO_X, &val);
        *gyro = sensor_value_to_float(&val) * invert * DEGREE_PER_RADIAN;
        return;
    case IMU_AXIS_Y:
        sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_Y, &val);
        *accel = sensor_value_to_float(&val) * invert;
        sensor_channel_get(sensor, SENSOR_CHAN_GYRO_Y, &val);
        *gyro = sensor_value_to_float(&val) * invert * DEGREE_PER_RADIAN;
        return;
    case IMU_AXIS_Z:
        sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_Z, &val);
        *accel = sensor_value_to_float(&val) * invert;
        sensor_channel_get(sensor, SENSOR_CHAN_GYRO_Z, &val);
        *gyro = sensor_value_to_float(&val) * invert * DEGREE_PER_RADIAN;
        return;
    }
}

static int16_t calculate_axis_delta(struct imu_input_data *state, const struct device *sensor,
                                    struct imu_input_data_axis *data, uint8_t axis,
                                    uint8_t threshold) {
    // get current sensor values
    value_for_axis(sensor, axis, &state->accel, &state->gyro);

    data->prev = data->avg;
    data->avg = EWMA(0.70, EWMA(0.95, state->gyro, state->accel), data->prev);
    data->first += data->avg * (state->readings <= CALIBRATION_PERIOD);

    // return movement delta
    data->delta = data->avg - (data->first / state->readings);

    return data->delta * (data->delta > threshold || data->delta < -threshold);
}

static void behavior_imu_input_process_data(struct imu_input_data *state,
                                            const struct device *sensor) {
    const struct imu_input_config *config = state->self->config;

    if (!device_is_ready(config->sensor)) {
        LOG_WRN("sensor %s is not ready for reading in %s", config->sensor->name,
                state->self->name);
        return;
    }

    int rc = sensor_sample_fetch_chan(sensor, SENSOR_CHAN_ALL);
    if (rc != 0) {
        LOG_ERR("(%d) failed to fetch imu sensor sample for %s", rc, state->self->name);
        return;
    }

    // update the number of readings
    state->readings += 1 * (state->readings <= CALIBRATION_PERIOD);

    int16_t delta =
        calculate_axis_delta(state, config->sensor, &state->x, config->axis_x, config->threshold_x);
    input_report_rel(state->self, INPUT_REL_X, delta, false, K_FOREVER);

    delta =
        calculate_axis_delta(state, config->sensor, &state->y, config->axis_y, config->threshold_x);
    input_report_rel(state->self, INPUT_REL_Y, delta, true, K_FOREVER);
}

static void behavior_imu_input_work_handler(struct k_work *work) {
    struct imu_input_data *state = CONTAINER_OF(work, struct imu_input_data, work);
    const struct imu_input_config *config = state->self->config;
    behavior_imu_input_process_data(state, config->sensor);
}

static void behavior_imu_input_timer_handler(struct k_timer *timer) {
    struct imu_input_data *state = CONTAINER_OF(timer, struct imu_input_data, timer);
    k_work_submit(&state->work);
}

static int behavior_imu_input_init(const struct device *self) {
    const struct imu_input_config *config = self->config;
    struct imu_input_data *state = self->data;

    if (!device_is_ready(config->sensor)) {
        LOG_ERR("sensor %s is not ready.", config->sensor->name);
    }

    memset(state, 0, sizeof(struct imu_input_data));
    state->self = self;

    k_work_init(&state->work, behavior_imu_input_work_handler);
    k_timer_init(&state->timer, behavior_imu_input_timer_handler, NULL);

    LOG_DBG("imu input device %s for sensor %s initialized successfully", self->name,
            config->sensor->name);

    return 0;
}

static int on_behavior_imu_input_pressed(struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event) {
    const struct device *self = zmk_behavior_get_binding(binding->behavior_dev);
    struct imu_input_data *state = self->data;

    state->readings = 0;
    memset(&state->x, 0, sizeof(struct imu_input_data_axis));
    memset(&state->y, 0, sizeof(struct imu_input_data_axis));
    k_timer_start(&state->timer, K_MSEC(0), K_MSEC(DATA_RATE));

    LOG_DBG("work queue started for %s", state->self->name);

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_behavior_imu_input_released(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    const struct device *self = zmk_behavior_get_binding(binding->behavior_dev);
    struct imu_input_data *state = self->data;

    k_timer_stop(&state->timer);
    LOG_DBG("work queue stopped for %s", state->self->name);

    return ZMK_BEHAVIOR_OPAQUE;
}

const struct behavior_driver_api behavior_imu_input_driver_api = {
    .binding_pressed = on_behavior_imu_input_pressed,
    .binding_released = on_behavior_imu_input_released,
    .locality = BEHAVIOR_LOCALITY_EVENT_SOURCE,
};

#define IMU_INPUT_INST(n)                                                                          \
    static const struct imu_input_config behavior_imu_input_config_##n = {                         \
        .self = DEVICE_DT_INST_GET(n),                                                             \
        .sensor = DEVICE_DT_GET(DT_INST_PROP(n, sensor)),                                          \
        .axis_x = (uint8_t)(DT_INST_PROP(n, axis_x) & 0xf),                                        \
        .axis_y = (uint8_t)(DT_INST_PROP(n, axis_y) & 0xf),                                        \
        .threshold_x = (uint8_t)(DT_INST_PROP(n, threshold_x) & 0xf),                              \
        .threshold_y = (uint8_t)(DT_INST_PROP(n, threshold_y) & 0xf),                              \
    };                                                                                             \
                                                                                                   \
    static struct imu_input_data behavior_imu_input_data_##n = {};                                 \
                                                                                                   \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_imu_input_init, NULL, &behavior_imu_input_data_##n,        \
                            &behavior_imu_input_config_##n, POST_KERNEL,                           \
                            CONFIG_ZMK_BEHAVIOR_IMU_INPUT_INIT_PRIORITY,                           \
                            &behavior_imu_input_driver_api);

DT_INST_FOREACH_STATUS_OKAY(IMU_INPUT_INST)
