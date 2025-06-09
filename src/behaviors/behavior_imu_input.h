#pragma once

#include <zmk-imu-input.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/sensor.h>

struct imu_sensor_axis {
    float x, y, z;
};

struct imu_input_config {
    const struct device *self;
    const struct device *sensor;
    uint8_t axis_x, axis_y;
    uint8_t threshold_x, threshold_y;
};

struct imu_input_data {
    const struct device *self;
    struct k_work work;
    struct k_timer timer;

    float accel, gyro;
    uint8_t readings;

    struct imu_input_data_axis {
        int16_t avg, prev, delta;
        int32_t first;
    } x, y;
};
