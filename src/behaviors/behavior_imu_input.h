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
};

struct imu_input_data {
    const struct device *self;
    struct k_work work;
    struct k_timer timer;
    struct imu_input_data_average {
        int8_t x, y;
        uint8_t readings;
        int8_t first_x, first_y;
        int8_t prev_x, prev_y;
    } avg;
};
