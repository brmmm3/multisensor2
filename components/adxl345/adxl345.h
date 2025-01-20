/**
 * RTC
 *
 * MIT License
 *
 * Copyright (C) 2024 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#pragma once

#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "driver/i2c_master.h"

typedef struct adxl345_s {
    // I2C master handle via port with configuration
    i2c_master_dev_handle_t i2c_dev;
    // I2C master configuration
    i2c_device_config_t dev_cfg;
    // I2C master handle via port
    i2c_master_bus_handle_t bus_handle;
    float accel_x;  // [g]
    float accel_y;
    float accel_z;
    float accel_abs;
    float accel_offset_x;  // [g]
    float accel_offset_y;
    float accel_offset_z;
    uint8_t moving_cnt;
    uint8_t accel_range;
    uint8_t device_id;
} adxl345_t;

esp_err_t adxl345_get_device_id(adxl345_t *sensor);

esp_err_t adxl345_set_accel_range(adxl345_t *sensor, uint8_t range);

esp_err_t adxl345_read_data(adxl345_t *sensor);

esp_err_t adxl345_calibrate_offset(adxl345_t *sensor);

esp_err_t adxl345_init(adxl345_t **sensor, i2c_master_bus_handle_t bus_handle);

#ifdef __cplusplus
};
#endif
