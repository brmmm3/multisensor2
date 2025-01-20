/**
 * HMC5883L/QMC5883L
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

#define XMC5883L_MODE_STANDBY    0b00000000
#define XMC5883L_MODE_CONTINUOUS 0b00000001

#define XMC5883L_ODR_10Hz        0b00000000
#define XMC5883L_ODR_50Hz        0b00000100
#define XMC5883L_ODR_100Hz       0b00001000
#define XMC5883L_ODR_200Hz       0b00001100

#define XMC5883L_RNG_2G          0b00000000
#define XMC5883L_RNG_8G          0b00010000

#define XMC5883L_OSR_512         0b00000000
#define XMC5883L_OSR_256         0b01000000
#define XMC5883L_OSR_128         0b10000000
#define XMC5883L_OSR_64          0b11000000


typedef struct xmc5883l_s {
    // I2C master handle via port with configuration
    i2c_master_dev_handle_t i2c_dev;
    // I2C master configuration
    i2c_device_config_t dev_cfg;
    // I2C master handle via port
    i2c_master_bus_handle_t bus_handle;
    float accel_x;  // [g]
    float accel_y;
    float accel_z;
    float temp;     // [°C]
    uint8_t status;
    uint8_t accel_range;
    uint32_t device_id;
} xmc5883l_t;

esp_err_t xmc5883l_reset(xmc5883l_t *sensor);

esp_err_t xmc5883l_get_device_id(xmc5883l_t *sensor);

esp_err_t xmc5883l_get_status(xmc5883l_t *sensor);

bool xmc5883l_data_ready(xmc5883l_t *sensor);

bool xmc5883l_data_overflow(xmc5883l_t *sensor);

esp_err_t xmc5883l_set_mode(xmc5883l_t *sensor, uint8_t mode, uint8_t odr, uint8_t rng, uint8_t osr);

esp_err_t xmc5883l_set_accel_range(xmc5883l_t *sensor, uint8_t range);

esp_err_t xmc5883l_read_data(xmc5883l_t *sensor);

esp_err_t xmc5883l_init(xmc5883l_t **sensor, i2c_master_bus_handle_t bus_handle);

#ifdef __cplusplus
};
#endif
