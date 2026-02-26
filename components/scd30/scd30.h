/**
 * SCD30
 *
 * MIT License
 *
 * Copyright (C) 2026 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <esp_err.h>
#include "driver/i2c_master.h"

typedef struct __attribute__((packed)) {
    float co2;
    float temperature;
    float humidity;
} scd30_values_t;

typedef struct scd30_s {
    #if CONFIG_USE_I2C_MASTER_DRIVER
    // I2C master handle via port with configuration
    i2c_master_dev_handle_t dev_handle;
    // I2C master configuration
    i2c_device_config_t dev_config;
    // I2C master handle via port
    i2c_master_bus_handle_t bus_handle;
    #endif

    scd30_values_t values;
    uint16_t firmware_version;
    esp_err_t last_error;
    uint8_t debug;
} scd30_t;

scd30_t *scd30_create_master(i2c_master_bus_handle_t bus_handle);

esp_err_t scd30_device_create(scd30_t *sensor);

esp_err_t scd30_init(scd30_t **sensor_ptr, i2c_master_bus_handle_t bus_handle);

esp_err_t scd30_soft_reset(scd30_t *sensor);

void scd30_close(scd30_t *sensor);

esp_err_t scd30_start_continuous_measurement(scd30_t *sensor, uint16_t p_comp);

esp_err_t scd30_stop_continuous_measurement(scd30_t *sensor);

uint16_t scd30_get_measurement_interval(scd30_t *sensor);

esp_err_t scd30_set_measurement_interval(scd30_t *sensor, uint16_t interval_seconds);

bool scd30_get_data_ready_status(scd30_t *sensor);

esp_err_t scd30_read_measurement(scd30_t *sensor);

bool scd30_get_automatic_self_calibration(scd30_t *sensor);

esp_err_t scd30_set_automatic_self_calibration(scd30_t *sensor, bool enabled);

uint16_t scd30_get_forced_recalibration_value(scd30_t *sensor);

esp_err_t scd30_set_forced_recalibration_value(scd30_t *sensor, uint16_t target_co2);

uint16_t scd30_get_temperature_offset_int(scd30_t *senso);

float scd30_get_temperature_offset(scd30_t *sensor);

esp_err_t scd30_set_temperature_offset_int(scd30_t *sensor, uint16_t t_offset);

esp_err_t scd30_set_temperature_offset(scd30_t *sensor, float t_offset);

uint16_t scd30_get_sensor_altitude(scd30_t *sensor);

esp_err_t scd30_set_sensor_altitude(scd30_t *sensor, uint16_t altitude);

uint16_t scd30_get_firmware_version(scd30_t *sensor);

void scd30_dump_values(scd30_t *sensor, bool force);

#ifdef __cplusplus
};
#endif
