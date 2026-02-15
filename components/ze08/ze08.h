/**
 * ZE08 - ZE08-CH2O Driver for Esspressif ESP-32.
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

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct __attribute__((packed)) {
    uint16_t ch2o;               /*!< Latest raw CH2O value in PPB (PPM=PPB/1000, 1PPM*1.25=1.25mg/m³) */
} ze08_values_t;

typedef struct ze08_sensor_s {
    char *name;                  /*!< Optional name of this sensor */
    uint8_t *buffer;             /*!< Buffer for received sensor data */
    int baudrate;                /*!< baud rate, normally 9600 */
    uint8_t rx_pin;              /*!< Receive I/O pin */
    uint8_t cnt;                 /*!< Internal bit receive counter */
    ze08_values_t values;        /*!< Latest values */
    uint16_t error_cnt;          /*!< Receive error counter */
    uint8_t data_cnt;            /*!< Receive data counter */
    bool data_ready;             /*!< New data available flag */
    uint8_t debug;               /*!< Bitmask for debugging */
} ze08_sensor_t;

esp_err_t ze08_init(ze08_sensor_t **sensor, uint8_t rx_pin, uint8_t tx_pin);

bool ze08_data_ready(ze08_sensor_t *sensor);

uint16_t ze08_get_ch2o_raw(ze08_sensor_t *sensor);

float ze08_get_ch2o_ppm(ze08_sensor_t *sensor);

float ze08_get_ch2o_mg(ze08_sensor_t *sensor);

void ze08_dump_values(ze08_sensor_t *sensor, bool force);

#ifdef __cplusplus
};
#endif
