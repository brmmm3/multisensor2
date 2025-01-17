/**
 * GPS - GPS Driver for Esspressif ESP-32.
 *
 * MIT License
 *
 * Copyright (C) 2024 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "sw_serial.h"

typedef struct yys_sensor_s {
    char *name;
    uint8_t *buffer;
    uint8_t cnt;
    sw_serial_t *sw_serial;
    uint16_t value;
} yys_sensor_t;

typedef struct yys_sensors_s {
    yys_sensor_t *o2_sensor;
    yys_sensor_t *co_sensor;
    yys_sensor_t *h2s_sensor;
} yys_sensors_t;

esp_err_t yys_init(yys_sensors_t **sensor, uint8_t o2_pin_num, uint8_t co_pin_num, uint8_t h2s_pin_num);

uint16_t yys_get_co_raw(yys_sensors_t *sensor);

uint16_t yys_get_o2_raw(yys_sensors_t *sensor);

uint16_t yys_get_h2s_raw(yys_sensors_t *sensor);

float yys_get_co(yys_sensors_t *sensor);

float yys_get_o2(yys_sensors_t *sensor);

float yys_get_h2s(yys_sensors_t *sensor);

#ifdef __cplusplus
};
#endif
