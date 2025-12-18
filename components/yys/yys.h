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
    uint16_t co;
    uint16_t o2;
    uint16_t h2s;
    uint16_t ch4;
    uint16_t error_cnt;
    uint8_t data_cnt;
    uint8_t debug;
} yys_sensor_t;

esp_err_t yys_init(yys_sensor_t **sensor, uint8_t rx_pin, uint8_t tx_pin);

bool yys_data_ready(yys_sensor_t *sensor);

uint16_t yys_get_co_raw(yys_sensor_t *sensor);

uint16_t yys_get_o2_raw(yys_sensor_t *sensor);

uint16_t yys_get_h2s_raw(yys_sensor_t *sensor);

uint16_t yys_get_ch4_raw(yys_sensor_t *sensor);

float yys_get_co(yys_sensor_t *sensor);

float yys_get_o2(yys_sensor_t *sensor);

float yys_get_h2s(yys_sensor_t *sensor);

float yys_get_ch4(yys_sensor_t *sensor);

void yys_dump(yys_sensor_t *sensor);

#ifdef __cplusplus
};
#endif
