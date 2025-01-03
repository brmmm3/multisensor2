/**
 * GPS - GPS Driver for Esspressif ESP-32.
 *
 * MIT License
 *
 * Copyright (C) 2024 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#ifndef _YYS_H_
#define _YYS_H_

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
    QueueHandle_t queue;
    sw_serial_t *sw_serial;
} yys_sensor_t;

typedef struct yys_sensors_s {
    yys_sensor_t *o2_sensor;
    yys_sensor_t *co_sensor;
    yys_sensor_t *h2s_sensor;
} yys_sensors_t;

yys_sensors_t yys_init();

#ifdef __cplusplus
};
#endif

#endif
