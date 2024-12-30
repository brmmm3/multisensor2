/**
 * GPS - GPS Driver for Esspressif ESP-32.
 *
 * MIT License
 *
 * Copyright (C) 2024 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#ifndef _SW_SERIAL_H_
#define _SW_SERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "hw_serial.h"
#include "sw_serial.h"

typedef struct yys_sensor_s {
    char *name;
    uint8_t *buffer;
    uint8_t cnt;
    QueueHandle_t queue;
    hw_serial_t *hw_serial;
    sw_serial_t *sw_serial;
} yys_sensor_t;

void yys_init();

#ifdef __cplusplus
};
#endif

#endif
