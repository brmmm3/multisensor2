/**
 * SW Serial driver
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

#undef IRAM_ATTR
#define IRAM_ATTR __attribute__((section(".iram")))

typedef struct sw_serial_s {
    uint32_t baudrate;
    uint32_t time;      // Time of last level change (edge)
    uint8_t tmp_val;    // Temporary value
    uint8_t rx_pin;
    uint8_t bit_cnt;    // Number of bits received
    QueueHandle_t queue;
    uint16_t cnt;
} sw_serial_t;

void IRAM_ATTR sw_serial_irq_handler(void *args);

#ifdef __cplusplus
};
#endif
