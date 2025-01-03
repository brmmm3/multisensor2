/**
 * HW Serial driver
 *
 * MIT License
 *
 * Copyright (C) 2024 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#ifndef _HW_SERIAL_H_
#define _HW_SERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct hw_serial_s {
    uint32_t baudrate;
    uint8_t uart_num;
    uint8_t rx_pin;
    uint8_t tx_pin;
    QueueHandle_t queue;
} hw_serial_t;

void uart_init(uint8_t uart_num, int rx_pin, int tx_pin);

#ifdef __cplusplus
};
#endif

#endif
