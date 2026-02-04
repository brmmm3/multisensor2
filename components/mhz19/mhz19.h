/**
 * MHZ19 Driver for Esspressif ESP-32.
 *
 * MIT License
 *
 * Copyright (C) 2025 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#ifndef _MHZ19_H_
#define _MHZ19_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hw_serial.h"

enum MHZ19_RANGE {
	MHZ19_RANGE_1000,
	MHZ19_RANGE_2000,
	MHZ19_RANGE_3000,
	MHZ19_RANGE_5000,
	MHZ19_RANGE_10000,
    MHZ19_RANGE_INVALID
};

typedef struct __attribute__((packed)) {
    uint16_t co2;
    uint8_t temp;
    uint8_t status;
} mhz19_values_t;

typedef struct mhz19_s {
    char *name;
    hw_serial_t *hw_serial;
    QueueHandle_t queue;
    uint8_t pending;

    bool auto_calib;
    mhz19_values_t values;
    uint8_t data_cnt;
    enum MHZ19_RANGE range;
    char fw_version[7];
    uint8_t debug;
    uint16_t error_cnt;
} mhz19_t;

esp_err_t mhz19_init(mhz19_t **sensor, uint8_t uart_num, uint8_t rx_pin, uint8_t tx_pin);

uint8_t mhz19_pending(mhz19_t *sensor);

bool mhz19_data_ready(mhz19_t *sensor);

esp_err_t mhz19_set_auto_calibration(mhz19_t *sensor, bool autocalib);

esp_err_t mhz19_get_auto_calibration(mhz19_t *sensor);

esp_err_t mhz19_calibrate_zero(mhz19_t *sensor);

esp_err_t mhz19_calibrate_span(mhz19_t *sensor, uint16_t ppm);

enum MHZ19_RANGE mhz19_get_range(mhz19_t *sensor);

esp_err_t mhz19_set_range(mhz19_t *sensor, enum MHZ19_RANGE range);

esp_err_t mhz19_reset(mhz19_t *sensor);

void mhz19_dump_values(mhz19_t *sensor, bool force);

#ifdef __cplusplus
};
#endif

#endif
