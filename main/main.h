/**
 * RTC
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
#include <stdio.h>
#include <string.h>
#include <argtable3/argtable3.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hal/wdt_hal.h"
#include "esp_log.h"
#include "esp_console.h"

#include "led_strip.h"

#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "hal/uart_types.h"
#include "driver/i2c_master.h"

#include "config.h"

#include "wifi/include/wifi.h"
#include "rtc_tiny.h"
#include "ftp.h"
#include "gps.h"
#include "lcd.h"
#include "ui/include/ui.h"
#include "sdcard.h"
#include "bmx280.h"
#include "scd4x.h"
#include "mhz19.h"
#include "sps30.h"
#include "yys.h"
#include "adxl345.h"
#include "qmc5883l.h"


extern rtc_t *rtc;
extern gps_sensor_t *gps;
extern gps_status_t *gps_status;
extern bmx280_t *bmx280lo;
extern bmx280_t *bmx280hi;
extern mhz19_t *mhz19;
extern scd4x_t *scd4x;
extern yys_sensor_t *yys_sensor;
extern sps30_t *sps30;
extern adxl345_t *adxl345;
extern qmc5883l_t *qmc5883l;
extern wdt_hal_context_t rtc_wdt_ctx;
extern i2c_master_bus_handle_t bus_handle;
extern int spi_host_id;
extern lv_display_t *lcd;
extern ui_t *ui;
extern led_strip_handle_t led_strip;

typedef enum {
    E_SENSOR_GPS = 0,
    E_SENSOR_BMX280_LO,
    E_SENSOR_BMX280_HI,
    E_SENSOR_MHZ19,
    E_SENSOR_SCD4XCAL,
    E_SENSOR_SCD4X,
    E_SENSOR_YYS,
    E_SENSOR_SPS30,
    E_SENSOR_ADXL345,
    E_SENSOR_QMC5883L
} sensors_enum_t;

typedef struct status_s {
    bool recording;
    uint16_t record_pos;
    uint64_t start_time;
} status_t;

extern status_t status;

extern bool gps_update;
extern bool bmx280lo_update;
extern bool bmx280hi_update;
extern bool mhz19_update;
extern bool scd4x_update;
extern bool yys_update;
extern bool sps30_update;
extern bool adxl345_update;
extern bool qmc5883l_update;

#ifdef __cplusplus
};
#endif
