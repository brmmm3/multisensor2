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
#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hal/wdt_hal.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_console.h"

#include "led_strip.h"

#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "rom/gpio.h"
#include "hal/uart_types.h"
#include "driver/i2c_master.h"

#include "config.h"

#include "wifi.h"
#include "rtc_tiny.h"
#include "ftp.h"
#include "gps.h"
#include "lcd.h"
#include "ui/ui.h"
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
extern adxl345_t *adxl345;
extern mhz19_t *mhz19;
extern scd4x_t *scd4x;
extern sps30_t *sps30;
extern qmc5883l_t *qmc5883l;
extern yys_sensor_t *yys_sensor;
extern wdt_hal_context_t rtc_wdt_ctx;
extern i2c_master_bus_handle_t bus_handle;
extern int spi_host_id;
extern lv_display_t *lcd;
extern ui_t *ui;
extern led_strip_handle_t led_strip;

extern bool gps_update;
extern bool sps30_update;
extern bool bmx280lo_update;
extern bool bmx280hi_update;
extern bool scd4x_update;
extern bool mhz19_update;
extern bool yys_update;
extern bool qmc5883l_update;
extern bool adxl345_update;

#ifdef __cplusplus
};
#endif
