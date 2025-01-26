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

#include "wifi.h"
#include "rtc_tiny.h"
#include "ftp.h"
#include "gps.h"
#include "lcd.h"
#include "ui.h"
#include "sdcard.h"
#include "bmx280.h"
#include "scd4x.h"
#include "mhz19.h"
#include "sps30.h"
#include "yys.h"
#include "adxl345.h"
#include "qmc5883l.h"

#include "wifi_sntp.h"


rtc_t *rtc = NULL;
gps_sensor_t *gps = NULL;
gps_status_t *gps_status = NULL;
bmx280_t *bmx280lo = NULL;
bmx280_t *bmx280hi = NULL;
adxl345_t *adxl345 = NULL;
mhz19_t *mhz19 = NULL;
scd4x_t *scd4x = NULL;
sps30_t *sps30 = NULL;
qmc5883l_t *qmc5883l = NULL;
yys_sensors_t *yys_sensors = NULL;
wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
i2c_master_bus_handle_t bus_handle;
int spi_host_id;
lv_display_t *lcd;
ui_t *ui;
led_strip_handle_t led_strip;

#ifdef __cplusplus
};
#endif
