/* MIT License
*
* Copyright (c) 2026 Martin Bammer
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
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
#include <esp_log.h>
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
#include "s11.h"
#include "scd30.h"
#include "scd4x.h"
#include "mhz19.h"
#include "ze08.h"
#include "sps30.h"
#include "yys.h"
#include "adxl345.h"
#include "qmc5883l.h"
//#include "mqtt.h"
#include "tcp_server.h"

typedef enum {
    E_SENSOR_INFO = 0,
    E_SENSOR_CONFIG,
    E_SENSOR_STATUS,
    E_SENSOR_UNITS,
    E_SENSOR_FREE,
    E_SENSOR_DATE = 0x10,
    E_SENSOR_TIME,
    E_SENSOR_DATETIME,
    E_SENSOR_GPS = 0x20,
    E_SENSOR_BMX280_LO = 0x30,
    E_SENSOR_BMX280_HI,
    E_SENSOR_SCD41CAL = 0x40,
    E_SENSOR_S11 = 0x50,
    E_SENSOR_SCD30,
    E_SENSOR_SCD41,
    E_SENSOR_MHZ19,
    E_SENSOR_YYS = 0x60,
    E_SENSOR_ZE08,
    E_SENSOR_SPS30 = 0x70,
    E_SENSOR_ADXL345 = 0x80,
    E_SENSOR_QMC5883L
} sensors_enum_t;

typedef struct  __attribute__((__packed__)) {
    uint8_t sat;        // gps_sat_enum_t
    uint32_t date;
    uint32_t time;
    float lat;
    uint8_t ns;
    float lng;
    uint8_t ew;
    float altitude;
    float speed;
    uint8_t mode_3d;
    uint8_t sats;
    float pdop;         // Position dilution of precision (PDOP), typically 1 or 2 dp
    float hdop;         // Horizontal dilution of precision (HDOP), typically 1 or 2 dp
    float vdop;         // Vertical dilution of precision (VDOP), typically 1 or 2 dp
    uint8_t status;
    uint8_t data_cnt;
    uint8_t error_cnt;
} gps_values_t;

typedef struct  __attribute__((__packed__)) {
    float temperature_offset;
    uint16_t altitude;
    uint16_t pressure;
} scd41_cal_values_t;

typedef gps_values_t sensors_data_gps_t;

typedef bmx280_values_t sensors_data_bmx280_t;

typedef s11_values_t sensors_data_s11_t;

typedef scd30_values_t sensors_data_scd30_t;

typedef scd4x_values_t sensors_data_scd41_t;

typedef mhz19_values_t sensors_data_mhz19_t;

typedef yys_values_t sensors_data_yys_t;

typedef ze08_values_t sensors_data_ze08_t;

typedef sps30_values_t sensors_data_sps30_t;

typedef adxl345_values_t sensors_data_adxl345_t;

typedef qmc5883l_values_t sensors_data_qmc5883l_t;

typedef struct  __attribute__((__packed__)) {
    sensors_data_gps_t gps;
    sensors_data_bmx280_t bmx280lo;
    sensors_data_bmx280_t bmx280hi;
    sensors_data_s11_t s11;
    sensors_data_scd30_t scd30;
    sensors_data_scd41_t scd41;
    sensors_data_mhz19_t mhz19;
    sensors_data_yys_t yys;
    sensors_data_ze08_t ze08;
    sensors_data_sps30_t sps30;
    sensors_data_adxl345_t adxl345;
    sensors_data_qmc5883l_t qmc5883l;
} sensors_data_t;

typedef struct {
    time_t start_time;
    time_t save_time;
    uint32_t startup_cnt;
    uint32_t uptime_cnt;
    bool force_update;
    bool recording;
    bool auto_status;
    bool scd4x_auto_values;
    uint16_t record_pos;
    uint16_t file_cnt;
    char filename[32];
    uint8_t lcd_pwr;
    int8_t rssi;
} status_t;

extern rtc_t *rtc;
extern gps_sensor_t *gps;
extern gps_values_t gps_values;
extern gps_status_t *gps_status;
extern bmx280_t *bmx280lo;
extern bmx280_t *bmx280hi;
extern s11_t *s11_sensor;
extern scd30_t *scd30_sensor;
extern scd4x_t *scd41_sensor;
extern mhz19_t *mhz19_sensor;
extern yys_sensor_t *yys_sensor;
extern ze08_t *ze08_sensor;
extern sps30_t *sps30;
extern adxl345_t *adxl345;
extern qmc5883l_t *qmc5883l;

extern status_t status;
extern wdt_hal_context_t rtc_wdt_ctx;
extern i2c_master_bus_handle_t bus_handle;
extern int spi_host_id;
extern lv_display_t *lcd;
extern ui_t *ui;
extern led_strip_handle_t led_strip;

extern bool gps_update;
extern bool bmx280lo_update;
extern bool bmx280hi_update;
extern bool mhz19_update;
extern bool s11_update;
extern bool scd30_update;
extern bool scd41_calibrate;
extern bool scd41_update;
extern bool yys_update;
extern bool ze08_update;
extern bool sps30_update;
extern bool adxl345_update;
extern bool qmc5883l_update;
extern bool any_sensor_update;

// Debug flags for main file
// Bit 0: Force sensor update flags to true
// Bit 1: Log new sensor values
// Bit 2: Log GPS sensor values
// Bit 3: Log BME280 sensor values
// Bit 4: Log SCD41 sensor calibration values
// Bit 5: Log S11 sensor values
// Bit 6: Log SCD30 sensor values
// Bit 7: Log SCD41 sensor values
// Bit 8: Log MHZ19 sensor values
// Bit 9: Log YYS sensor values
// Bit 10: Log ZE08 sensor values
// Bit 11: Log SPS30 sensor values
// Bit 12: Log ADXL345 sensor values
// Bit 13: Log QMC5883L sensor values
extern uint32_t debug_main;

extern bool force_update_all;

esp_err_t set_sys_time(struct tm *timeinfo, bool set_rtc_time);

void get_current_date_time(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *min, uint8_t *sec);

void set_data_filename();

void show_sd_card_info(int file_cnt);

#ifdef __cplusplus
};
#endif
