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
//#include "mqtt.h"
#include "tcp_server.h"

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
    E_SENSOR_INFO = 0,
    E_SENSOR_STATUS,
    E_SENSOR_UNITS,
    E_SENSOR_DATE = 0x0F,
    E_SENSOR_TIME,
    E_SENSOR_GPS,
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

typedef struct gps_values_s {
    const char *sat;
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
    uint8_t status;
    float pdop;         // Position dilution of precision (PDOP), typically 1 or 2 dp
    float hdop;         // Horizontal dilution of precision (HDOP), typically 1 or 2 dp
    float vdop;         // Vertical dilution of precision (VDOP), typically 1 or 2 dp
} gps_values_t;

typedef struct scd4x_cal_values_s {
    float temperature_offset;
    uint16_t altitude;
    uint16_t pressure;
} scd4x_cal_values_t;

typedef gps_values_t sensors_data_gps_t;

typedef bmx280_values_t sensors_data_bmx280_t;

typedef mhz19_values_t sensors_data_mhz19_t;

typedef scd4x_values_t sensors_data_scd4x_t;

typedef yys_values_t sensors_data_yys_t;

typedef sps30_values_t sensors_data_sps30_t;

typedef adxl345_values_t sensors_data_adxl345_t;

typedef qmc5883l_values_t sensors_data_qmc5883l_t;

typedef struct sensors_data_s {
    sensors_data_gps_t gps;
    sensors_data_bmx280_t bmx280lo;
    sensors_data_bmx280_t bmx280hi;
    sensors_data_mhz19_t mhz19;
    sensors_data_scd4x_t scd4x;
    sensors_data_yys_t yys;
    sensors_data_sps30_t sps30;
    sensors_data_adxl345_t adxl345;
    sensors_data_qmc5883l_t qmc5883l;
} sensors_data_t;

typedef struct status_s {
    bool force_update;
    bool recording;
    uint16_t record_pos;
    uint16_t file_cnt;
    char filename[32];
} status_t;

extern gps_sensor_t *gps;
extern gps_values_t gps_values;
extern gps_status_t *gps_status;
extern status_t status;
extern bmx280_t *bmx280lo;
extern bmx280_t *bmx280hi;
extern mhz19_t *mhz19;
extern scd4x_t *scd4x;
extern sps30_t *sps30;
extern yys_sensor_t *yys_sensor;
extern adxl345_t *adxl345;
extern qmc5883l_t *qmc5883l;

extern bool gps_update;
extern bool bmx280lo_update;
extern bool bmx280hi_update;
extern bool mhz19_update;
extern bool scd4x_calibrate;
extern bool scd4x_update;
extern bool yys_update;
extern bool sps30_update;
extern bool adxl345_update;
extern bool qmc5883l_update;

// Debug flags for main file
// Bit 0: Force sensor update flags to true
// Bit 1: Log new sensor values
// Bit 2: Log BMX280 sensor values
// Bit 3: Log MHZ19 and SCD4X sensor values
// Bit 4: Log YYS sensor values
// Bit 5: Log SPS30 sensor values
// Bit 6: Log ADXL345 ad QMC5883L sensor values
extern uint32_t debug_main;

void get_current_date_time(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *min, uint8_t *sec);

void set_data_filename();

#ifdef __cplusplus
};
#endif
