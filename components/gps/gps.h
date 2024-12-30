/**
 * GPS - GPS Driver for Esspressif ESP-32.
 *
 * MIT License
 *
 * Copyright (C) 2024 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#ifndef _GPS_H_
#define _GPS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "hw_serial.h"

// GPS NMEA
#define NMEA_END_CHAR_1 '\n'
#define NMEA_MAX_LENGTH 70

typedef struct gps_rmc_s {
    uint32_t date;
    float utc_pos;
    char status;
    float lat;
    char ns;
    float lng;
    char ew;
    float speed;
    float course;
    float mv;   // Magnetic Variation
    char mv_ew;
} gps_rmc_t;

typedef struct gps_gll_s {
    float utc_pos;
    char status;
    float lat;
    char ns;
    float lng;
    char ew;
} gps_gll_t;

typedef struct gps_gsa_s {
    char mode_auto;
    char mode_3d;
    uint8_t sats[12];
    float pdop;
    float hdop;
    float vdop;
} gps_gsa_t;

typedef struct gps_gsv_sat_s {
    uint8_t svid;
    uint8_t elv;    // Elevation (0..90)
    uint16_t az;    // Azimuth (0..359)
    uint8_t snr;    // SNR (0..99, 255=NOT TRACKING)
} gps_gsv_sat_t;

typedef struct gps_gsv_s {
    uint8_t num_sv;
    gps_gsv_sat_t sats[9];
} gps_gsv_t;

typedef struct gps_gga_s {
    float utc_pos;
    float lat;
    char ns;
    float lng;
    char ew;
    uint8_t pos_fix;
    uint8_t sat_used;
    float hdop;
    float altitude;
    char alt_unit;
    float goid;
    char goid_unit;
    float age;
    uint16_t diff_stat_id;
} gps_gga_t;

typedef struct gps_vtg_s {
    float true_track;
    char true_unit;
    float magnetic_track;
    char magnetic_unit;
    float speed_knots;
    char knot_unit;
    float speed_kmh;
    char kmh_unit;
} gps_vtg_t;

typedef struct gps_zda_s {
    float utc_pos;
    uint8_t year;
    uint8_t month;
    uint8_t day;
    int8_t zone_hours;
    int8_t zone_minutes;
} gps_zda_t;

typedef struct gps_sensor_s {
    char *name;
    uint8_t *buffer;
    uint8_t cnt;
    QueueHandle_t queue;
    hw_serial_t *serial;
    char *messages;
    uint8_t msg_size;
    gps_rmc_t rmc;
    gps_gll_t gll;
    gps_gsa_t gsa;
    gps_gsv_t gsv;
    gps_gga_t gga;
    gps_vtg_t vtg;
    gps_zda_t zda;
} gps_sensor_t;

gps_sensor_t gps_init();

#ifdef __cplusplus
};
#endif

#endif
