/**
 * GPS - GPS Driver for Esspressif ESP-32.
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

#include "hw_serial.h"

// GPS NMEA
#define NMEA_END_CHAR_1 '\n'
#define NMEA_MAX_LENGTH 70

// RMC - Recommended Minimum Navigation Information
typedef struct gps_rmc_s {
    uint16_t cnt;
    uint32_t date;      // Date, ddmmyy
    float time;         // UTC time, typically 2 or 3 dp. Leading zeros are always included
    char status;        // Data status; A = active (data valid), V = void (receiver warning)
    float lat;          // Latitude, typically 4 or 5 dp. Leading zeros are always included
    char ns;            // Hemispherical orientation N or S (north or south)
    float lng;          // Longitude, typically 4 or 5 dp. Leading zeros are always included
    char ew;            // Hemispherical orientation E or W (east or west)
    float speed;        // Speed over ground in knots, typically 2 or 3 dp
    float course;       // Course over ground in degrees true, typically 2 dp
    float mv;           // Magnetic Variation in degrees. Null (empty) if unavailable.
    char mv_ew;         // Magnetic Variation, E or W (East or West). Null (empty) if unavailable.
} gps_rmc_t;

// GLL - Geographic Latitude + Longitude
typedef struct gps_gll_s {
    uint16_t cnt;
    float time;         // UTC time, typically 2 or 3 dp. Leading zeros are always included
    char status;        // Data status; A = active (data valid), V = void (data invalid)
    float lat;          // Latitude, typically 4 or 5 dp. Leading zeros are always included
    char ns;            // Hemispherical orientation N or S (north or south)
    float lng;          // Longitude, typically 4 or 5 dp. Leading zeros are always included
    char ew;            // Hemispherical orientation E or W (east or west)
} gps_gll_t;

// GSA - GNSS Satellites Active
typedef struct gps_gsa_s {
    uint16_t cnt;
    char mode_auto;     // 2D / 3D operation mode; M = manual, A = automatic
    char mode_3d;       // Fix mode; 1 = no fix, 2 = 2D, 3 = 3D
    uint8_t sats[12];   // Satellite IDs or PRN numbers (leading zeros sent)
    float pdop;         // Position dilution of precision (PDOP), typically 1 or 2 dp
    float hdop;         // Horizontal dilution of precision (HDOP), typically 1 or 2 dp
    float vdop;         // Vertical dilution of precision (VDOP), typically 1 or 2 dp
} gps_gsa_t;

typedef struct gps_gsv_sat_s {
    uint8_t svid;       // Satellite ID or PRN number (leading zeros sent)
    uint8_t elv;        // Elevation in degrees (leading zeros sent); 00 to 90
    uint16_t az;        // Azimuth in degrees to true north (leading zeros sent); 000 to 359
    uint8_t snr;        // SNR (0..99, 255=NOT TRACKING)
} gps_gsv_sat_t;

// GSV - GNSS Satellites in View
typedef struct gps_gsv_s {
    uint16_t cnt;
    uint8_t num_sv;     // Number of space vehicles (satellites) in view for the talker ID and signal ID
    gps_gsv_sat_t sats[9];
} gps_gsv_t;

// GGA - GNSS Fix
typedef struct gps_gga_s {
    uint16_t cnt;
    float time;         // UTC time, typically 2 or 3 dp. Leading zeros are always included
    float lat;          // Latitude, typically 4 or 5 dp. Leading zeros are always included
    char ns;            // Hemispherical orientation N or S (north or south)
    float lng;          // Longitude, typically 4 or 5 dp. Leading zeros are always included
    char ew;            // Hemispherical orientation E or W (east or west)
    uint8_t pos_fix;
    uint8_t sat_used;   // Number of space vehicles (satellites) in use (00-12)
    float hdop;         // Horizontal dilution of precision (HDOP), typically 1 or 2 dp
    float altitude;     // Antenna altitude above/below mean-sea-level, typically 1 or 2 dp
    char alt_unit;      // Fixed field. M = meters
    float goid;         // Geoidal separation in meters, difference between ellipsoid and mean-sea-level
    char goid_unit;     // Fixed field. M = meters
    float age;          // Age of differential corrections in seconds (null when DGPS is not used)
    uint16_t diff_stat_id;  // Differential reference station ID; 0000-1023. Null (empty) or 0000 when DGPS not used
} gps_gga_t;

// VTG - Velocity and Track made Good
typedef struct gps_vtg_s {
    uint16_t cnt;
    float true_track;   // Course over ground in degrees (true), typically 2 dp
    char true_unit;     // Fixed field. T = degrees true
    float magnetic_track;  // Course over ground in degrees (magnetic). Null (empty) if unavailable.
    char magnetic_unit;    // Fixed field. M = degrees magnetic
    float speed_knots;  // Speed over ground in knots, typically 2 or 3 dp
    char knot_unit;     // Fixed field. N = Knots
    float speed_kmh;    // Speed over ground in km/h, typically 2 or 3 dp
    char kmh_unit;      // Fixed field. K = Kilometers per hour
} gps_vtg_t;

// ZDA - Zone Date and Time
typedef struct gps_zda_s {
    uint16_t cnt;
    float time;         // UTC time, typically 2 or 3 dp. Leading zeros are always included
    uint8_t year;       // UTC year
    uint8_t month;      // UTC month of the year; 01-12. Leading zeros are always included
    uint8_t day;        // UTC day of the month; 01-31. Leading zeros are always included
    int8_t zone_hours;  // Local time zone (hours); -13 to +13 hours
    int8_t zone_minutes;  // Local time zone (minutes); 00 to 59. Apply same sign as hours
} gps_zda_t;

typedef struct gps_status_s {
    const char *sat;    // Statellite types
    uint32_t date;      // Date DDMMYY
    uint32_t time;      // UTC time HHMMSS
    float lat;          // Latitude
    char ns;            // Hemispherical orientation N or S (north or south)
    float lng;          // Longitude
    char ew;            // Hemispherical orientation E or W (east or west)
    float altitude;     // Aitutde in m
    float speed;        // Speed over ground in km/h
    char mode_3d;       // Fix mode; 1 = no fix, 2 = 2D, 3 = 3D
    uint8_t sats;       // Number of satellites
    uint8_t status;     // Data status.
    uint16_t error_cnt;
} gps_status_t;

typedef struct __attribute__((packed)) gps_data_s {
    uint32_t date;      // Date DDMMYY
    uint32_t time;      // UTC time HHMMSS
    char status;        // Data status; A = active (data valid), V = void (receiver warning)
    float lat;          // Latitude, typically 4 or 5 dp. Leading zeros are always included
    char ns;            // Hemispherical orientation N or S (north or south)
    float lng;          // Longitude, typically 4 or 5 dp. Leading zeros are always included
    char ew;            // Hemispherical orientation E or W (east or west)
    float speed;        // Speed over ground in knots, typically 2 or 3 dp
    float course;       // Course over ground in degrees true, typically 2 dp
} gps_data_t;

typedef struct gps_sensor_s {
    char *name;
    uint8_t *buffer;
    uint8_t cnt;
    hw_serial_t *serial;
    char *messages;
    uint8_t msg_size;
    gps_rmc_t rmc;      // RMC - Recommended Minimum Navigation Information
    gps_gll_t gll;      // GLL - Geographic Latitude + Longitude
    gps_gsa_t gsa;      // GSA - GNSS Satellites Active
    gps_gsv_t gsv;      // GSV - GNSS Satellites in View
    gps_gga_t gga;      // GGA - GNSS Fix
    gps_vtg_t vtg;      // VTG - Velocity and Track made Good
    gps_zda_t zda;      // ZDA - Zone Date and Time
    gps_status_t status;
    uint8_t debug;
} gps_sensor_t;

esp_err_t gps_init(gps_sensor_t **sensor, uint8_t uart_num, uint8_t rx_pin, uint8_t tx_pin);

bool gps_data_ready(gps_sensor_t *sensor);

int gps_set_power_mode(gps_sensor_t *sensor, uint8_t mode);

int gps_power_off(gps_sensor_t *sensor);

void gps_dump_values(gps_sensor_t *sensor, bool force);

#ifdef __cplusplus
};
#endif
