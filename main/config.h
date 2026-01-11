/**
 * MIT License
 *
 * Copyright (C) 2026 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define CONFIG_VERSION 6

typedef struct wifi_networks_s {
    // Connection info for up to 4 SSIDs and their passwords
    char ssid[4][32];
    char password[4][64];
} wifi_networks_t;


typedef struct nvs_config_s {
    wifi_networks_t wifi;
} config_nvs_t;

typedef struct config_s {
    uint8_t cfg_version;  // Configuration version
    uint8_t auto_connect; // 0 base index of SSID to connect to in the known networks list. If index >3 then do not connect.
    bool auto_record;
    // Power mode after startup
    uint8_t lcd_pwr;
    uint8_t gps_pwr;
    uint8_t scd4x_pwr;
    uint8_t wifi_pwr;
    uint8_t mode_pwr;
    char mqtt_broker[32];
    bool mqtt_auto_connect;
    bool tcp_auto_start;
    bool ftp_auto_start;
    bool cfg_locked;
} config_t;

extern config_t *config;
extern config_nvs_t *config_nvs;

esp_err_t create_default_config_sd();

esp_err_t config_nvs_read(void);

esp_err_t config_nvs_write(void);

esp_err_t config_sd_read(void);

esp_err_t config_sd_write(void);

esp_err_t config_read(void);

esp_err_t config_write(void);

esp_err_t lcd_set_pwr_mode(uint8_t mode);

esp_err_t gps_set_pwr_mode(uint8_t mode);

esp_err_t scd4x_set_pwr_mode(uint8_t mode);

#ifdef __cplusplus
};
#endif
