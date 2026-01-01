/**
 * WiFi
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
#include <stdbool.h>
#include "esp_err.h"

typedef struct wifi_networks_s {
    // Connection info for up to 4 SSIDs and their passwords
    char ssid[4][32];
    char password[4][64];
} wifi_networks_t;


typedef struct nvs_config_s {
    wifi_networks_t wifi;
} nvs_config_t;

typedef struct config_s {
    uint8_t cfg_version;  // Configuration version
    uint8_t auto_connect; // 0 base index of SSID to connect to in the known networks list. If index >3 then do not connect.
    nvs_config_t nvs;
} config_t;

extern config_t *config;

esp_err_t config_read(void);

esp_err_t config_write(void);

void register_config_cmd(void);

#ifdef __cplusplus
};
#endif
