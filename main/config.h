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

typedef struct config_s {
    uint8_t cfg_version;  // Configuration version
    // WiFi
    bool auto_connect;
    char ssid[4][32];
    char password[4][64];
} config_t;

extern config_t *config;

esp_err_t config_read(config_t *config);

esp_err_t config_write(config_t *config);

#ifdef __cplusplus
};
#endif
