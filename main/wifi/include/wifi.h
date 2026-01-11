/**
 * WiFi
 *
 * MIT License
 *
 * Copyright (C) 2024 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#pragma once

#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi_sntp.h"
#include "wifi_scan.h"

typedef struct wifi_network_s {
    const char *ssid;
    const char *password;
} wifi_network_t;

extern bool wifi_connected;

const char *wifi_ip();

int8_t wifi_get_rssi();

bool wifi_netif_enabled();

esp_err_t wifi_init(bool scan);

void wifi_uninit();

bool wifi_initialized();

esp_err_t wifi_connect(const char *ssid, const char *password);

void wifi_disconnect();

void set_scanning(bool scanning);

esp_err_t wifi_set_pwr_mode(uint8_t mode);

#ifdef __cplusplus
};
#endif
