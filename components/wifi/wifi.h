/**
 * WiFi Driver for Esspressif ESP-32.
 *
 * MIT License
 *
 * Copyright (C) 2025 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"

esp_err_t wifi_init(esp_netif_ip_info_t *ip_info);

#ifdef __cplusplus
};
#endif
