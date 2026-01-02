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

esp_err_t wifi_init(bool scan);

void wifi_uninit();

bool wifi_initialized();

void wifi_disconnect();

void set_scanning(bool scanning);

#ifdef __cplusplus
};
#endif
