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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

esp_err_t wifi_init();

void wifi_scan(void);

void set_scanning(bool scanning);

#ifdef __cplusplus
};
#endif
