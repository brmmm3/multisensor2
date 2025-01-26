/**
 * SNTP
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
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"

typedef struct sntp_s {
    struct tm timeinfo;
} sntp_t;

struct tm *sntp_get_timeinfo();

esp_err_t sntp_obtain_time(void);

#ifdef __cplusplus
};
#endif
