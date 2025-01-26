/**
 * RTC
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
#include "driver/i2c_types.h"

typedef struct rtc_s {
    void *rtc;
} rtc_t;

esp_err_t rtc_init(rtc_t **rtc_ptr, i2c_master_bus_handle_t *bus_handle);

#ifdef __cplusplus
};
#endif
