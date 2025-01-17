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

void rtc_init(i2c_master_bus_handle_t *bus_handle);

#ifdef __cplusplus
};
#endif
