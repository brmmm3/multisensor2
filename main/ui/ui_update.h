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

#include "main.h"

void btn_calibrate_pressed(lv_event_t *e);

void ui_update();

void ui_register_cb();

#ifdef __cplusplus
};
#endif
