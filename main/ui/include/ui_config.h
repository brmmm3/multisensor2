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

#include <esp_lvgl_port.h>
#include "main.h"

void ui_register_callbacks(ui_t *ui);

#ifdef __cplusplus
};
#endif
