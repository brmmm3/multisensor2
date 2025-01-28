/**
 * LCD
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

#include "lvgl.h"

void lv_lock_acquire();

void lv_lock_release();

lv_display_t *lcd_init(int spi_host_id, uint8_t cs_pin, uint8_t dc_pin, uint8_t reset_pin, uint8_t led_pin, uint8_t t_cs_pin);

void lcd_set_bg_pwr(uint8_t mode);

#ifdef __cplusplus
};
#endif
