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

void ui_set_label_text(lv_obj_t *obj, const char *text);

void ui_set_switch_state(lv_obj_t *obj, bool enabled);

void ui_set_text_color(lv_obj_t *obj, int color);

void ui_set_tab_color(int index, int color);

void ui_remove_style(lv_obj_t *obj, lv_style_t *style);

void ui_list_clear(lv_obj_t *obj);

lv_obj_t *ui_list_add(lv_obj_t *obj, const char *symbol, const char *text);

esp_err_t ui_lcd_set_pwr_mode(uint8_t mode);

esp_err_t ui_gps_set_pwr_mode(uint8_t mode);

esp_err_t ui_scd4x_set_pwr_mode(uint8_t mode);

esp_err_t ui_wifi_set_pwr_mode(uint8_t mode);

esp_err_t ui_mode_set_pwr_mode(uint8_t mode);

void ui_register_callbacks(ui_t *ui);

#ifdef __cplusplus
};
#endif
