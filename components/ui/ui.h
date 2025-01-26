/**
 * UI - User Interface
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
#include "misc/lv_types.h"

typedef struct ui_s {
    lv_obj_t *tbv_main;
    // Tab Air
    lv_obj_t *tab_air;
    lv_obj_t *titles_bg;
    lv_obj_t *lbl_bmx280lo;
    lv_obj_t *lbl_bmx280hi;
    lv_obj_t *lbl_mhz19;
    lv_obj_t *lbl_scd4x;
    lv_obj_t *lbl_yys;
    // Tab Dust
    lv_obj_t *tab_dust;
    lv_obj_t *lbl_sps30_1;
    lv_obj_t *lbl_sps30_2;
    lv_obj_t *lbl_sps30_3;
    lv_obj_t *lbl_sps30_4;
    lv_obj_t *lbl_sps30_5;
    lv_obj_t *lbl_sps30_6;
    // Tab GPS
    lv_obj_t *lbl_gps_date;
    lv_obj_t *lbl_gps_time;
    lv_obj_t *lbl_gps_lat;
    lv_obj_t *lbl_gps_lng;
    lv_obj_t *lbl_gps_alt;
    lv_obj_t *lbl_gps_speed;
    lv_obj_t *lbl_gps_sats;
    // Tab WiFi
    lv_obj_t *tab_wifi;
    lv_obj_t *sw_wifi_enable;
    // Tab SD-Card
    lv_obj_t *tab_sd;
    // Tab Config
    lv_obj_t *tab_cfg;
    lv_obj_t *lbl_adxl345;
    lv_obj_t *lbl_qmc5883L;
    lv_obj_t *btn_calibrate;
} ui_t;

ui_t *ui_init(lv_display_t *disp, void(*btn_pressed)(lv_event_t *));

void label_set_text(lv_obj_t *lbl, const char *text);

#ifdef __cplusplus
};
#endif
