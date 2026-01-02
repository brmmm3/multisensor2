/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#loader-with-arc

#include "lcd.h"
#include "main.h"
#include "include/ui_update.h"

//static const char *TAG = "UIU";

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)


void update_air_tab()
{
    char buf[100];
    if (bmx280lo_update) {
        sprintf(buf, "P=%.1f hPa  A=%.1f m\nT=%.1f °C  H=%.1f %%",
                bmx280lo->values.pressure, bmx280lo->values.altitude, bmx280lo->values.temperature, bmx280lo->values.humidity);
        lv_label_set_text(ui->lbl_bmx280lo, buf);
    }
    if (bmx280hi_update) {
        sprintf(buf, "P=%.1f hPa  A=%.1f m\nT=%.1f °C  H=%.1f %%",
                bmx280hi->values.pressure, bmx280hi->values.altitude, bmx280hi->values.temperature, bmx280hi->values.humidity);
        lv_label_set_text(ui->lbl_bmx280hi, buf);
    }
    if (scd4x_update) {
        scd4x_values_t *scd4x_values = &scd4x->values;

        sprintf(buf, "CO2=%d ppm\nT=%.1f °C  H=%.1f %%", scd4x_values->co2, scd4x_values->temperature, scd4x_values->humidity);
        lv_label_set_text(ui->lbl_scd4x, buf);
    }
    if (mhz19_update) {
        sprintf(buf, "CO2=%d ppm\nT=%d °C ", mhz19->values.co2, mhz19->values.temp);
        lv_label_set_text(ui->lbl_mhz19, buf);
    }
    if (yys_update) {
        sprintf(buf, "O2=%.1f %%  CO=%d ppm\nH2S=%.1f ppm  CH4=%d ppm",
            yys_get_o2(yys_sensor), yys_get_co_raw(yys_sensor),
            yys_get_h2s(yys_sensor), yys_get_ch4_raw(yys_sensor));
        lv_label_set_text(ui->lbl_yys, buf);
    }
}

void update_dust_tab()
{
    if (sps30_update) {
        char buf[100];
        sps30_values_t *sps30_values = &sps30->values;

        sprintf(buf, "%.1f #/cm3", sps30_values->nc_0p5);
        lv_label_set_text(ui->lbl_sps30_1, buf);
        sprintf(buf, "%.1f ug/cm3 (%.1f #/cm3)", sps30_values->mc_1p0, sps30_values->nc_1p0);
        lv_label_set_text(ui->lbl_sps30_2, buf);
        sprintf(buf, "%.1f ug/cm3 (%.1f #/cm3)", sps30_values->mc_2p5, sps30_values->nc_2p5);
        lv_label_set_text(ui->lbl_sps30_3, buf);
        sprintf(buf, "%.1f ug/cm3 (%.1f #/cm3)", sps30_values->mc_4p0, sps30_values->nc_4p0);
        lv_label_set_text(ui->lbl_sps30_4, buf);
        sprintf(buf, "%.1f ug/cm3 (%.1f #/cm3)", sps30_values->mc_10p0, sps30_values->nc_10p0);
        lv_label_set_text(ui->lbl_sps30_5, buf);
        sprintf(buf, "%.3f um", sps30_values->typical_particle_size);
        lv_label_set_text(ui->lbl_sps30_6, buf);
    }
}

void update_gps_tab()
{
    if (gps_update) {
        char buf[100];
        uint32_t day;
        uint32_t month;
        uint32_t year;
        uint32_t hours;
        uint32_t minutes;
        uint32_t seconds;

        day = gps_status->date / 10000;
        month = gps_status->date / 100 - day * 100;
        year = gps_status->date % 100;
        sprintf(buf, "%02lu.%02lu.20%02lu", (unsigned long)day, (unsigned long)month, (unsigned long)year);
        lv_label_set_text(ui->lbl_gps_date, buf);
        hours = gps_status->time / 10000;
        minutes = gps_status->time / 100 - hours * 100;
        seconds = gps_status->time % 100;
        sprintf(buf, "%02lu:%02lu:%02lu", (unsigned long)hours, (unsigned long)minutes, (unsigned long)seconds);
        lv_label_set_text(ui->lbl_gps_time, buf);
        sprintf(buf, "%f %c", gps_status->lat, gps_status->ns);
        lv_label_set_text(ui->lbl_gps_lat, buf);
        sprintf(buf, "%f %c", gps_status->lng, gps_status->ew);
        lv_label_set_text(ui->lbl_gps_lng, buf);
        sprintf(buf, "%.1f m", gps_status->altitude);
        lv_label_set_text(ui->lbl_gps_alt, buf);
        sprintf(buf, "%.1f km/h", gps_status->speed);
        lv_label_set_text(ui->lbl_gps_speed, buf);
        sprintf(buf, "%d  ST=%d  2/3D=%s  %s", gps_status->sats, gps_status->status,
            gps_status->mode_3d == 2 ? "2D" : gps_status->mode_3d == 3 ? "3D": "-",
            gps_status->sat);
        lv_label_set_text(ui->lbl_gps_sats, buf);
    }
}

void update_wifi_tab()
{
}

void update_sd_tab()
{
}

void update_cfg_tab()
{
    char buf[100];

    // Update QMC5883L
    if (qmc5883l_update) {
        sprintf(buf, "X=%5.2f  Y=%5.2f  Z=%5.2f", qmc5883l->values.mag_x, qmc5883l->values.mag_y, qmc5883l->values.mag_z);
        lv_label_set_text(ui->lbl_qmc5883L, buf);
    }
    // Update ADXL345
    if (adxl345_update) {
        sprintf(buf, "%5.2f g  Moving=%d", adxl345->values.accel_abs, adxl345->moving_cnt);
        lv_label_set_text(ui->lbl_adxl345, buf);
    }
}

void ui_update()
{
    lv_lock_acquire();

    uint32_t tab_idx = lv_tabview_get_tab_active(ui->tbv_main);

    if (tab_idx == 0) {
        update_air_tab();
    } else if (tab_idx == 1) {
        update_dust_tab();
    } else if (tab_idx == 2) {
        update_gps_tab();
    } else if (tab_idx == 3) {
        update_wifi_tab();
    } else if (tab_idx == 4) {
        update_sd_tab();
    } else if (tab_idx == 5) {
        update_cfg_tab();
    }
    lv_lock_release();
}
