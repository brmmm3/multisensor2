/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#loader-with-arc

#include "ui_update.h"
#include "lcd.h"

static const char *TAG = "UI";

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)


void btn_calibrate_pressed(lv_event_t *e)
{
    esp_err_t err;
    uint16_t frc;
    const char *key = lv_event_get_user_data(e);

    if (!strcmp(key, "CAL")) {
        ESP_LOGI(TAG, "Calibrating sensors...");
        if ((err = mhz19_calibrate_zero(mhz19)) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to calibrate MHZ19 with error %d", err);
        }
        if ((err = scd4x_stop_periodic_measurement(scd4x)) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop SCD41 with error %d", err);
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
            frc = scd4x_perform_forced_recalibration(scd4x, mhz19->co2);
            ESP_LOGI(TAG, "FRC=%d", frc);
            vTaskDelay(pdMS_TO_TICKS(400));
            if ((err = scd4x_start_periodic_measurement(scd4x)) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start SCD41 with error %d", err);
            }
        }
        ESP_LOGI(TAG, "Calibrating sensors DONE");
    }
}

void sw_lcd_pwr_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if (code == LV_EVENT_VALUE_CHANGED) {
        if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
            lcd_set_bg_pwr(1);
        } else {
            lcd_set_bg_pwr(2);
        }
    }
}

void sw_gps_pwr_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if (code == LV_EVENT_VALUE_CHANGED) {
        if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
            gps_set_power_mode(gps, 0);
        } else {
            gps_set_power_mode(gps, 2);
        }
    }
}

void sw_wifi_pwr_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if (code == LV_EVENT_VALUE_CHANGED) {
        if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
            wifi_init();
        } else {
            //esp_http_client_cleanup(client); // dismiss the TCP stack
            esp_wifi_disconnect();             // break connection to AP
            esp_wifi_stop();                   // shut down the wifi radio
            esp_wifi_deinit();                 // release wifi resources
            lv_lock_acquire();
            lv_label_set_text(ui->lbl_wifi_status, "Disconnected");
            lv_lock_release();
        }
    }
}

void sw_sps30_pwr_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if (code == LV_EVENT_VALUE_CHANGED) {
        if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
            sps30_stop_measurement(sps30);
        } else {
            sps30_start_measurement(sps30);
        }
    }
}

void sw_scd4x_pwr_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if (code == LV_EVENT_VALUE_CHANGED) {
        if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
            scd4x_power_down(scd4x);
        } else {
            scd4x_device_init_do(scd4x);
        }
    }
}

void update_air_tab() {
    char buf[100];
    if (bmx280lo_update) {
        bmx280lo_update = false;
        sprintf(buf, "P=%.1f hPa  A=%.1f m\nT=%.1f °C  H=%.1f %%",
                bmx280lo->values.pressure, bmx280lo->values.altitude, bmx280lo->values.temperature, bmx280lo->values.humidity);
        lv_label_set_text(ui->lbl_bmx280lo, buf);
    }
    if (bmx280hi_update) {
        bmx280hi_update = false;
        sprintf(buf, "P=%.1f hPa  A=%.1f m\nT=%.1f °C  H=%.1f %%",
                bmx280hi->values.pressure, bmx280hi->values.altitude, bmx280hi->values.temperature, bmx280hi->values.humidity);
        lv_label_set_text(ui->lbl_bmx280hi, buf);
    }
    if (scd4x_update) {
        scd4x_values_t *scd4x_values = &scd4x->values;

        scd4x_update = false;
        sprintf(buf, "CO2=%d ppm\nT=%.1f °C  H=%.1f %%", scd4x_values->co2, scd4x_values->temperature, scd4x_values->humidity);
        lv_label_set_text(ui->lbl_scd4x, buf);
    }
    if (mhz19_update) {
        mhz19_update = false;
        sprintf(buf, "CO2=%d ppm\nT=%d °C ", mhz19->co2, mhz19->temp);
        lv_label_set_text(ui->lbl_mhz19, buf);
    }
    if (yys_update) {
        yys_update = false;
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

        sps30_update = false;
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

        gps_update = false;
        day = gps_status->date / 10000;
        month = gps_status->date / 100 - day * 100;
        year = gps_status->date % 100;
        sprintf(buf, "%02lu.%02lu.20%02lu", day, month, year);
        lv_label_set_text(ui->lbl_gps_date, buf);
        hours = gps_status->time / 10000;
        minutes = gps_status->time / 100 - hours * 100;
        seconds = gps_status->time % 100;
        sprintf(buf, "%02lu:%02lu:%02lu", hours, minutes, seconds);
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
        qmc5883l_update = false;
        sprintf(buf, "X=%5.2f  Y=%5.2f  Z=%5.2f", qmc5883l->mag_x, qmc5883l->mag_y, qmc5883l->mag_z);
        lv_label_set_text(ui->lbl_qmc5883L, buf);
    }
    // Update ADXL345
    if (adxl345_update) {
        adxl345_update = false;
        sprintf(buf, "%5.2f g  Moving=%d", adxl345->accel_abs, adxl345->moving_cnt);
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

void ui_register_cb()
{
    lv_obj_add_event_cb(ui->sw_wifi_enable, sw_wifi_pwr_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui->sw_lcd_pwr, sw_lcd_pwr_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui->sw_gps_pwr, sw_gps_pwr_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui->sw_sps30_pwr, sw_sps30_pwr_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui->sw_scd4x_pwr, sw_scd4x_pwr_cb, LV_EVENT_ALL, NULL);
}