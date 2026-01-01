/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#loader-with-arc

#include <string.h>
#include <esp_log.h>
#include "main.h"
#include "ui_config.h"

static const char *TAG = "UIC";

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)


static void btn_calibrate_pressed(lv_event_t *e)
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

static void sw_lcd_pwr_cb(lv_event_t * e)
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

static void sw_gps_pwr_cb(lv_event_t *e)
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

static void sw_wifi_pwr_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    ESP_LOGI(TAG, "sw_wifi_pwr_cb %d", code);
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

static void sw_sps30_pwr_cb(lv_event_t *e)
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

static void sw_scd4x_pwr_cb(lv_event_t *e)
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

static void tabview_event_cb(lv_event_t * e)
{
    ui_update();
}

void ui_register_callbacks(ui_t *ui)
{
    lv_obj_add_event_cb(ui->btn_calibrate, btn_calibrate_pressed, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui->tbv_main, tabview_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sw_wifi_enable, sw_wifi_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sw_lcd_pwr, sw_lcd_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sw_gps_pwr, sw_gps_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sw_sps30_pwr, sw_sps30_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sw_scd4x_pwr, sw_scd4x_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
}