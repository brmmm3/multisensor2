/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#loader-with-arc

#include <string.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include "esp_err.h"
#include "misc/lv_types.h"
#include "main.h"
#include "include/ui.h"

static const char *TAG = "UIC";

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

static lv_style_t tab_styles[6];


void ui_set_label_text(lv_obj_t *obj, const char *text)
{
    lv_lock_acquire();
    lv_label_set_text(obj, text);
    lv_lock_release();
}


void ui_set_switch_state(lv_obj_t *obj, bool enabled)
{
    lv_lock_acquire();
    if (enabled) {
        lv_obj_add_state(obj, LV_STATE_CHECKED);
    } else {
        lv_obj_remove_state(obj, LV_STATE_CHECKED);
    }
    lv_lock_release();
}


void ui_set_text_color(lv_obj_t *obj, int color)
{
    static lv_style_t style;
    lv_lock_acquire();
    lv_style_init(&style);
    //lv_style_set_text_opa(&style, LV_OPA_COVER);
    lv_style_set_text_color(&style, lv_palette_main(color));  // Any color
    lv_obj_add_style(obj, &style, 0);
    lv_lock_release();
}


void ui_set_tab_color(int index, int color)
{
    lv_style_t *style = &tab_styles[index];
    lv_lock_acquire();
    lv_style_init(style);
    //lv_style_set_text_opa(&style, LV_OPA_COVER);
    lv_style_set_text_color(style, lv_palette_main(color));  // Any color
    lv_obj_t *tab_bar = lv_tabview_get_tab_bar(ui->tbv_main);
    lv_obj_t *btn = lv_obj_get_child(tab_bar, index);
    lv_obj_add_style(btn, style, 0);
    lv_lock_release();
}


void ui_remove_style(lv_obj_t *obj, lv_style_t *style)
{
    lv_lock_acquire();
    lv_obj_remove_style(obj, style, 0);
    lv_lock_release();
}


void ui_list_clear(lv_obj_t *obj)
{
    lv_lock_acquire();
    lv_obj_clean(obj);
    lv_lock_release();
}


lv_obj_t *ui_list_add(lv_obj_t *obj, const char *symbol, const char *text)
{
    return lv_list_add_button(obj, symbol, text);
}


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
    lv_obj_t * obj = lv_event_get_target(e);

    if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
        lcd_set_bg_pwr(1);
    } else {
        lcd_set_bg_pwr(2);
    }
}

static void sw_gps_pwr_cb(lv_event_t *e)
{
    lv_obj_t * obj = lv_event_get_target(e);

    if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
        gps_set_power_mode(gps, 0);
    } else {
        gps_set_power_mode(gps, 2);
    }
}

static void sw_wifi_pwr_cb(lv_event_t *e)
{
    lv_obj_t * obj = lv_event_get_target(e);

    if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
        wifi_init(true);
    } else {
        wifi_uninit();
    }
}

static void sw_sd_record_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);

    if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
        status.recording = true;
        status.record_pos = 0;
        set_data_filename();
        ui_set_tab_color(4, LV_PALETTE_GREEN);
        ui_set_label_text(ui->lbl_sd_fill, "0 / 65000");
    } else {
        status.recording = false;
        ui_set_tab_color(4, LV_PALETTE_GREY);
    }
}

static void sw_sd_auto_record_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);

    if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
        config->auto_record = true;
    } else {
        config->auto_record = false;
    }
}

static void sw_sps30_pwr_cb(lv_event_t *e)
{
    lv_obj_t * obj = lv_event_get_target(e);

    if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
        sps30_stop_measurement(sps30);
    } else {
        sps30_start_measurement(sps30);
    }
}

static void sw_scd4x_pwr_cb(lv_event_t *e)
{
    lv_obj_t * obj = lv_event_get_target(e);

    if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
        scd4x_power_down(scd4x);
    } else {
        scd4x_device_init_do(scd4x);
    }
}

static void tabview_event_cb(lv_event_t * e)
{
    ui_update();
}

void ui_register_callbacks(ui_t *ui)
{
    lv_obj_add_event_cb(ui->tbv_main, tabview_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    // Tab WiFi
    lv_obj_add_event_cb(ui->sw_wifi_enable, sw_wifi_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    // Tab SD
    lv_obj_add_event_cb(ui->sw_record, sw_sd_record_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sw_auto_record, sw_sd_auto_record_cb, LV_EVENT_VALUE_CHANGED, NULL);
    // Tab CFG
    lv_obj_add_event_cb(ui->sw_lcd_pwr, sw_lcd_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sw_gps_pwr, sw_gps_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sw_sps30_pwr, sw_sps30_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sw_scd4x_pwr, sw_scd4x_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->btn_calibrate, btn_calibrate_pressed, LV_EVENT_CLICKED, NULL);
}
