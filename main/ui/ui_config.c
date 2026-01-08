/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#loader-with-arc

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include <misc/lv_types.h>
#include <esp_lvgl_port.h>
#include "main.h"

static const char *TAG = "UIC";

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

static lv_style_t tab_styles[6];


void ui_set_label_text(lv_obj_t *obj, const char *text)
{
    lvgl_port_lock(-1);
    lv_label_set_text(obj, text);
    lvgl_port_unlock();
}


void ui_set_switch_state(lv_obj_t *obj, bool enabled)
{
    lvgl_port_lock(-1);
    if (enabled) {
        lv_obj_add_state(obj, LV_STATE_CHECKED);
    } else {
        lv_obj_remove_state(obj, LV_STATE_CHECKED);
    }
    lvgl_port_unlock();
}


void ui_set_text_color(lv_obj_t *obj, int color)
{
    static lv_style_t style;
    lvgl_port_lock(-1);
    lv_style_init(&style);
    //lv_style_set_text_opa(&style, LV_OPA_COVER);
    lv_style_set_text_color(&style, lv_palette_main(color));  // Any color
    lv_obj_add_style(obj, &style, 0);
    lvgl_port_unlock();
}


void ui_set_tab_color(int index, int color)
{
    lv_style_t *style = &tab_styles[index];
    lvgl_port_lock(-1);
    lv_style_init(style);
    //lv_style_set_text_opa(&style, LV_OPA_COVER);
    lv_style_set_text_color(style, lv_palette_main(color));  // Any color
    lv_obj_t *tab_bar = lv_tabview_get_tab_bar(ui->tbv_main);
    lv_obj_t *btn = lv_obj_get_child(tab_bar, index);
    lv_obj_add_style(btn, style, 0);
    lvgl_port_unlock();
}


void ui_remove_style(lv_obj_t *obj, lv_style_t *style)
{
    lvgl_port_lock(-1);
    lv_obj_remove_style(obj, style, 0);
    lvgl_port_unlock();
}


void ui_list_clear(lv_obj_t *obj)
{
    lvgl_port_lock(-1);
    lv_obj_clean(obj);
    lvgl_port_unlock();
}


lv_obj_t *ui_list_add(lv_obj_t *obj, const char *symbol, const char *text)
{
    lvgl_port_lock(-1);
    obj = lv_list_add_button(obj, symbol, text);
    lvgl_port_unlock();
    return obj;
}

esp_err_t ui_lcd_set_pwr_mode(uint8_t mode)
{
    esp_err_t err;

    if ((err = lcd_set_pwr_mode(mode)) != ESP_OK) {
        return err;
    }
    lv_slider_set_value(ui->sl_lcd_pwr, config->lcd_pwr, LV_ANIM_OFF);
    return ESP_OK;
}

esp_err_t ui_gps_set_pwr_mode(uint8_t mode)
{
    esp_err_t err;

    if ((err = gps_set_pwr_mode(mode)) != ESP_OK) {
        return err;
    }
    lv_slider_set_value(ui->sl_gps_pwr, config->gps_pwr, LV_ANIM_OFF);
    return ESP_OK;
}

esp_err_t ui_scd4x_set_pwr_mode(uint8_t mode)
{
    esp_err_t err;

    if ((err = scd4x_set_pwr_mode(mode)) != ESP_OK) {
        return err;
    }
    lv_slider_set_value(ui->sl_scd4x_pwr, config->scd4x_pwr, LV_ANIM_OFF);
    return ESP_OK;
}

esp_err_t ui_wifi_set_pwr_mode(uint8_t mode)
{
    esp_err_t err;

    if (mode > 2) {
        mode = 2;
    }
    if ((err = wifi_set_pwr_mode(mode)) != ESP_OK) {
        return err;
    }
    lv_slider_set_value(ui->sl_wifi_pwr, config->wifi_pwr, LV_ANIM_OFF);
    return ESP_OK;
}

esp_err_t ui_mode_set_pwr_mode(uint8_t mode)
{
    esp_err_t err;

    if (mode < 4) {
        if ((err = ui_lcd_set_pwr_mode(mode)) != ESP_OK) return err;
        if ((err = ui_gps_set_pwr_mode(mode)) != ESP_OK) return err;
        if ((err = ui_scd4x_set_pwr_mode(mode)) != ESP_OK) return err;
        if ((err = ui_wifi_set_pwr_mode(mode)) != ESP_OK) return err;
    }
    return ESP_OK;
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

void ui_sd_record_set_value(bool enable)
{
    if (enable) {
        status.recording = true;
        status.record_pos = 0;
        set_data_filename();
        ui_set_tab_color(4, LV_PALETTE_GREEN);
        ui_set_label_text(ui->lbl_sd_fill, "0.0 %");
    } else {
        status.recording = false;
        ui_set_tab_color(4, LV_PALETTE_GREY);
    }
}

static void sw_sd_record_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);

    if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
        ui_sd_record_set_value(true);
    } else {
        ui_sd_record_set_value(false);
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

static void sl_lcd_pwr_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    int32_t mode = lv_slider_get_value(obj);

    lcd_set_bg_pwr(mode);
}

static void sl_gps_pwr_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    int32_t mode = lv_slider_get_value(obj);

    gps_set_pwr_mode(mode);
}

static void sl_scd4x_pwr_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    int32_t mode = lv_slider_get_value(obj);

    scd4x_set_pwr_mode(mode);
}

static void sl_wifi_pwr_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    int32_t mode = lv_slider_get_value(obj);

    wifi_set_pwr_mode(mode);
}

static void sl_mode_pwr_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    int32_t mode = lv_slider_get_value(obj);

    ui_lcd_set_pwr_mode(mode);
    ui_gps_set_pwr_mode(mode);
    ui_scd4x_set_pwr_mode(mode);
    ui_wifi_set_pwr_mode(mode > 2 ? 2 : mode);
    config->mode_pwr = mode;
}

static void btn_calibrate_pressed(lv_event_t *e)
{
    esp_err_t err;
    uint16_t frc;

    ESP_LOGI(TAG, "Calibrating sensors...");
    if ((err = mhz19_calibrate_zero(mhz19)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate MHZ19 with error %d", err);
    }
    if ((err = scd4x_stop_periodic_measurement(scd4x)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop SCD41 with error %d", err);
    } else {
        vTaskDelay(pdMS_TO_TICKS(500));
        frc = scd4x_perform_forced_recalibration(scd4x, mhz19->values.co2);
        ESP_LOGI(TAG, "FRC=%d", frc);
        vTaskDelay(pdMS_TO_TICKS(400));
        if ((err = scd4x_start_periodic_measurement(scd4x)) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start SCD4x err=%d", err);
        }
    }
    ESP_LOGI(TAG, "Calibrating sensors DONE");
}

static void btn_save_config_pressed(lv_event_t *e)
{
    esp_err_t err;

    if ((err = config_write()) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write config err=%d", err);
    } else {
        ESP_LOGI(TAG, "Config written");
    }
}

static void tabview_event_cb(lv_event_t *e)
{
    status.force_update = true;
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
    lv_obj_add_event_cb(ui->sl_lcd_pwr, sl_lcd_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sl_gps_pwr, sl_gps_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sl_scd4x_pwr, sl_scd4x_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sl_wifi_pwr, sl_wifi_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->sl_mode_pwr, sl_mode_pwr_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui->btn_calibrate, btn_calibrate_pressed, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui->btn_save_config, btn_save_config_pressed, LV_EVENT_CLICKED, NULL);
}
