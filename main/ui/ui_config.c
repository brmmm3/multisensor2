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
#include "core/lv_obj_style_gen.h"
#include "freertos/projdefs.h"
#include "main.h"

static const char *TAG = "UIC";

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)


void ui_set_label_text(lv_obj_t *obj, const char *text)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return;
    lv_label_set_text(obj, text);
    lvgl_port_unlock();
}


void ui_set_switch_state(lv_obj_t *obj, bool enabled)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return;
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
    static bool style_init = false;

    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return;
    if (!style_init) {
        lv_style_init(&style);
        style_init = true;
    }
    //lv_style_set_text_opa(&style, LV_OPA_COVER);
    lv_style_set_text_color(&style, lv_palette_main(color));  // Any color
    //lv_obj_remove_style(obj, &style, 0);
    lv_obj_add_style(obj, &style, 0);
    lvgl_port_unlock();
}


esp_err_t ui_set_current_tab(uint32_t tab_num)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    lv_tabview_set_act(ui->tbv_main, tab_num, LV_ANIM_OFF);
    lvgl_port_unlock();
    return ESP_OK;
}


void ui_set_tab_color(int index, int color)
{
    static lv_style_t tab_styles[6];
    static bool tab_styles_init[6] = {false};
    lv_style_t *style = &tab_styles[index];

    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return;
    if (!tab_styles_init[index]) {
        lv_style_init(style);
        tab_styles_init[index] = true;
    }
    //lv_style_set_text_opa(&style, LV_OPA_COVER);
    lv_style_set_text_color(style, lv_palette_main(color));  // Any color
    lv_obj_t *tab_bar = lv_tabview_get_tab_bar(ui->tbv_main);
    lv_obj_t *btn = lv_obj_get_child(tab_bar, index);
    lv_obj_add_style(btn, style, 0);
    lvgl_port_unlock();
}


void ui_remove_style(lv_obj_t *obj, lv_style_t *style)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return;
    lv_obj_remove_style(obj, style, 0);
    lvgl_port_unlock();
}


void ui_list_clear(lv_obj_t *obj)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return;
    lv_obj_clean(obj);
    lvgl_port_unlock();
}


lv_obj_t *ui_list_add(lv_obj_t *obj, const char *symbol, const char *text)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return NULL;
    obj = lv_list_add_button(obj, symbol, text);
    lvgl_port_unlock();
    return obj;
}

esp_err_t ui_lcd_set_pwr_mode(uint8_t mode)
{
    esp_err_t err;

    if ((err = lcd_set_pwr_mode(mode)) != ESP_OK) return err;
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    lv_slider_set_value(ui->sl_lcd_pwr, config->lcd_pwr, LV_ANIM_OFF);
    lvgl_port_unlock();
    return ESP_OK;
}

esp_err_t ui_gps_set_pwr_mode(uint8_t mode)
{
    esp_err_t err;

    if ((err = gps_set_pwr_mode(mode)) != ESP_OK) return err;
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    lv_slider_set_value(ui->sl_gps_pwr, config->gps_pwr, LV_ANIM_OFF);
    lvgl_port_unlock();
    return ESP_OK;
}

esp_err_t ui_scd4x_set_pwr_mode(uint8_t mode)
{
    esp_err_t err;

    if ((err = scd4x_set_pwr_mode(mode)) != ESP_OK) return err;
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    lv_slider_set_value(ui->sl_scd4x_pwr, config->scd4x_pwr, LV_ANIM_OFF);
    lvgl_port_unlock();
    return ESP_OK;
}

esp_err_t ui_wifi_set_pwr_mode(uint8_t mode)
{
    esp_err_t err;

    if (mode > 2) mode = 2;
    if ((err = wifi_set_pwr_mode(mode)) != ESP_OK) return err;
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    lv_slider_set_value(ui->sl_wifi_pwr, config->wifi_pwr, LV_ANIM_OFF);
    lvgl_port_unlock();
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


esp_err_t ui_lock_obj(lv_obj_t *obj, bool lock)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    if (lock) lv_obj_add_state(obj, LV_STATE_DISABLED);
    else lv_obj_clear_state(obj, LV_STATE_DISABLED);
    lvgl_port_unlock();
    return ESP_OK;
}


void ui_config_lock(bool lock)
{
    config->cfg_locked = lock;
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_lock_obj(ui->sw_wifi_enable, lock));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_lock_obj(ui->sw_record, lock));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_lock_obj(ui->sw_auto_record, lock));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_lock_obj(ui->sl_lcd_pwr, lock));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_lock_obj(ui->sl_gps_pwr, lock));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_lock_obj(ui->sl_scd4x_pwr, lock));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_lock_obj(ui->sl_wifi_pwr, lock));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_lock_obj(ui->sl_mode_pwr, lock));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_lock_obj(ui->btn_calibrate, lock));
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
        ui_set_switch_state(ui->sw_record, true);
    } else {
        status.recording = false;
        ui_set_tab_color(4, LV_PALETTE_GREY);
        ui_set_switch_state(ui->sw_record, false);
    }
}


void ui_set_time_value(lv_obj_t *obj, time_t *time)
{
    if (*time == 0) {
        ui_set_label_text(obj, "-");
        return;
    }

    char buf[32];
    struct tm *t = localtime(time);
    uint16_t year = t->tm_year + 1900;

    sprintf(buf, "%d.%02d.%02d  %02d:%02d:%02d", year, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
    ui_set_label_text(obj, buf);
}


void ui_set_duration_value(lv_obj_t *obj, uint32_t duration)
{
    char buf[25];
    uint8_t secs = duration % 60;
    duration /= 60;
    uint8_t mins = duration % 60;
    duration /= 60;
    uint8_t hours = duration % 24;
    duration /= 24;

    sprintf(buf, "%lu days  %02d:%02d:%02d", (unsigned long)duration, hours, mins, secs);
    ui_set_label_text(obj, buf);
}


void ui_set_dop_value(lv_obj_t *obj, float dop)
{
    char buf[20];
    const char *quality = "POOR";

    if (dop < 2) quality = "EXCELLENT";
    else if (dop < 6) quality = "GOOD";
    else if (dop < 11) quality = "MODERATE";
    else if (dop < 21) quality = "FAIR";
    sprintf(buf, "%.1f  %s", dop, quality);
    ui_set_label_text(obj, buf);
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
    ESP_LOGI(TAG, "Calibrating sensors...");
    /*if ((err = mhz19_calibrate_zero(mhz19)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate MHZ19 with error %d", err);
    }*/
    scd4x_state_machine_cmd(SCD4X_CMD_FRC, mhz19->values.co2);
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


static void sw_config_lock_cb(lv_event_t *e)
{
    lv_obj_t * obj = lv_event_get_target(e);

    if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
        ui_config_lock(true);
    } else {
        ui_config_lock(false);
    }
}


static void tabview_event_cb(lv_event_t *e)
{
    status.force_update = true;
}


static void screen_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        status.tap_time = time(NULL);
        lcd_set_bg_pwr(0);
    }
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
    lv_obj_add_event_cb(ui->sw_cfg_lock, sw_config_lock_cb, LV_EVENT_VALUE_CHANGED, NULL);
    // Tap event
    lv_obj_add_event_cb(lv_screen_active(), screen_event_cb, LV_EVENT_CLICKED, NULL);
}
