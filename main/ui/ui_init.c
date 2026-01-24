/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#loader-with-arc

#include "core/lv_obj_style.h"
#include "esp_log.h"

#include "misc/lv_area.h"
#include "misc/lv_color.h"
#include "misc/lv_palette.h"
#include "widgets/label/lv_label.h"

#include "include/ui.h"

static const char *TAG = "UII";

static lv_style_t style_section;


void init_styles()
{
    lv_style_init(&style_section);
    lv_style_set_bg_opa(&style_section, LV_OPA_COVER);
    lv_style_set_bg_color(&style_section, lv_palette_main(LV_PALETTE_LIGHT_GREEN));
    lv_style_set_bg_grad_color(&style_section, lv_palette_main(LV_PALETTE_GREEN));
    lv_style_set_bg_grad_dir(&style_section, LV_GRAD_DIR_VER);
}

lv_obj_t *add_button(lv_obj_t *scr, int32_t x, int32_t y, int32_t w, int32_t h, const char *text)
{
    /*Init the style for the default state*/
    static lv_style_t style;

    lv_style_init(&style);

    lv_style_set_radius(&style, 3);

    lv_style_set_bg_opa(&style, LV_OPA_100);
    lv_style_set_bg_color(&style, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_color(&style, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_bg_grad_dir(&style, LV_GRAD_DIR_VER);

    lv_style_set_border_opa(&style, LV_OPA_40);
    lv_style_set_border_width(&style, 2);
    lv_style_set_border_color(&style, lv_palette_main(LV_PALETTE_GREY));

    lv_style_set_shadow_width(&style, 8);
    lv_style_set_shadow_color(&style, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_shadow_offset_y(&style, 8);

    lv_style_set_outline_opa(&style, LV_OPA_COVER);
    lv_style_set_outline_color(&style, lv_palette_main(LV_PALETTE_BLUE));

    lv_style_set_text_color(&style, lv_color_white());
    lv_style_set_pad_all(&style, 10);

    /*Init the pressed style*/
    static lv_style_t style_pr;

    lv_style_init(&style_pr);

    /*Add a large outline when pressed*/
    lv_style_set_outline_width(&style_pr, 30);
    lv_style_set_outline_opa(&style_pr, LV_OPA_TRANSP);

    lv_style_set_translate_y(&style_pr, 5);
    lv_style_set_shadow_offset_y(&style_pr, 3);
    lv_style_set_bg_color(&style_pr, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_bg_grad_color(&style_pr, lv_palette_darken(LV_PALETTE_BLUE, 4));

    /*Add a transition to the outline*/
    static lv_style_transition_dsc_t trans;
    static lv_style_prop_t props[] = {LV_STYLE_OUTLINE_WIDTH, LV_STYLE_OUTLINE_OPA, 0};

    lv_style_transition_dsc_init(&trans, props, lv_anim_path_linear, 300, 0, NULL);
    lv_style_set_transition(&style_pr, &trans);

    lv_obj_t *btn = lv_button_create(scr);

    lv_obj_remove_style_all(btn);                          /*Remove the style coming from the theme*/
    lv_obj_add_style(btn, &style, 0);
    lv_obj_add_style(btn, &style_pr, LV_STATE_PRESSED);
    lv_obj_set_size(btn, w > 0 ? w : LV_SIZE_CONTENT, h > 0 ? h : LV_SIZE_CONTENT);
    lv_obj_center(btn);

    lv_obj_t *label = lv_label_create(btn);

    lv_label_set_text(label, text);
    lv_obj_center(label);
    lv_obj_align(btn, LV_ALIGN_DEFAULT, x, y);
    return btn;
}

static void set_angle(void * obj, int32_t v)
{
    lv_arc_set_value(obj, v);
}

void create_animated_arc(lv_obj_t *scr)
{
    /*Create an Arc*/
    lv_obj_t *arc = lv_arc_create(scr);
    lv_arc_set_rotation(arc, 270);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    lv_obj_center(arc);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, arc);
    lv_anim_set_exec_cb(&a, set_angle);
    lv_anim_set_duration(&a, 1000);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);    /*Just for the demo*/
    lv_anim_set_repeat_delay(&a, 500);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_start(&a);
}

lv_obj_t *add_label(lv_obj_t *scr, int32_t x, int32_t y)
{
    lv_obj_t *obj = lv_label_create(scr);

    lv_obj_set_pos(obj, x, y);
    lv_label_set_text(obj, "-");
    return obj;
}

lv_obj_t *add_label_text(lv_obj_t *scr, int32_t x, int32_t y, const char *text, lv_color_t color)
{
    lv_obj_t *obj = add_label(scr, 0, 0);

    lv_obj_set_pos(obj, x, y);
    lv_label_set_text(obj, text);
    lv_obj_set_style_text_color(obj, color, LV_PART_MAIN);
    return obj;
}

lv_obj_t *add_rectangle(lv_obj_t *scr, int32_t x, int32_t y, int32_t w, int32_t h)
{
    lv_obj_t *obj = lv_obj_create(scr);

    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
    lv_obj_set_pos(obj , x, y);
    lv_obj_set_size(obj , w, h);
    return obj;
}

lv_obj_t *add_filled_rectangle(lv_obj_t *scr, int32_t x, int32_t y, int32_t w, int32_t h, lv_color_t color)
{
    lv_obj_t *obj = lv_obj_create(scr);

    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
    lv_obj_set_pos(obj , x, y);
    lv_obj_set_size(obj , w, h);
    lv_obj_set_style_bg_color(obj , color, LV_PART_MAIN);
    return obj;
}

lv_obj_t *add_switch(lv_obj_t *scr, int32_t x, int32_t y, int32_t w, int32_t h)
{
    lv_obj_t *obj = lv_switch_create(scr);

    lv_obj_set_style_bg_color(obj, lv_palette_darken(LV_PALETTE_GREY, 1), 0);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, w, h);
    return obj;
}

lv_obj_t *add_slider(lv_obj_t *scr, int32_t x, int32_t y, int32_t w, int32_t value, int32_t max)
{
    lv_obj_t *obj = lv_slider_create(scr);

    lv_slider_set_mode(obj, LV_SLIDER_MODE_NORMAL);
    lv_slider_set_range(obj, 0, max);
    lv_slider_set_value(obj, value, LV_ANIM_OFF);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_width(obj, w);
    return obj;
}

lv_obj_t *add_section(lv_obj_t *scr, int32_t x, int32_t y, int32_t w, int32_t h, int32_t w0, const char *text)
{
    lv_obj_t *obj1 = add_rectangle(scr, x, y, w, h);

    add_filled_rectangle(obj1, w0, 1, w - w0 - 6, h - 6, lv_color_white());

    lv_obj_add_style(obj1, &style_section, LV_PART_MAIN);
    add_label_text(obj1, 2, h / 2 - 9, text, lv_color_white());
    return obj1;
}

lv_obj_t *add_tabiew(lv_obj_t *scr, int32_t x, int32_t y)
{
    lv_obj_t *obj = lv_tabview_create(scr);
    lv_obj_t *tab_buttons;

    lv_obj_set_pos(obj, x, y);
    lv_tabview_set_tab_bar_size(obj, 32);
    tab_buttons = lv_tabview_get_tab_bar(obj);
    lv_obj_set_style_bg_color(tab_buttons, lv_palette_darken(LV_PALETTE_GREY, 3), 0);
    lv_obj_set_style_text_color(tab_buttons, lv_palette_lighten(LV_PALETTE_GREY, 5), 0);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    return obj;
}

lv_obj_t *add_tab(lv_obj_t *tab, const char *title)
{
    lv_obj_t *obj = lv_tabview_add_tab(tab, title);

    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    return obj;
}

lv_obj_t *add_section_label(lv_obj_t *tab, int32_t y, int32_t h, int32_t w0, const char *title)
{
    lv_obj_t *section = add_section(tab, 0, y, 320, h, w0, title);
    return add_label(lv_obj_get_child(section, 0), 8, 0);
}

lv_obj_t *add_section_label_xw(lv_obj_t *tab, int32_t x, int32_t y, int32_t w, int32_t h, int32_t w0, const char *title)
{
    lv_obj_t *section = add_section(tab, x, y, w, h, w0, title);
    return add_label(lv_obj_get_child(section, 0), 8, 0);
}

lv_obj_t *add_switch_label(lv_obj_t *tab, int32_t y, const char *title)
{
    add_label_text(tab, 0, y + 4, title, lv_color_black());
    return add_switch(tab, 260, y, 60, 30);
}

void add_2switch_label(lv_obj_t *tab, int32_t y, const char *title, lv_obj_t **sw1, lv_obj_t **sw2)
{
    add_label_text(tab, 0, y + 4, title, lv_color_black());
    *sw1 = add_switch(tab, 180, y, 60, 30);
    *sw2 = add_switch(tab, 260, y, 60, 30);
}

lv_obj_t *add_slider_label(lv_obj_t *tab, int32_t y, int32_t max, const char *title)
{
    add_label_text(tab, 0, y, title, lv_color_black());
    return add_slider(tab, 120, y, 180, 0, max);
}

lv_obj_t *add_page_air(ui_t *ui)
{
    lv_obj_t *tab = add_tab(ui->tbv_main, "Air");
    ui->lbl_bmx280lo = add_section_label(tab, 0, 42, 72, "BME280L");
    ui->lbl_bmx280hi = add_section_label(tab, 40, 42, 72, "BME280H");
    ui->lbl_scd4x = add_section_label(tab, 80, 42, 72, "SCD41");
    ui->lbl_mhz19 = add_section_label(tab, 120, 42, 72, "MHZ19");
    ui->lbl_yys = add_section_label(tab, 160, 42, 72, "YYS");
    return tab;
}

lv_obj_t *add_page_dust(ui_t *ui)
{
    lv_obj_t *tab = add_tab(ui->tbv_main, "Dust");
    ui->lbl_sps30_1 = add_section_label(tab, 0, 30, 72, "PM0.5");
    ui->lbl_sps30_2 = add_section_label(tab, 28, 30, 72, "PM1.0");
    ui->lbl_sps30_3 = add_section_label(tab, 56, 30, 72, "PM2.5");
    ui->lbl_sps30_4 = add_section_label(tab, 84, 30, 72, "PM4.0");
    ui->lbl_sps30_5 = add_section_label(tab, 112, 30, 72, "PM10.0");
    ui->lbl_sps30_6 = add_section_label(tab, 140, 30, 72, "TypPartSz");
    return tab;
}

lv_obj_t *add_page_gps(ui_t *ui)
{
    lv_obj_t *tab = add_tab(ui->tbv_main, "GPS");
    ui->lbl_gps_date = add_section_label_xw(tab, 0, 0, 180, 30, 64, "Date");
    ui->lbl_gps_time = add_section_label_xw(tab, 0, 28, 180, 30, 64, "Time");
    ui->lbl_gps_lat = add_section_label_xw(tab, 0, 56, 180, 30, 64, "Lat");
    ui->lbl_gps_status = add_section_label_xw(tab, 180, 0, 140, 30, 64, "Status");
    ui->lbl_gps_data_cnt = add_section_label_xw(tab, 180, 28, 140, 30, 64, "DatCnt");
    ui->lbl_gps_error_cnt = add_section_label_xw(tab, 180, 56, 140, 30, 64, "Error");
    ui->lbl_gps_lng = add_section_label(tab, 84, 30, 64, "Lng");
    ui->lbl_gps_alt = add_section_label(tab, 112, 30, 64, "Alt");
    ui->lbl_gps_speed = add_section_label(tab, 140, 30, 64, "Speed");
    ui->lbl_gps_sats = add_section_label(tab, 168, 30, 64, "Sats");
    ui->lbl_gps_pdop = add_section_label(tab, 196, 30, 64, "PDOP");
    ui->lbl_gps_hdop = add_section_label(tab, 224, 30, 64, "HDOP");
    ui->lbl_gps_vdop = add_section_label(tab, 252, 30, 64, "VDOP");
    return tab;
}

lv_obj_t *add_page_wifi(ui_t *ui)
{
    lv_obj_t *tab = add_tab(ui->tbv_main, LV_SYMBOL_WIFI);
    add_2switch_label(tab, 0, "WiFi Enable/Auto", &ui->sw_wifi_enable, &ui->sw_wifi_auto);
    add_2switch_label(tab, 32, "TCP Server Enable/Auto", &ui->sw_tcp_server_enable, &ui->sw_tcp_server_auto);
    add_2switch_label(tab, 64, "FTP Server Enable/Auto", &ui->sw_ftp_server_enable, &ui->sw_ftp_server_auto);
    ui->lbl_wifi_name = add_section_label(tab, 100, 30, 64, "Network");
    ui->lbl_wifi_ip = add_section_label_xw(tab, 0, 130, 170, 30, 64, "IP");
    ui->lbl_wifi_rssi = add_section_label_xw(tab, 170, 130, 150, 30, 64, "RSSI");
    ui->lbl_ftp_status = add_section_label(tab, 160, 30, 64, "FTP");
    ui->lbl_mqtt_status = add_section_label(tab, 190, 30, 64, "MQTT");
    ui->btn_wifi_scan = add_button(tab, 0, 220, 200, 0, "WiFi Scan");
    ui->lst_wifi = lv_list_create(tab);
    lv_obj_set_pos(ui->lst_wifi, 0, 260);
    lv_obj_set_size(ui->lst_wifi, 320, 120);
    return tab;
}

lv_obj_t *add_page_sd(ui_t *ui)
{
    lv_obj_t *tab = add_tab(ui->tbv_main, LV_SYMBOL_SD_CARD);
    ui->sw_record = add_switch_label(tab, 0, "Record");
    ui->sw_auto_record = add_switch_label(tab, 30, "Auto Record");
    ui->lbl_sd_card = add_section_label(tab, 60, 30, 64, "SD-Card");
    ui->lbl_sd_free = add_section_label(tab, 90, 30, 64, "Free");
    ui->lbl_sd_files = add_section_label(tab, 120, 30, 64, "Files");
    ui->lbl_sd_fill = add_section_label(tab, 150, 30, 64, "FillLvl");
    return tab;
}

lv_obj_t *add_page_cfg(ui_t *ui)
{
    lv_obj_t *tab = add_tab(ui->tbv_main, LV_SYMBOL_SETTINGS);

    ui->lbl_time = add_section_label(tab, 0, 30, 72, "Time");
    ui->lbl_savetime = add_section_label(tab, 30, 30, 72, "Save Time");
    ui->lbl_uptime = add_section_label(tab, 60, 30, 72, "Up Time");
    ui->lbl_counter = add_section_label(tab, 90, 30, 72, "Counter");
    ui->lbl_heap = add_section_label(tab, 120, 30, 72, "Heap");
    ui->lbl_qmc5883L = add_section_label(tab, 150, 30, 72, "QMC5883L");
    ui->lbl_adxl345 = add_section_label(tab, 180, 30, 72, "ADXL345");

    add_label_text(tab, 0, 210, "Power Mode", lv_color_black());
    ui->sl_lcd_pwr = add_slider_label(tab, 230, 2, "LCD");
    ui->sl_gps_pwr = add_slider_label(tab, 270, 3, "GPS");
    ui->sl_scd4x_pwr = add_slider_label(tab, 310, 3, "SCD4x");
    ui->sl_wifi_pwr = add_slider_label(tab, 350, 2, "WiFi");
    ui->sl_mode_pwr = add_slider_label(tab, 390, 4, "Mode");

    ui->btn_calibrate = add_button(tab, 0, 430, 200, 0, "Calibrate Sensors");
    ui->btn_save_config = add_button(tab, 0, 480, 200, 0, "Save Config");
    ui->sw_cfg_lock = add_switch_label(tab, 530, "Lock Config");
    return tab;
}

ui_t *ui_init(lv_display_t *disp)
{
    ESP_LOGI(TAG, "Initialize UI");

    lv_obj_t *scr = lv_display_get_screen_active(disp);
    ui_t *ui = pvPortMalloc(sizeof(ui_t));

    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return NULL;
    init_styles();
    ESP_LOGI(TAG, "Add Tab main");
    ui->tbv_main = add_tabiew(scr, 0, 0);
    ESP_LOGI(TAG, "Add Tab air");
    ui->tab_air = add_page_air(ui);
    ESP_LOGI(TAG, "Add Tab dust");
    ui->tab_dust = add_page_dust(ui);
    ESP_LOGI(TAG, "Add Tab gps");
    ui->tab_gps = add_page_gps(ui);
    ESP_LOGI(TAG, "Add Tab wifi");
    ui->tab_wifi = add_page_wifi(ui);
    ESP_LOGI(TAG, "Add Tab sd");
    ui->tab_sd = add_page_sd(ui);
    ESP_LOGI(TAG, "Add Tab cfg");
    ui->tab_cfg = add_page_cfg(ui);
    lvgl_port_unlock();
    ESP_LOGI(TAG, "DONE");
    return ui;
}
