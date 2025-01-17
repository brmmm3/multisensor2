/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#loader-with-arc

#include "core/lv_obj_event.h"
#include "esp_log.h"

#include "lvgl.h"
#include "misc/lv_area.h"
#include "misc/lv_event.h"

static const char *TAG = "LCD";

static lv_display_rotation_t rotation = LV_DISP_ROTATION_0;


static void btn_cb(lv_event_t *e)
{
    lv_display_t *disp = lv_event_get_user_data(e);
    lv_obj_t *btn = lv_event_get_current_target(e);

    ESP_LOGI(TAG, "Button pressed %p", btn);
    /*rotation++;
    if (rotation > LV_DISP_ROTATION_270) {
        rotation = LV_DISP_ROTATION_0;
    }
    lv_disp_set_rotation(disp, rotation);*/
}

lv_obj_t *create_button(lv_obj_t *scr, int32_t x, int32_t y, int32_t w, int32_t h, const char *text, void(*cb)(lv_event_t *e), void *data)
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

    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, data);
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

void ui_init(lv_display_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_t *btn1 = create_button(scr, 30, 30, 0, 0, "Button 1", btn_cb, disp);
    lv_obj_t *btn2 = create_button(scr, 130, 30, 0, 0, "Button 2", btn_cb, disp);

    create_animated_arc(scr);
}
