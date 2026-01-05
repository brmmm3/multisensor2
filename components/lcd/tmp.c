#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_touch.h"
#include "driver/gpio.h"

#define LCD_H_RES          320
#define LCD_V_RES          240
#define LCD_PCLK_HZ        (20 * 1000 * 1000)  // Start with 20 MHz, increase to 40 MHz if stable
#define LVGL_DRAW_BUF_LINES 40                 // ~1/6 of height for good performance/RAM balance

static const char *TAG = "lcd_init";

lv_display_t *lcd_init(int spi_host_id, uint8_t cs_pin, uint8_t dc_pin, uint8_t reset_pin,
                       uint8_t led_pin, uint8_t t_cs_pin)
{
    /* Backlight control (active high assumed) */
    ESP_LOGI(TAG, "Configure backlight GPIO");
    gpio_config_t bk_gpio_config = {
        .pin_bit_mask = 1ULL << led_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(led_pin, 0);  // Off initially

    /* Initialize LVGL port first (handles lv_init(), tick timer, task lock, etc.) */
    ESP_LOGI(TAG, "Initialize LVGL port");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    /* LCD Panel IO (SPI) */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = cs_pin,
        .dc_gpio_num = dc_pin,
        .spi_mode = 0,
        .pclk_hz = LCD_PCLK_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = {
            .dc_low_on_data = false,
            .octal_mode = false,
            .lsb_first = false,
        },
    };
    ESP_LOGI(TAG, "Install LCD panel IO");
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)spi_host_id, &io_config, &io_handle));

    /* LCD Panel driver (ILI9341) */
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = reset_pin,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    /* Add display to LVGL using esp_lvgl_port */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .buffer_size = LCD_V_RES * LVGL_DRAW_BUF_LINES,  // In pixels
        .double_buffer = true,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = true,      // Essential for good SPI performance
            .buff_spiram = false,  // Set true if PSRAM available and need larger buffers
            .full_refresh = false,
        },
    };

    ESP_LOGI(TAG, "Add display to LVGL");
    lv_display_t *display = lvgl_port_add_disp(&disp_cfg);

    /* Rotate to 270° (common for many 320x240 modules in portrait) - adjust as needed */
    lv_display_set_rotation(display, LV_DISPLAY_ROTATION_270);

    /* Touch controller (XPT2046) - shares the same SPI bus */
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(t_cs_pin);

    ESP_LOGI(TAG, "Install touch panel IO");
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)spi_host_id, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = false,
            .mirror_x = true,   // Common calibration - adjust based on your module
            .mirror_y = false,
        },
    };

    esp_lcd_touch_handle_t tp = NULL;
    ESP_LOGI(TAG, "Install XPT2046 touch driver");
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp));

    /* Add touch to LVGL */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = display,
        .handle = tp,
    };
    ESP_LOGI(TAG, "Add touch input to LVGL");
    lv_indev_t *touch_indev = lvgl_port_add_touch(&touch_cfg);

    /* Turn on backlight */
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(led_pin, 1);

    ESP_LOGI(TAG, "LCD + Touch + LVGL initialization complete");
    return display;
}