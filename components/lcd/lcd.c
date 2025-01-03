#include "esp_log.h"

#include "include/lcd.h"

#include "font8x8.h"

static const char *LCD_TAG = "LCD";

esp_lcd_panel_handle_t lcd_init(i2c_master_bus_handle_t bus_handle)
{
    ESP_LOGI(LCD_TAG, "Install LCD IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_IO_I2C_SH1107_CONFIG();
    io_config.scl_speed_hz = CONFIG_LCD_I2C_CLK_SPEED_HZ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(bus_handle, &io_config, &io_handle));

    ESP_LOGI(LCD_TAG, "Install LCD driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
        .vendor_config = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));

    ESP_LOGI(LCD_TAG, "Initialize LCD panel_handle=%p", panel_handle);
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    ESP_ERROR_CHECK(lcd_clear(panel_handle));
    ESP_LOGI(LCD_TAG, "LCD initialized");
    return panel_handle;
}

esp_err_t lcd_clear(esp_lcd_panel_handle_t panel_handle)
{
    char space[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    for (int x = 0; x < 128; x += 8) {
        for (int y = 0; y < 128; y += 8) {
            ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + 8, y + 8, space));
        }
    }
    return ESP_OK;
}

int lcd_print(esp_lcd_panel_handle_t panel_handle, uint8_t x, uint8_t y, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    char buffer[256];
    snprintf(buffer, sizeof(buffer), fmt, args);
    x <<= 3;
    y <<= 3;
    y = (y + 96) % 128;
    char *p = &buffer[0];
    while (1) {
        char c = *p;
        if (c == 0) break;
        p++;
        if (c == 13) {
            x = 0;
            continue;
        }
        if (c == 13) {
            y += 8;
            continue;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + 8, y + 8, &font8x8[(uint8_t)c]));
        x = (x + 8) % 128;
    }
    return ESP_OK;
}
