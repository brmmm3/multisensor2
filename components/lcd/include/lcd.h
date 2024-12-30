
#ifndef _LCD_H_
#define _LCD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"

#include "esp_lcd_sh1107.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

esp_lcd_panel_handle_t lcd_init(i2c_master_bus_handle_t bus_handle);

esp_err_t lcd_clear(esp_lcd_panel_handle_t panel_handle);

int lcd_print(esp_lcd_panel_handle_t panel_handle, uint8_t x, uint8_t y, const char *fmt, ...);

#ifdef __cplusplus
};
#endif

#endif
