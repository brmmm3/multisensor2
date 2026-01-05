#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_lcd_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

#include "driver/gpio.h"
#include "hal/gpio_types.h"

#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch_xpt2046.h"
#include "esp_lvgl_port.h"
#include "lcd.h"

static const char *TAG = "LCD";

#define LCD_H_RES               240
#define LCD_V_RES               320
#define LCD_PCLK_HZ             8000000  /* 8MHz */

#define LCD_TOUCH_X_MIN        25
#define LCD_TOUCH_X_MAX        224
#define LCD_TOUCH_Y_MIN        20
#define LCD_TOUCH_Y_MAX        295

#define LVGL_DRAW_BUF_LINES     24       /* number of display lines in each draw buffer */
#define LVGL_TICK_PERIOD_MS     2
#define LVGL_TASK_MAX_DELAY_MS  500
#define LVGL_TASK_MIN_DELAY_MS  1
#define LVGL_TASK_STACK_SIZE    8192
#define LVGL_TASK_PRIORITY      2

gpio_config_t bk_gpio_config;
uint8_t bk_led_pin = 0;


typedef struct {
    esp_lcd_touch_handle_t  handle;     /* LCD touch IO handle */
    lv_indev_t              *indev;     /* LVGL input device driver */
    struct {
        float x;
        float y;
    } scale;                            /* Touch scale */
} lvgl_port_touch_ctx_t;

static void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    assert(indev);
    lvgl_port_touch_ctx_t *touch_ctx = (lvgl_port_touch_ctx_t *)lv_indev_get_driver_data(indev);
    assert(touch_ctx);
    assert(touch_ctx->handle);

    uint8_t touch_cnt = 0;
    esp_lcd_touch_point_data_t touch_data[CONFIG_ESP_LCD_TOUCH_MAX_POINTS] = {0};

    /* Read data from touch controller into memory */
    ESP_ERROR_CHECK(esp_lcd_touch_read_data(touch_ctx->handle));

    /* Read data from touch controller */
    ESP_ERROR_CHECK(esp_lcd_touch_get_data(touch_ctx->handle, touch_data, &touch_cnt, CONFIG_ESP_LCD_TOUCH_MAX_POINTS));
    if (touch_cnt > 0) {
        // Calibrate Touchscreen points with map function to the correct width and height
        int32_t x = touch_data[0].x; //25, 223, 1, LCD_H_RES);
        int32_t y = touch_data[0].y; //20, 285, 1, LCD_V_RES);
        data->point.x = x < LCD_TOUCH_X_MIN ? 0 : (x - LCD_TOUCH_X_MIN) * LCD_H_RES / (LCD_TOUCH_X_MAX - LCD_TOUCH_X_MIN);
        data->point.y = y < LCD_TOUCH_Y_MIN ? 0 : (y - LCD_TOUCH_Y_MIN) * LCD_V_RES / (LCD_TOUCH_Y_MAX - LCD_TOUCH_Y_MIN);
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void lcd_set_bg_pwr(uint8_t mode)
{
    gpio_config_t config = {
        .mode = mode < 3 ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT,
        .pull_up_en = mode > 0 && mode < 3 ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = mode == 1 ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = bk_gpio_config.pin_bit_mask,
    };

    ESP_LOGI(TAG, "lcd_set_bg_pwr %d", mode);
    gpio_config(&config);
    gpio_set_level(bk_led_pin, mode > 2);
}

lv_display_t *lcd_init(int spi_host_id, uint8_t cs_pin, uint8_t dc_pin, uint8_t reset_pin, uint8_t led_pin, uint8_t t_cs_pin)
{
    bk_led_pin = led_pin;

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
    // Rotate LCD display by 90°
    //ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    //ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    //ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    /* Add display to LVGL using esp_lvgl_port */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .rotation.mirror_x = true,
        .buffer_size = LCD_V_RES * LVGL_DRAW_BUF_LINES,  // In pixels
        .double_buffer = true,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = true,      // Essential for good SPI performance
            .buff_spiram = false,  // Set true if PSRAM available and need larger buffers
            .full_refresh = false,
            .swap_bytes = true,
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
    ESP_LOGI(TAG, "touch_indev=%p", touch_indev);
    lv_indev_set_read_cb(touch_indev, lvgl_touch_cb);

    /* Turn on backlight */
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(led_pin, 1);

    ESP_LOGI(TAG, "LCD + Touch + LVGL initialization complete");

    /*bk_gpio_config.mode = GPIO_MODE_OUTPUT;
    bk_gpio_config.pin_bit_mask = 1ULL << led_pin;
    bk_led_pin = led_pin;

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    esp_err_t err = lvgl_port_init(&lvgl_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LVGL port init failed: %s", esp_err_to_name(err));
        return NULL;
    }

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = dc_pin,
        .cs_gpio_num = cs_pin,
        .pclk_hz = LCD_PCLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = reset_pin,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    lv_display_t *display;
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    size_t draw_buffer_sz = LCD_V_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    // Configure touch
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(t_cs_pin);
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 1,
            .mirror_y = 0,
        },
    };
    esp_lcd_touch_handle_t tp = NULL;
    static lv_indev_t *indev;

    ESP_LOGI(TAG, "Turn off LCD backlight");
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    ESP_LOGI(TAG, "Install panel IO");
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)spi_host_id, &io_config, &io_handle));
    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    // Rotate LCD display by 90°
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(led_pin, 1);
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // create a lvgl display
    display = lv_display_create(LCD_H_RES, LCD_V_RES);
    void *buf1 = spi_bus_dma_memory_alloc(spi_host_id, draw_buffer_sz, 0);
    assert(buf1);
    void *buf2 = spi_bus_dma_memory_alloc(spi_host_id, draw_buffer_sz, 0);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // associate the mipi panel handle to the display
    lv_display_set_user_data(display, panel_handle);
    // set color depth
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));
    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    // Register done callback
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));
    // Attach the TOUCH to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)spi_host_id, &tp_io_config, &tp_io_handle));
    ESP_LOGI(TAG, "Initialize touch controller XPT2046");
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp));
    indev = lv_indev_create();  // Input device driver (Touch)
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, display);
    lv_indev_set_user_data(indev, tp);
    lv_indev_set_read_cb(indev, lvgl_touch_cb);
    lv_disp_set_rotation(display, LV_DISPLAY_ROTATION_270);*/
    return display;
}
