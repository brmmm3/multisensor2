#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hal/wdt_hal.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "lwip/dns.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "rom/gpio.h"
#include "hal/uart_types.h"
#include "driver/i2c_master.h"

#include "rtci2c/rtci2c.h"

#include "lvgl.h"

#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch_xpt2046.h"

#include "wifi.h"
#include "ftp.h"
#include "gps.h"
#include "lcd.h"
#include "bmx280.h"
#include "scd4x.h"
#include "mhz19.h"
#include "sps30.h"
#include "yys.h"

static const char *TAG = "MS2";

#define LED_PIN_NUM         GPIO_NUM_8
// I2C
#define I2C_PORT_AUTO       -1
#define I2C_PIN_NUM_SDA     GPIO_NUM_6
#define I2C_PIN_NUM_SCL     GPIO_NUM_7
// SPI
#define SPI_PIN_NUM_SCLK    GPIO_NUM_19
#define SPI_PIN_NUM_MOSI    GPIO_NUM_18
#define SPI_PIN_NUM_MISO    GPIO_NUM_20
// SD-Card (SPI-Mode)
#define SDCARD_PIN_NUM_CS   GPIO_NUM_23
// LCD
#define LCD_PIN_NUM_CS      GPIO_NUM_0
#define LCD_PIN_NUM_DC_RS   GPIO_NUM_1
#define LCD_PIN_NUM_RESET   GPIO_NUM_21
#define LCD_PIN_NUM_LED     GPIO_NUM_22
#define LCD_PIN_NUM_T_CS    GPIO_NUM_15
// GPS (HW-UART)
#define GPS_UART_NUM        UART_NUM_1
#define GPS_PIN_NUM_RX      GPIO_NUM_10
// MHZ19 (LP HW-UART)
#define MHZ19_UART_NUM      LP_UART_NUM_0
#define MHZ19_PIN_NUM_RX    GPIO_NUM_4
#define MHZ19_PIN_NUM_TX    GPIO_NUM_5
// YYS (SW-UART)
#define YYS_PIN_NUM_O2      GPIO_NUM_11
#define YYS_PIN_NUM_CO      GPIO_NUM_12
#define YYS_PIN_NUM_H2S     GPIO_NUM_13

#define MOUNT_POINT "/sdcard"

// RTC (DS1307)
#define DS1307_ADDRESS      0 /* let the library figure it out */

#define LCD_H_RES               240
#define LCD_V_RES               320

#define LVGL_DRAW_BUF_LINES     20 // number of display lines in each draw buffer
#define LVGL_TICK_PERIOD_MS     2
#define LVGL_TASK_MAX_DELAY_MS  500
#define LVGL_TASK_MIN_DELAY_MS  1
#define LVGL_TASK_STACK_SIZE    (4 * 1024)
#define LVGL_TASK_PRIORITY      2

// UART
#define UART_BUFFER_SIZE 256

#define CONFIG_NTP_SERVER   "pool.ntp.org"

// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;

extern void example_lvgl_demo_ui(lv_disp_t *disp);

rtci2c_context *rtc = NULL;


i2c_master_bus_handle_t i2c_bus_init(uint8_t sda_io, uint8_t scl_io)
{
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_AUTO,
        .scl_io_num = scl_io,
        .sda_io_num = sda_io,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    ESP_LOGI(TAG,"I2C master bus created");
    return bus_handle;
}

/*void scd4x_read_values(scd4x_t *scd4x)
{
    scd4x_sensors_values_t values;

    vTaskDelay(pdMS_TO_TICKS(10));
    if (scd4x_read_measurement(scd4x, &values) != ESP_OK) {
        ESP_LOGE(TAG, "Sensors read measurement error!");
    } else {
        float co2_level = values.co2;
        float temperature = values.temperature;
        float humidity = values.humidity;

        ESP_LOG_BUFFER_HEX_LEVEL(TAG, &values, sizeof(values), ESP_LOG_DEBUG);
        ESP_LOGI(TAG, "CO₂ %4.0f ppm - Temperature %2.1f °%c - Humidity %2.1f%%",
                    co2_level, temperature, scd4x->scale, humidity);
    }
}*/

void time_sync_notification_cb(struct timeval *tv)
{
	ESP_LOGI(TAG, "Notification of a time synchronization event");
    ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %jd sec: %li ms: %li us",
                        (intmax_t)tv->tv_sec,
                        tv->tv_usec / 1000,
                        tv->tv_usec % 1000);
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    now = tv->tv_sec;
    //time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "#The current date/time in Vienna is: %s", strftime_buf);
    if (rtc != NULL) {
        struct tm t;

        rtci2c_set_datetime(rtc, &timeinfo);
        if (!rtci2c_get_datetime(rtc, &t)) {
            ESP_LOGE(TAG, "Date/tate query failed");
        } else {
            ESP_LOGI(TAG, "RTC Current: %02u/%02u/%u %u:%02u:%02u",
                     t.tm_mday, t.tm_mon, t.tm_year, t.tm_hour, t.tm_min, t.tm_sec);
        }
    }
}

static void initialize_sntp(void)
{
	ESP_LOGI(TAG, "Initializing SNTP");
	esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
	//sntp_setservername(0, "pool.ntp.org");
	ESP_LOGI(TAG, "Your NTP Server is %s", CONFIG_NTP_SERVER);
	esp_sntp_setservername(0, CONFIG_NTP_SERVER);
	sntp_set_time_sync_notification_cb(time_sync_notification_cb);
	esp_sntp_init();
}

static esp_err_t obtain_time(void)
{
	initialize_sntp();
	// wait for time to be set
	int retry = 0;
	const int retry_count = 10;
	while (sntp_get_sync_status() != SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
		ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	if (retry == retry_count) return ESP_FAIL;
	return ESP_OK;
}

static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_write_file_bin(const char *path, uint8_t *data, uint32_t size)
{
    ESP_LOGI(TAG, "Write binary file %s", path);
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fwrite(data, size, 1, f);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[64];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}

void list_dir(char *path)
{
    struct dirent *dp;
    DIR *dir = opendir(path);
    char p[80];

    if (dir == NULL) {
        ESP_LOGE(TAG, "Can't Open Dir.");
        return;
    }
    int pos = strlen(MOUNT_POINT) + 1;
    strcpy(p, MOUNT_POINT);
    p[pos - 1] = '/';

    while ((dp = readdir(dir)) != NULL) {
        struct stat st;
        strcpy(&p[pos], dp->d_name);
        if (stat(p, &st) == 0) {
        }
        ESP_LOGI(TAG, "%s: %s %lu",
            (dp->d_type == DT_DIR)
                ? "DIR"
                : "FILE",
            dp->d_name, st.st_size);
    }
    closedir(dir);
}

int sd_card_init()
{
    esp_err_t err;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 8000;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_PIN_NUM_MOSI,
        .miso_io_num = SPI_PIN_NUM_MISO,
        .sclk_io_num = SPI_PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 38400,
    };

    err = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return -1;
    }

    /*spi_device_interface_config_t dev_cfg = {
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_TXBIT_LSBFIRST | SPI_DEVICE_3WIRE,
        .clock_speed_hz = 1000000,
        .mode = 0,
        .queue_size = 10,
    };

    ESP_LOGI(TAG, "Adding fake LCD SPI device");
    dev_cfg.spics_io_num = PIN_NUM_LCD_CS;
    spi_device_handle_t lcd_handle;
    lcd_handle = (spi_device_handle_t)malloc(sizeof(lcd_handle));
    ESP_ERROR_CHECK(spi_bus_add_device(host.slot, &dev_cfg, &lcd_handle));

    ESP_LOGI(TAG, "Adding fake LCD TOUCH device");
    dev_cfg.spics_io_num = PIN_NUM_TOUCH_CS;
    spi_device_handle_t touch_handle;
    touch_handle = (spi_device_handle_t)malloc(sizeof(touch_handle));
    ESP_ERROR_CHECK(spi_bus_add_device(host.slot, &dev_cfg, &touch_handle));*/

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();

    slot_config.gpio_cs = SDCARD_PIN_NUM_CS;
    slot_config.host_id = host.slot;  // Card handle

    ESP_LOGI(TAG, "Mounting filesystem");
    // IMPORTANT WORKAROUND: Comment out "SDMMC_INIT_STEP(is_spi, sdmmc_init_spi_crc);" in sdmmc_init.c
    err = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (err != ESP_OK) {
        if (err == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(err));
        }
        return -1;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    ESP_LOGI(TAG, "Get FAT Info");
    uint64_t bytes_total, bytes_free;
    esp_vfs_fat_info(MOUNT_POINT, &bytes_total, &bytes_free);
    ESP_LOGI(TAG, "FAT FS: %" PRIu64 " kB total, %" PRIu64 " kB free", bytes_total / 1024, bytes_free / 1024);

    // First create a file.
    const char *file_hello = MOUNT_POINT"/hello.txt";
    char data[64];
    snprintf(data, 64, "%s %d %d %s!\n", "Hello", card->cid.mfg_id, card->cid.oem_id, card->cid.name);
    err = s_example_write_file(file_hello, data);
    if (err != ESP_OK) {
        return -1;
    }

    const char *file_foo = MOUNT_POINT"/foo.txt";

    // Check if destination file exists before renaming
    struct stat st;
    if (stat(file_foo, &st) == 0) {
        // Delete it if it exists
        unlink(file_foo);
    }

    // Rename original file
    ESP_LOGI(TAG, "Renaming file %s to %s", file_hello, file_foo);
    if (rename(file_hello, file_foo) != 0) {
        ESP_LOGE(TAG, "Rename failed");
        return -1;
    }

    err = s_example_read_file(file_foo);
    if (err != ESP_OK) {
        return -1;
    }

    const char *file_hello_bin = MOUNT_POINT"/hello.bin";
    snprintf(data, 64, "%s %d %d %s!\n", "Hello", card->cid.mfg_id, card->cid.oem_id, card->cid.name);
    err = s_example_write_file_bin(file_hello_bin, (uint8_t*)data, 64);
    if (err != ESP_OK) {
        return -1;
    }

    list_dir(MOUNT_POINT);
    return host.slot;
}

void rtc_init(i2c_master_bus_handle_t *bus_handle)
{
    i2c_lowlevel_config config = {
        .bus = bus_handle
    };

    rtc = rtci2c_init(RTCI2C_DEVICE_DS1307, DS1307_ADDRESS, &config);
    if (rtc == NULL) {
        ESP_LOGE(TAG, "RTC Initialization failed");
    } else {
        struct tm t;

        if (!rtci2c_get_datetime(rtc, &t)) {
            ESP_LOGE(TAG, "Date/tate query failed");
        } else {
            ESP_LOGI(TAG, "RTC Current: %02u/%02u/%u %u:%02u:%02u",
                     t.tm_mday, t.tm_mon, t.tm_year, t.tm_hour, t.tm_min, t.tm_sec);
        }
    }
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void lvgl_port_update_callback(lv_display_t *disp)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);

    switch (rotation) {
    case LV_DISPLAY_ROTATION_0:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISPLAY_ROTATION_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISPLAY_ROTATION_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISPLAY_ROTATION_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    lvgl_port_update_callback(disp);
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // because SPI LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void lvgl_touch_cb(lv_indev_t * indev, lv_indev_data_t * data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    esp_lcd_touch_handle_t touch_pad = lv_indev_get_user_data(indev);
    esp_lcd_touch_read_data(touch_pad);
    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_pad, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    uint32_t time_threshold_ms = 1000 / CONFIG_FREERTOS_HZ;
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, time_threshold_ms);
        usleep(1000 * time_till_next_ms);
    }
}

void lcd_init2(int spi_host_id)
{
    ESP_LOGI(TAG, "Initialize SPI bus");
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_PIN_NUM_LED
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PIN_NUM_DC_RS,
        .cs_gpio_num = LCD_PIN_NUM_CS,
        .pclk_hz = 8000000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)spi_host_id, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_RESET,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(LCD_PIN_NUM_LED, 1);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    // create a lvgl display
    lv_display_t *display = lv_display_create(LCD_H_RES, LCD_V_RES);

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    size_t draw_buffer_sz = LCD_H_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);

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
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    /* Register done callback */
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));

    // Configure touch
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(LCD_PIN_NUM_T_CS);
    // Attach the TOUCH to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)spi_host_id, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_touch_handle_t tp = NULL;

    ESP_LOGI(TAG, "Initialize touch controller XPT2046");
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp));

    static lv_indev_t *indev;
    indev = lv_indev_create();  // Input device driver (Touch)
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, display);
    lv_indev_set_user_data(indev, tp);
    lv_indev_set_read_cb(indev, lvgl_touch_cb);

    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    _lock_acquire(&lvgl_api_lock);
    example_lvgl_demo_ui(display);
    _lock_release(&lvgl_api_lock);
}

void app_main(void)
{
    esp_err_t err;
    mhz19_t *mhz19 = NULL;

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    i2c_master_bus_handle_t bus_handle = i2c_bus_init(I2C_PIN_NUM_SDA, I2C_PIN_NUM_SCL);

    //esp_lcd_panel_handle_t panel_handle = lcd_init(bus_handle);
    //lcd_print(panel_handle, 0, 0, "Hello World");

    //bmx280_t *bmx280 = NULL;
    //ESP_ERROR_CHECK(bmx280_device_init(&bmx280, bus_handle));
    //ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_CYCLE));
 
    //scd4x_t *scd4x = NULL;
    //scd4x_device_init(&scd4x, bus_handle);
 
    /*err = mhz19_init(&mhz19, MHZ19_UART_NUM, MHZ19_PIN_NUM_RX, MHZ19_PIN_NUM_TX);
    ESP_LOGI(TAG, "mhz19_init=%d", err);

    sps30_t *sps30 = sps30_init(bus_handle);
    if (sps30 != NULL) {
        ESP_LOGI("SPS30", "DevInfo = %s", sps30->device_info);
        ESP_LOGI("SPS30", "Serial = %s", sps30->serial);
        ESP_LOGI("SPS30", "Firmware Version = %d.%d", sps30->firmware_version >> 8, sps30->firmware_version & 0xff);
        if (strncmp(sps30->device_info, "00080000", 8)) {
            ESP_LOGE("SPS30", "returned invalid DevInfo");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize SPS30 sensor");
    }*/

    //gps_sensor_t gps_sensor = gps_init();

    //yys_sensors_t yys_sensors = yys_init(YYS_PIN_NUM_O2, YYS_PIN_NUM_CO, YYS_PIN_NUM_H2S);

    // RTC
    //rtc_init(&bus_handle);

    // WiFi
    /*esp_netif_ip_info_t ip_info;

    ESP_ERROR_CHECK(wifi_init(&ip_info));
    if (ip_info.ip.addr != 0) {
        // Austria/Vienna: CET-1CEST,M3.5.0,M10.5.0/3
        setenv("TZ","CET-1CEST,M3.5.0,M10.5.0/3",1);
        tzset();
        ESP_ERROR_CHECK(obtain_time());
    }*/

    // SD-Card (SPI Mode)
    int spi_host_id = sd_card_init();
    lcd_init2(spi_host_id);

    wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();

    ESP_LOGI(TAG, "Start main loop.");

    uint16_t v16;
    //sps30_values_t *values = &sps30->values;

    while (1) {
        /*bmx280_readout(bmx280);
        lcd_print(panel_handle, 0, 1, "T=%.2f °C", bmx280->values.temperature);
        lcd_print(panel_handle, 0, 2, "H=%.2f %%", bmx280->values.humidity);
        lcd_print(panel_handle, 0, 3, "P=%.2f hPa", bmx280->values.pressure);
        lcd_print(panel_handle, 0, 4, "A=%.2f m", bmx280->values.altitude);*/

        /*if (xQueueReceive(gps_sensor.queue, &v16, 1))
        {
            ESP_LOGI(TAG, "GPS %04X", v16);
        }*/

        /*if (scd4x_get_data_ready_status(scd4x)) {
            ESP_LOGI("SCD", "READY");
            scd4x_sensors_values_t sv = {
                .co2 = 0,
                .temperature = 0,
                .humidity = 0
            };
            err = scd4x_read_measurement(scd4x, &sv);
            ESP_LOGI("SCD", "VAL %d: CO2=%d ppm  HUM=%f %%  Temp=%f", err, sv.co2, sv.humidity, sv.temperature);
            err = scd4x_start_periodic_measurement(scd4x);
            ESP_LOGI("SCD", "START_MEAS %d", err);
        }*/

        /*ESP_LOGI("YYS", "O2=%d  CO=%d  H2S=%d", yys_get_o2(yys_sensors), yys_get_co(yys_sensors), yys_get_h2s(yys_sensors));
        ESP_LOGI("MHZ19", "CO2=%d ppm  Temp=%d", mhz19->co2, mhz19->temp);
        bool rdy = sps30_read_data_ready(sps30);
        if ((err = sps30_read_device_status_register(sps30)) == ESP_OK) {
            if (sps30->status != 0) {
                ESP_LOGW("SPS30", "STATUS=%08X", (unsigned int)sps30->status);
            }
        } else {
            ESP_LOGE("SPS30", "Failed to read status");
        }
        if (rdy) {
            if ((err = sps30_read_measurement(sps30)) == ESP_OK) {
                ESP_LOGI("SPS30", "PM0.5 =%f #/cm3", values->nc_0p5);
                ESP_LOGI("SPS30", "PM1.0 =%f ug/cm3 P1.0 =%f #/cm3", values->mc_1p0, values->nc_1p0);
                ESP_LOGI("SPS30", "PM2.5 =%f ug/cm3 P2.5 =%f #/cm3", values->mc_2p5, values->nc_2p5);
                ESP_LOGI("SPS30", "PM4.0 =%f ug/cm3 P4.0 =%f #/cm3", values->mc_4p0, values->nc_4p0);
                ESP_LOGI("SPS30", "PM10.0=%f ug/cm3 P10.0=%f #/cm3", values->mc_10p0, values->nc_10p0);
                ESP_LOGI("SPS30", "TypPartSz=%f um", values->typical_particle_size);
            } else {
                ESP_LOGE("SPS30", "Failed to read measurement values with error %d", err);
            }
        } else {
            ESP_LOGE("SPS30", "not ready");
        }*/

        wdt_hal_write_protect_disable(&rtc_wdt_ctx);
        wdt_hal_feed(&rtc_wdt_ctx);
        wdt_hal_write_protect_enable(&rtc_wdt_ctx);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
