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

#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "rom/gpio.h"
#include "hal/uart_types.h"
#include "driver/i2c_master.h"

#include "rtci2c/rtci2c.h"

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

// I2C
#define I2C_PORT_AUTO       -1
#define I2C_PIN_NUM_SDA     GPIO_NUM_6
#define I2C_PIN_NUM_SCL     GPIO_NUM_7
// MHZ19
#define MHZ19_UART_NUM      LP_UART_NUM_0
#define MHZ19_PIN_NUM_RX    GPIO_NUM_4
#define MHZ19_PIN_NUM_TX    GPIO_NUM_5
// SD-Card (SPI-Mode)
#define SDCARD_PIN_NUM_CS   GPIO_NUM_23
#define SDCARD_PIN_NUM_SCLK GPIO_NUM_19
#define SDCARD_PIN_NUM_MOSI GPIO_NUM_18
#define SDCARD_PIN_NUM_MISO GPIO_NUM_20

#define MOUNT_POINT "/sdcard"

// RTC (DS1307)
#define DS1307_ADDRESS   0 /* let the library figure it out */

// UART
#define UART_BUFFER_SIZE 256

#define LED_PIN_NUM     GPIO_NUM_8

#define CONFIG_NTP_SERVER   "pool.ntp.org"


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

void sd_card_init()
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
    host.max_freq_khz = 1000;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SDCARD_PIN_NUM_MOSI,
        .miso_io_num = SDCARD_PIN_NUM_MISO,
        .sclk_io_num = SDCARD_PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    err = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
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
        return;
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
        return;
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
        return;
    }

    err = s_example_read_file(file_foo);
    if (err != ESP_OK) {
        return;
    }

    const char *file_hello_bin = MOUNT_POINT"/hello.bin";
    snprintf(data, 64, "%s %d %d %s!\n", "Hello", card->cid.mfg_id, card->cid.oem_id, card->cid.name);
    err = s_example_write_file_bin(file_hello_bin, (uint8_t*)data, 64);
    if (err != ESP_OK) {
        return;
    }

    list_dir(MOUNT_POINT);
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
 
    err = mhz19_init(&mhz19, MHZ19_UART_NUM, MHZ19_PIN_NUM_RX, MHZ19_PIN_NUM_TX);
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
    }

    //gps_sensor_t gps_sensor = gps_init();

    //yys_sensors_t yys_sensors = yys_init();

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
    //sd_card_init();

    wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();

    ESP_LOGI(TAG, "Start main loop.");

    uint16_t v16;
    sps30_values_t *values = &sps30->values;

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

        /*if (xQueueReceive(yys_sensors.o2_sensor->queue, &v16, 1))
        {
            lcd_print(panel_handle, 0, 5, "O2=%04X", v16);
            ESP_LOGI(TAG, "O2 %04X", v16);
        }

        if (xQueueReceive(yys_sensors.co_sensor->queue, &v16, 1))
        {
            lcd_print(panel_handle, 0, 6, "CO=%04X", v16);
            ESP_LOGI(TAG, "CO %04X", v16);
        }

        if (xQueueReceive(yys_sensors.h2s_sensor->queue, &v16, 1))
        {
            lcd_print(panel_handle, 0, 7, "H2S=%04X", v16);
            ESP_LOGI(TAG, "H2S %04X", v16);
        }*/
        //read_scd4x_values();

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
        }

        wdt_hal_write_protect_disable(&rtc_wdt_ctx);
        wdt_hal_feed(&rtc_wdt_ctx);
        wdt_hal_write_protect_enable(&rtc_wdt_ctx);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
