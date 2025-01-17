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
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "led_strip.h"

#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "rom/gpio.h"
#include "hal/uart_types.h"
#include "driver/i2c_master.h"

#include "wifi.h"
#include "rtc_tiny.h"
#include "ftp.h"
#include "gps.h"
#include "lcd.h"
#include "ui.h"
#include "sdcard.h"
#include "bmx280.h"
#include "scd4x.h"
#include "mhz19.h"
#include "sps30.h"
#include "yys.h"

static const char *TAG = "MS2";

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

#define LED_PIN_NUM         GPIO_NUM_8
// I2C
#define I2C_PORT_AUTO       -1
#define I2C_PIN_NUM_SDA     GPIO_NUM_6
#define I2C_PIN_NUM_SCL     GPIO_NUM_7
// SPI
#define SPI_PIN_NUM_SCLK    GPIO_NUM_20
#define SPI_PIN_NUM_MOSI    GPIO_NUM_18  /* Master Output */
#define SPI_PIN_NUM_MISO    GPIO_NUM_19  /* Master Input */
// SD-Card (SPI-Mode)
#define SDCARD_PIN_NUM_CS   GPIO_NUM_23
// LCD
#define LCD_PIN_NUM_CS      GPIO_NUM_0
#define LCD_PIN_NUM_DC      GPIO_NUM_1
#define LCD_PIN_NUM_RST     GPIO_NUM_21
#define LCD_PIN_NUM_LED     GPIO_NUM_22
#define LCD_PIN_NUM_T_CS    GPIO_NUM_15
// GPS (HW-UART)
#define GPS_UART_NUM        UART_NUM_1
#define GPS_PIN_NUM_RX      GPIO_NUM_10
#define GPS_PIN_NUM_TX      GPIO_NUM_2
// MHZ19 (LP HW-UART)
#define MHZ19_UART_NUM      LP_UART_NUM_0
#define MHZ19_PIN_NUM_RX    GPIO_NUM_4
#define MHZ19_PIN_NUM_TX    GPIO_NUM_5
// YYS (SW-UART)
#define YYS_PIN_NUM_H2S     GPIO_NUM_11
#define YYS_PIN_NUM_O2      GPIO_NUM_12
#define YYS_PIN_NUM_CO      GPIO_NUM_13

// Unused GPIOs: 3, 9

// Special GPIO functions:
//  0 = Boot Mode (H = Normal Boot, internal Pull-Up)
// 15 = JTAG Signal Source Control

// UART
#define UART_BUFFER_SIZE 256

#define CONFIG_NTP_SERVER   "pool.ntp.org"

led_strip_handle_t led_strip;


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

void led_init(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        // Set the GPIO that the LED is connected
        .strip_gpio_num = LED_PIN_NUM,
        // Set the number of connected LEDs in the strip
        .max_leds = 1,
        // Set the pixel format of your LED strip
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        // LED strip model
        .led_model = LED_MODEL_WS2812,
        // In some cases, the logic is inverted
        .flags.invert_out = false,
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        // Set the clock source
        .clk_src = RMT_CLK_SRC_DEFAULT,
        // Set the RMT counter clock
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        // Set the DMA feature (not supported on the ESP32-C6)
        .flags.with_dma = false,
    };

    // LED Strip object handle
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
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

void app_main(void)
{
    esp_err_t err;
    gps_sensor_t *gps_sensor = NULL;
    bmx280_t *bmx280 = NULL;
    mhz19_t *mhz19 = NULL;
    scd4x_t *scd4x = NULL;
    sps30_t *sps30 = NULL;
    yys_sensors_t *yys_sensors = NULL;
    wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();

    /*gpio_config_t config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << SDCARD_PIN_NUM_CS | 1ULL << LCD_PIN_NUM_CS | 1ULL << LCD_PIN_NUM_T_CS | 1ULL << LCD_PIN_NUM_DC | 1ULL << LCD_PIN_NUM_RST | 1ULL << LCD_PIN_NUM_LED | 1ULL << SPI_PIN_NUM_MOSI,
    };
    gpio_config(&config);

    uint8_t level = 0;

    while (1) {
        gpio_set_level(SDCARD_PIN_NUM_CS, level);
        gpio_set_level(LCD_PIN_NUM_CS, level);
        gpio_set_level(LCD_PIN_NUM_T_CS, level);
        gpio_set_level(LCD_PIN_NUM_DC, level);
        gpio_set_level(LCD_PIN_NUM_RST, level);
        gpio_set_level(LCD_PIN_NUM_LED, level);
        gpio_set_level(SPI_PIN_NUM_MOSI, level);

        wdt_hal_write_protect_disable(&rtc_wdt_ctx);
        wdt_hal_feed(&rtc_wdt_ctx);
        wdt_hal_write_protect_enable(&rtc_wdt_ctx);

        vTaskDelay(10 / portTICK_PERIOD_MS);
        level = 1 - level;
    }*/

    led_init();

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    i2c_master_bus_handle_t bus_handle = i2c_bus_init(I2C_PIN_NUM_SDA, I2C_PIN_NUM_SCL);

    ESP_ERROR_CHECK(gps_init(&gps_sensor, GPS_UART_NUM, GPS_PIN_NUM_RX, GPS_PIN_NUM_TX));

    ESP_ERROR_CHECK(bmx280_device_init(&bmx280, bus_handle));
 
    ESP_ERROR_CHECK(mhz19_init(&mhz19, MHZ19_UART_NUM, MHZ19_PIN_NUM_RX, MHZ19_PIN_NUM_TX));
    ESP_ERROR_CHECK(scd4x_device_init(&scd4x, bus_handle));
 
    ESP_ERROR_CHECK(sps30_init(&sps30, bus_handle));
    ESP_LOGI("SPS30", "DevInfo = %s", sps30->device_info);
    ESP_LOGI("SPS30", "Serial = %s", sps30->serial);
    ESP_LOGI("SPS30", "Firmware Version = %d.%d", sps30->firmware_version >> 8, sps30->firmware_version & 0xff);
    if (strncmp(sps30->device_info, "00080000", 8)) {
        ESP_LOGE("SPS30", "returned invalid DevInfo");
    }

    ESP_ERROR_CHECK(yys_init(&yys_sensors, YYS_PIN_NUM_O2, YYS_PIN_NUM_CO, YYS_PIN_NUM_H2S));

    // RTC
    rtc_init(&bus_handle);

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
    int spi_host_id = sd_card_init(SDCARD_PIN_NUM_CS, SPI_PIN_NUM_SCLK, SPI_PIN_NUM_MOSI, SPI_PIN_NUM_MISO);

    lv_display_t *lcd = lcd_init(spi_host_id, LCD_PIN_NUM_CS, LCD_PIN_NUM_DC, LCD_PIN_NUM_RST, LCD_PIN_NUM_LED, LCD_PIN_NUM_T_CS);

    ui_init(lcd);

    ESP_LOGI(TAG, "Start main loop.");

    uint16_t v16;
    sps30_values_t *values = &sps30->values;

    while (true) {
        bmx280_readout(bmx280);
        ESP_LOGI("BMX280", "T=%.2f °C  H=%.2f %%  P=%.2f hPa  A=%.2f m", bmx280->values.temperature, bmx280->values.humidity, bmx280->values.pressure, bmx280->values.altitude);

        if (xQueueReceive(gps_sensor->queue, &v16, 1))
        {
            ESP_LOGI(TAG, "GPS %04X", v16);
        }

        if (scd4x_get_data_ready_status(scd4x)) {
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
        } else {
            err = scd4x_start_periodic_measurement(scd4x);
            ESP_LOGI("SCD", "NOT READY. START_MEAS %d", err);
        }

        ESP_LOGI("YYS", "O2=%f %%  CO=%f ppm  H2S=%f ppm", yys_get_o2(yys_sensors), yys_get_co(yys_sensors), yys_get_h2s(yys_sensors));
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
