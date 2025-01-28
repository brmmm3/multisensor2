#include "bmx280.h"
#include "config.h"
#include "gps.h"
#include "hal/gpio_types.h"
#include "lcd.h"
#include "main.h"
#include "scd4x.h"
#include "wifi.h"

#include "ui/ui.h"
#include "console/console.h"
#include <stdlib.h>

static const char *TAG = "MS2";

// I2C Address assignment:
//  (0x0D)  QMC5883L not used
//  0x1F    TLV493D
//  (0x3C)  HMC5883L (write) not used
//  (0x3D)  HMC5883L (read) not used
//  0x50    ?
//  0x53    ADXL345
//  (0x5E)  (TLV493D) not used
//  0x62    SCD4x (SCD41)
//  0x68    RTC Tiny
//  0x69    SPS30
//  0x76    BMx280 (BME280 beside SPS30)
//  0x77    Second BMx280 (BME280 beside MHZ19)

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

config_t *config = NULL;

rtc_t *rtc = NULL;
gps_sensor_t *gps = NULL;
gps_status_t *gps_status = NULL;
bmx280_t *bmx280lo = NULL;
bmx280_t *bmx280hi = NULL;
adxl345_t *adxl345 = NULL;
mhz19_t *mhz19 = NULL;
scd4x_t *scd4x = NULL;
sps30_t *sps30 = NULL;
qmc5883l_t *qmc5883l = NULL;
yys_sensors_t *yys_sensors = NULL;
wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
i2c_master_bus_handle_t bus_handle;
int spi_host_id;
lv_display_t *lcd;
ui_t *ui;
led_strip_handle_t led_strip;

bool gps_update = false;
bool sps30_update = false;
bool bmx280lo_update = false;
bool bmx280hi_update = false;
bool scd4x_update = false;
bool mhz19_update = false;
bool yys_update = false;
bool qmc5883l_update = false;
bool adxl345_update = false;


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

void dump_data()
{
    gps_dump(gps);
    bmx280_dump(bmx280lo);
    bmx280_dump(bmx280hi);
    scd4x_dump(scd4x);
    mhz19_dump(mhz19);
    yys_dump(yys_sensors);
    sps30_dump(sps30);
    qmc5883l_dump(qmc5883l);
    adxl345_dump(adxl345);
}

static void sensors_update()
{
    esp_err_t err;
    uint16_t v16;

    // Update/Read data
    if ((err = bmx280_readout(bmx280lo)) == ESP_OK) {
        bmx280lo_update = true;
    } else {
        ESP_LOGE("BME280LO", "Error=%d", err);
    }
    if ((err = bmx280_readout(bmx280hi)) == ESP_OK) {
        bmx280hi_update = true;
    } else {
        ESP_LOGE("BME280HI", "Error=%d", err);
    }
    if (scd4x->auto_adjust > 0 && scd4x->auto_adjust-- == 1) {
        bmx280_t *bmx280;

        scd4x->auto_adjust = 255;
        if (bmx280lo->values.temperature < bmx280hi->values.temperature) bmx280 = bmx280lo;
        else bmx280 = bmx280hi;

        float temp_offset = scd4x->values.temperature - bmx280->values.temperature + scd4x->temperature_offset;

        scd4x_set_temperature_offset(scd4x, temp_offset);
        scd4x_set_sensor_altitude(scd4x, bmx280->values.altitude);
        scd4x_set_ambient_pressure(scd4x, bmx280->values.pressure);
        ESP_LOGI("SCD4X", "Adjust: TempOffset=%f °C  Alt=%f m  Press=%f hPa",
                    temp_offset, bmx280->values.altitude, bmx280->values.pressure);
    }
    if (scd4x_get_data_ready_status(scd4x)) {
        if ((err = scd4x_read_measurement(scd4x)) == ESP_OK) {
            scd4x_update = true;
        }
        err = scd4x_start_periodic_measurement(scd4x);
    } else if (scd4x->enabled) {
        err = scd4x_start_periodic_measurement(scd4x);
    }
    mhz19_update = mhz19_data_ready(mhz19);
    yys_update = yys_data_ready(yys_sensors);
    if (sps30->enabled) {
        if ((err = sps30_read_device_status_register(sps30)) == ESP_OK) {
            if (sps30->status != 0) {
                ESP_LOGW("SPS30", "STATUS=%08X", (unsigned int)sps30->status);
            }
        } else {
            ESP_LOGE("SPS30", "Failed to read status");
        }
        if (sps30_read_data_ready(sps30)) {
            if ((err = sps30_read_measurement(sps30)) == ESP_OK) {
                sps30_update = true;
            } else {
                ESP_LOGE("SPS30", "Failed to read measurement values with error %d", err);
            }
        } else {
            ESP_LOGE("SPS30", "not ready");
        }
    }
    if ((err = qmc5883l_read_data(qmc5883l)) == ESP_OK) {
        qmc5883l_update = true;
    } else {
        ESP_LOGE("QMC5883L", "Error=%d", err);
    }
    if ((err = adxl345_read_data(adxl345)) == ESP_OK) {
        adxl345_update = true;
    } else {
        ESP_LOGE("ADXL345", "Error=%d", err);
    }
    gps_update = gps_data_ready(gps);
    if (xQueueReceive(gps->queue, &v16, 1)) {
        if (gps->debug & 1) {
            ESP_LOGI("GPS", "%04X", v16);
        }
    }
}

static void update_task(void *arg)
{
    ESP_LOGI(TAG, "Start main loop.");
    while (true) {
        sensors_update();
        ui_update();

        // Dump data for debugging
        dump_data();

        wdt_hal_write_protect_disable(&rtc_wdt_ctx);
        wdt_hal_feed(&rtc_wdt_ctx);
        wdt_hal_write_protect_enable(&rtc_wdt_ctx);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sensors_init()
{
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
    ESP_ERROR_CHECK(gps_init(&gps, GPS_UART_NUM, GPS_PIN_NUM_RX, GPS_PIN_NUM_TX));
    gps_status = &gps->status;
    ESP_ERROR_CHECK(adxl345_init(&adxl345, bus_handle));
    ESP_ERROR_CHECK(bmx280_init(&bmx280lo, bus_handle, false));
    ESP_ERROR_CHECK(bmx280_init(&bmx280hi, bus_handle, true));
    ESP_ERROR_CHECK(mhz19_init(&mhz19, MHZ19_UART_NUM, MHZ19_PIN_NUM_RX, MHZ19_PIN_NUM_TX));
    ESP_ERROR_CHECK(scd4x_device_init(&scd4x, bus_handle));
    ESP_ERROR_CHECK(yys_init(&yys_sensors, YYS_PIN_NUM_O2, YYS_PIN_NUM_CO, YYS_PIN_NUM_H2S));
    ESP_ERROR_CHECK(qmc5883l_init(&qmc5883l, bus_handle));
    //ESP_ERROR_CHECK(tlv493_init(&tlv493, bus_handle));
    //ESP_LOGI("TLV493D", "ChipId = %d", tlv493->device_id);
    ESP_ERROR_CHECK(sps30_init(&sps30, bus_handle));
}

void app_main(void)
{
    // Wait 100ms to give sensors time to power up.
    vTaskDelay(pdMS_TO_TICKS(100));

    config = calloc(sizeof(config_t), 1);
    config_read(config);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    bus_handle = i2c_bus_init(I2C_PIN_NUM_SDA, I2C_PIN_NUM_SCL);
    // SD-Card (SPI Mode)
    spi_host_id = sd_card_init(SDCARD_PIN_NUM_CS, SPI_PIN_NUM_SCLK, SPI_PIN_NUM_MOSI, SPI_PIN_NUM_MISO);
    // LCD (SPI Mode)
    lcd = lcd_init(spi_host_id, LCD_PIN_NUM_CS, LCD_PIN_NUM_DC, LCD_PIN_NUM_RST, LCD_PIN_NUM_LED, LCD_PIN_NUM_T_CS);
    ui = ui_init(lcd);

    led_init();
    sensors_init();
    rtc_init(&rtc, &bus_handle);
    // WiFi
    //ESP_ERROR_CHECK(wifi_init());
    /*if (ip_info.ip.addr != 0) {
        esp_netif_ip_info_t ip_info;

        ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info));

        // Print the local IP address
        ESP_LOGI(TAG, "IP Address : " IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI(TAG, "Subnet mask: " IPSTR, IP2STR(&ip_info.netmask));
        ESP_LOGI(TAG, "Gateway    : " IPSTR, IP2STR(&ip_info.gw));
        // Austria/Vienna: CET-1CEST,M3.5.0,M10.5.0/3
        setenv("TZ","CET-1CEST,M3.5.0,M10.5.0/3",1);
        tzset();
        ESP_ERROR_CHECK(sntp_obtain_time());
    }*/

    xTaskCreate(update_task, "update_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);

    console_init();
}
