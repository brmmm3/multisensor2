#include "bmx280.h"
#include "config.h"
#include "gps.h"
#include "lcd.h"
#include "main.h"
#include "misc/lv_palette.h"
#include "nvs_flash.h"
#include "scd4x.h"
#include "sdcard.h"
#include "ui/include/ui_config.h"
#include "ui/include/ui_update.h"
#include "wifi/include/wifi.h"

#include "esp_task_wdt.h"
#include "console/console.h"

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
#define YYS_PIN_NUM_NC      GPIO_NUM_11
#define YYS_PIN_NUM_TX      GPIO_NUM_12
#define YYS_PIN_NUM_RX      GPIO_NUM_13

// Unused GPIOs: 3, 9

// Special GPIO functions:
//  0 = Boot Mode (H = Normal Boot, internal Pull-Up)
// 15 = JTAG Signal Source Control

// UART
#define UART_BUFFER_SIZE 256

#define CONFIG_NTP_SERVER   "pool.ntp.org"

#define UPDATE_TASK_PRIORITY 3

#define DATA_HEADER_ID0     0xAA
#define DATA_HEADER_ID1     0x55
#define DATA_HEADER_VERSION 1

rtc_t *rtc = NULL;
gps_sensor_t *gps = NULL;
gps_status_t *gps_status = NULL;
bmx280_t *bmx280lo = NULL;
bmx280_t *bmx280hi = NULL;
mhz19_t *mhz19 = NULL;
scd4x_t *scd4x = NULL;
sps30_t *sps30 = NULL;
yys_sensor_t *yys_sensor = NULL;
adxl345_t *adxl345 = NULL;
qmc5883l_t *qmc5883l = NULL;
wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
i2c_master_bus_handle_t bus_handle;
int spi_host_id;
lv_display_t *lcd;
ui_t *ui;
led_strip_handle_t led_strip;

status_t status = {0};

bool gps_update = false;
bool bmx280lo_update = false;
bool bmx280hi_update = false;
bool mhz19_update = false;
bool scd4x_update = false;
bool yys_update = false;
bool sps30_update = false;
bool adxl345_update = false;
bool qmc5883l_update = false;

// Measurement data
uint8_t data[65536];

void dump_data();


void data_add_uint16(uint16_t value)
{
    data[status.record_pos++] = value & 0xff;
    data[status.record_pos++] = (value >> 8) & 0xff;
}


void data_add_uint32(uint32_t value)
{
    data[status.record_pos++] = value & 0xff;
    data[status.record_pos++] = (value >> 8) & 0xff;
    data[status.record_pos++] = (value >> 16) & 0xff;
    data[status.record_pos++] = (value >> 24) & 0xff;
}


void data_add_gps()
{
    // Write up to 38 bytes
    data[status.record_pos++] = E_SENSOR_GPS;
    status.record_pos += sprintf((char *)&data[status.record_pos], "%s", gps->status.sat);
    status.record_pos++;
    data_add_uint32(gps->status.date);
    data_add_uint32(gps->status.time);
    data_add_uint32((uint32_t)gps->status.lat);
    data[status.record_pos++] = gps->status.ns;
    data_add_uint32((uint32_t)gps->status.lng);
    data[status.record_pos++] = gps->status.ew;
    data_add_uint32((uint32_t)gps->status.altitude);
    data_add_uint32((uint32_t)gps->status.speed);
    data[status.record_pos++] = gps->status.mode_3d;
    data[status.record_pos++] = gps->status.sats;
    data[status.record_pos++] = gps->status.status;
}


void data_add_bmx280(uint8_t id, bmx280_t *bmx280)
{
    // Write 17 bytes
    data[status.record_pos++] = id;
    data_add_uint32((uint32_t)bmx280->values.temperature);
    data_add_uint32((uint32_t)bmx280->values.humidity);
    data_add_uint32((uint32_t)bmx280->values.pressure);
    data_add_uint32((uint32_t)bmx280->values.altitude);
}


void data_add_mhz19()
{
    // Write 4 bytes
    data[status.record_pos++] = E_SENSOR_MHZ19;
    data_add_uint16(mhz19->co2);
    data[status.record_pos++] = mhz19->temp;
}


void data_add_yys()
{
    // Write 9 bytes
    data[status.record_pos++] = E_SENSOR_YYS;
    data_add_uint16(yys_sensor->co);
    data_add_uint16(yys_sensor->o2);
    data_add_uint16(yys_sensor->h2s);
    data_add_uint16(yys_sensor->ch4);
}


void data_add_sps30()
{
    // Write 41 bytes
    data[status.record_pos++] = E_SENSOR_SPS30;
    data_add_uint32((uint32_t)sps30->values.mc_1p0);
    data_add_uint32((uint32_t)sps30->values.mc_2p5);
    data_add_uint32((uint32_t)sps30->values.mc_4p0);
    data_add_uint32((uint32_t)sps30->values.mc_10p0);
    data_add_uint32((uint32_t)sps30->values.nc_0p5);
    data_add_uint32((uint32_t)sps30->values.nc_1p0);
    data_add_uint32((uint32_t)sps30->values.nc_2p5);
    data_add_uint32((uint32_t)sps30->values.nc_4p0);
    data_add_uint32((uint32_t)sps30->values.nc_10p0);
    data_add_uint32((uint32_t)sps30->values.typical_particle_size);
}


void data_add_adxl345()
{
    // Write 25 bytes
    data[status.record_pos++] = E_SENSOR_ADXL345;
    data_add_uint32((uint32_t)adxl345->accel_x);
    data_add_uint32((uint32_t)adxl345->accel_y);
    data_add_uint32((uint32_t)adxl345->accel_z);
    data_add_uint32((uint32_t)adxl345->accel_offset_x);
    data_add_uint32((uint32_t)adxl345->accel_offset_y);
    data_add_uint32((uint32_t)adxl345->accel_offset_z);
}


void data_add_qmc5883l()
{
    // Write 13 bytes
    data[status.record_pos++] = E_SENSOR_QMC5883L;
    data_add_uint32((uint32_t)qmc5883l->mag_x);
    data_add_uint32((uint32_t)qmc5883l->mag_y);
    data_add_uint32((uint32_t)qmc5883l->mag_z);
}


esp_err_t nvs_init()
{
    ESP_LOGI(TAG, "Initialize NVS");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition is truncated or version mismatch → erase and re-init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

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
    ESP_LOGI(TAG, "Initialize LED");

    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        // Set the GPIO that the LED is connected
        .strip_gpio_num = LED_PIN_NUM,
        // Set the number of connected LEDs in the strip
        .max_leds = 1,
        // LED strip model
        .led_model = LED_MODEL_WS2812,
        // Set the pixel format of your LED strip
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        // In some cases, the logic is inverted
        .flags.invert_out = false,
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        // Set the clock source
        .clk_src = RMT_CLK_SRC_XTAL,
        // Set the RMT counter clock
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        // Set the DMA feature (not supported on the ESP32-C6)
        .flags.with_dma = false,
    };

    // LED Strip object handle
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
}

void sensors_init()
{
    ESP_LOGI(TAG, "Initialize Sensors");
    ESP_ERROR_CHECK(gps_init(&gps, GPS_UART_NUM, GPS_PIN_NUM_RX, GPS_PIN_NUM_TX));
    gps_status = &gps->status;
    ESP_ERROR_CHECK(adxl345_init(&adxl345, bus_handle));
    ESP_ERROR_CHECK(bmx280_init(&bmx280lo, bus_handle, false));
    ESP_ERROR_CHECK(bmx280_init(&bmx280hi, bus_handle, true));
    ESP_ERROR_CHECK(mhz19_init(&mhz19, MHZ19_UART_NUM, MHZ19_PIN_NUM_RX, MHZ19_PIN_NUM_TX));
    ESP_ERROR_CHECK(scd4x_device_init(&scd4x, bus_handle));
    ESP_ERROR_CHECK(yys_init(&yys_sensor, YYS_PIN_NUM_RX, YYS_PIN_NUM_TX));
    ESP_ERROR_CHECK(qmc5883l_init(&qmc5883l, bus_handle));
    //ESP_ERROR_CHECK(tlv493_init(&tlv493, bus_handle));
    //ESP_LOGI("TLV493D", "ChipId = %d", tlv493->device_id);
    ESP_ERROR_CHECK(sps30_init(&sps30, bus_handle));
}

static void update_scd4x()
{
    esp_err_t err;

    if (scd4x->auto_adjust > 0 && scd4x->auto_adjust-- == 1) {
        bmx280_t *bmx280;

        scd4x->auto_adjust = 255;
        if (bmx280lo->values.temperature < bmx280hi->values.temperature) bmx280 = bmx280lo;
        else bmx280 = bmx280hi;

        float temp_offset = scd4x->values.temperature - bmx280->values.temperature + scd4x->temperature_offset;

        scd4x_set_temperature_offset(scd4x, temp_offset);
        scd4x_set_sensor_altitude(scd4x, bmx280->values.altitude);
        scd4x_set_ambient_pressure(scd4x, bmx280->values.pressure);
        if (status.recording) {
            data[status.record_pos++] = E_SENSOR_SCD4XCAL;
            data_add_uint32((uint32_t)scd4x->temperature_offset);
            data_add_uint16(scd4x->altitude);
            data_add_uint16(scd4x->pressure);
        }
        ESP_LOGI("SCD4X", "Adjust: TempOffset=%f °C  Alt=%f m  Press=%f hPa",
                    temp_offset, bmx280->values.altitude, bmx280->values.pressure);
    }
    if (scd4x_get_data_ready_status(scd4x)) {
        if ((err = scd4x_read_measurement(scd4x)) == ESP_OK) {
            scd4x_update = true;
            if (status.recording) {
                data[status.record_pos++] = E_SENSOR_SCD4X;
                data_add_uint16(scd4x->values.co2);
                data_add_uint32((uint32_t)scd4x->values.temperature);
                data_add_uint32((uint32_t)scd4x->values.humidity);
            }
        }
        err = scd4x_start_periodic_measurement(scd4x);
    } else if (scd4x->enabled) {
        err = scd4x_start_periodic_measurement(scd4x);
    }
}

static void update_sps30()
{
    esp_err_t err;

    if (!sps30->enabled) {
        return;
    }
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

static void sensors_update()
{
    esp_err_t err;
    uint16_t v16;
    bool recording = status.recording;

    // Update/Read data
    if (recording && status.record_pos == 0) {
        data[status.record_pos++] = DATA_HEADER_ID0;
        data[status.record_pos++] = DATA_HEADER_ID1;
        data[status.record_pos++] = DATA_HEADER_VERSION;
    }
    if (gps != NULL) {
        gps_update = gps_data_ready(gps);
        if (xQueueReceive(gps->queue, &v16, 1)) {
            if (gps->debug & 1) {
                ESP_LOGI("GPS", "%04X", v16);
            }
        }
        if (gps_update && recording) {
            data_add_gps();
        }
    }
    if (bmx280lo != NULL) {
        if ((err = bmx280_readout(bmx280lo)) == ESP_OK) {
            bmx280lo_update = true;
            if (recording) {
                data_add_bmx280(E_SENSOR_BMX280_LO, bmx280lo);
            }
        } else {
            ESP_LOGE("BME280LO", "Error=%d", err);
        }
    }
    if (bmx280hi != NULL) {
        if ((err = bmx280_readout(bmx280hi)) == ESP_OK) {
            bmx280hi_update = true;
            if (recording) {
                data_add_bmx280(E_SENSOR_BMX280_HI, bmx280hi);
            }
        } else {
            ESP_LOGE("BME280HI", "Error=%d", err);
        }
    }
    if (mhz19 != NULL) {
        mhz19_update = mhz19_data_ready(mhz19);
        if (mhz19_update && recording) {
            data_add_mhz19();
        }
    }
    if (scd4x != NULL) {
        update_scd4x();
    }
    if (yys_sensor != NULL) {
        yys_update = yys_data_ready(yys_sensor);
        if (yys_update && recording) {
            data_add_yys();
        }
    }
    if (sps30 != NULL) {
        update_sps30();
        if (sps30_update && recording) {
            data_add_sps30();
        }
    }
    if (adxl345 != NULL) {
        if ((err = adxl345_read_data(adxl345)) == ESP_OK) {
            adxl345_update = true;
            if (recording) {
                data_add_adxl345();
            }
        } else {
            ESP_LOGE("ADXL345", "Error=%d", err);
        }
    }
    if (qmc5883l != NULL) {
        if ((err = qmc5883l_read_data(qmc5883l)) == ESP_OK) {
            qmc5883l_update = true;
            if (recording) {
                data_add_qmc5883l();
            }
        } else {
            ESP_LOGE("QMC5883L", "Error=%d", err);
        }
    }
}

static void update_task(void *arg)
{
    ESP_LOGI(TAG, "Start main loop.");
    esp_task_wdt_add(NULL);

    // Update SD-Card info
    ensure_dir(MOUNT_POINT"/data");
    char buf[32];
    uint64_t bytes_total, bytes_free;
    sd_get_info(buf, &bytes_total, &bytes_free);
    ui_set_label_text(ui->lbl_sd_card, buf);
    sprintf(buf, "%llu MB", bytes_free / (1024 * 1024));
    ui_set_label_text(ui->lbl_sd_free, buf);
    int file_count = sd_get_file_count(MOUNT_POINT"/data");
    sprintf(buf, "%d data files", file_count);
    ui_set_label_text(ui->lbl_sd_files, buf);

    while (true) {
        sensors_update();
        ui_update();

        // Dump data for debugging
        dump_data();

        esp_task_wdt_reset();

        if (status.record_pos > 65000) {
            char buf[32];
            int pos = strlen(MOUNT_POINT);

            strcpy(buf, MOUNT_POINT);
            sprintf(&buf[pos], "/data/%llu.dat", status.start_time);
            ESP_LOGI(TAG, "Write Data to File %s", buf);
            write_bin_file(buf, data, status.record_pos);
            status.record_pos = 0;
            status.start_time = ((uint64_t)gps->status.date) << 32 | (uint64_t)gps->status.time;
            ESP_LOGI(TAG, "Data Filename=%llu.dat", status.start_time);
        } else {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void dump_data()
{
    gps_dump(gps);
    bmx280_dump(bmx280lo);
    bmx280_dump(bmx280hi);
    scd4x_dump(scd4x);
    mhz19_dump(mhz19);
    yys_dump(yys_sensor);
    sps30_dump(sps30);
    qmc5883l_dump(qmc5883l);
    adxl345_dump(adxl345);
}

void app_main(void)
{
    // Wait 500ms to give sensors time to power up.
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_task_wdt_config_t twdt_config = {
        .timeout_ms     = 5000,     // 5 seconds is safe
        .idle_core_mask = 0,        // don’t watch idle task on C6 (single core)
        .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&twdt_config);
    //esp_task_wdt_deinit();

    ESP_ERROR_CHECK(nvs_init());
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    bus_handle = i2c_bus_init(I2C_PIN_NUM_SDA, I2C_PIN_NUM_SCL);
    // SD-Card (SPI Mode)
    spi_host_id = sd_card_init(SDCARD_PIN_NUM_CS, SPI_PIN_NUM_SCLK, SPI_PIN_NUM_MOSI, SPI_PIN_NUM_MISO);

    config_read();

    // LCD (SPI Mode)
    lcd = lcd_init(spi_host_id, LCD_PIN_NUM_CS, LCD_PIN_NUM_DC, LCD_PIN_NUM_RST, LCD_PIN_NUM_LED, LCD_PIN_NUM_T_CS);
    ui = ui_init(lcd);
    ui_register_callbacks(ui);
    lcd_start();

    led_init();
    sensors_init();
    rtc_init(&rtc, &bus_handle);
    if (config->auto_connect < 4) {
        wifi_init(false);
    }

    if (!sd_card_mounted()) {
        ui_set_tab_color(4, LV_PALETTE_RED);
    }

    xTaskCreate(update_task, "update_task", 4096, NULL, UPDATE_TASK_PRIORITY, NULL);

    console_init();
}
