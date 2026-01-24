#include "esp_err.h"
#include "esp_rom_crc.h"
#include "bmx280.h"
#include "config.h"
#include "freertos/portable.h"
#include "ftp.h"
#include "gps.h"
#include "lcd.h"
#include "main.h"
#include "misc/lv_palette.h"
//#include "mqtt.h"
#include "nvs_flash.h"
#include "rtc_tiny.h"
#include "scd4x.h"
#include "sdcard.h"
#include "tcp_server.h"
#include "ui/include/ui_config.h"
#include "ui/include/ui_update.h"
#include "wifi/include/wifi.h"

#include "esp_task_wdt.h"
#include "console/include/console.h"
#include <sys/time.h>

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
#define DATA_MAX_SIZE       40000

typedef struct startup_cnt_s {
    uint32_t startup_cnt;
    uint32_t uptime_cnt;
} startup_cnt_t;

typedef struct uptime_record_s {
    time_t start_time;
    time_t save_time;
} uptime_record_t;

wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
i2c_master_bus_handle_t bus_handle;
int spi_host_id;

rtc_t *rtc = NULL;

gps_sensor_t *gps = NULL;
gps_values_t gps_values = {0};
gps_status_t *gps_status = NULL;
bmx280_t *bmx280lo = NULL;
bmx280_t *bmx280hi = NULL;
mhz19_t *mhz19 = NULL;
scd4x_t *scd4x = NULL;
sps30_t *sps30 = NULL;
yys_sensor_t *yys_sensor = NULL;
adxl345_t *adxl345 = NULL;
qmc5883l_t *qmc5883l = NULL;

led_strip_handle_t led_strip;
lv_display_t *lcd;
ui_t *ui;

status_t status = {0};

bool gps_update = false;
bool bmx280lo_update = false;
bool bmx280hi_update = false;
bool mhz19_update = false;
bool scd4x_calibrate = false;
bool scd4x_update = false;
bool yys_update = false;
bool sps30_update = false;
bool adxl345_update = false;
bool qmc5883l_update = false;

uint32_t debug_main = 0;

bool force_update_all = false;

uint8_t sps30_not_ready_cnt = 0;

// Last sensor values
sensors_data_t last_values = {0};

// Recorded measurement data
uint8_t data[DATA_MAX_SIZE + 512];

void dump_values(bool force);


void get_current_date_time(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *min, uint8_t *sec)
{
    if (gps->status.date > 0 && gps->status.time > 0 && gps->status.sats > 2 && gps->gsa.pdop < 50.0) {
        uint32_t date = gps->status.date;
        uint32_t time = gps->status.time;
        uint32_t _day = date / 10000;
        uint32_t _month = (date - _day * 10000) / 100;
        *year = (uint16_t)(date - _day * 10000 - _month * 100);
        *month = (uint8_t)_month;
        *day = (uint8_t)_day;
        uint32_t _hour = time / 10000;
        uint32_t _min = (time - _hour * 10000) / 100;
        *sec = (uint8_t)(time - _hour * 10000 - _min * 100);
        *min = (uint8_t)_min;
        *hour = (uint8_t)_hour;
    } else {
        // Fallback is RTC
        ESP_LOGD(TAG, "No GPS date/time -> fallback to RTC");
        struct tm t;
        rtc_get_datetime(rtc->rtc, &t);
        *year = t.tm_year + 1900;
        *month = t.tm_mon + 1;
        *day = t.tm_mday;
        *hour = t.tm_hour;
        *min = t.tm_min;
        *sec = t.tm_sec;
    }
}


void data_add_uint32(uint32_t value)
{
    data[status.record_pos++] = value & 0xff;
    data[status.record_pos++] = (value >> 8) & 0xff;
    data[status.record_pos++] = (value >> 16) & 0xff;
    data[status.record_pos++] = (value >> 24) & 0xff;
}


void data_add(uint8_t id, void *values, size_t size)
{
    data[status.record_pos++] = id;
    memcpy(&data[status.record_pos], (uint8_t *)values, size);
    status.record_pos += size;
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

void sensors_init()
{
    ESP_LOGI(TAG, "Initialize Sensors");
    // This will never fail here
    gps_init(&gps, GPS_UART_NUM, GPS_PIN_NUM_RX, GPS_PIN_NUM_TX);
    gps_status = &gps->status;
    ESP_ERROR_CHECK_WITHOUT_ABORT(bmx280_init(&bmx280lo, bus_handle, false));
    ESP_ERROR_CHECK_WITHOUT_ABORT(bmx280_init(&bmx280hi, bus_handle, true));
    // This will never fail here
    mhz19_init(&mhz19, MHZ19_UART_NUM, MHZ19_PIN_NUM_RX, MHZ19_PIN_NUM_TX);
    ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_init(&scd4x, bus_handle));
    // This will never fail here
    yys_init(&yys_sensor, YYS_PIN_NUM_RX, YYS_PIN_NUM_TX);
    ESP_ERROR_CHECK_WITHOUT_ABORT(sps30_init(&sps30, bus_handle));
    ESP_ERROR_CHECK_WITHOUT_ABORT(adxl345_init(&adxl345, bus_handle));
    ESP_ERROR_CHECK_WITHOUT_ABORT(qmc5883l_init(&qmc5883l, bus_handle));
}

static bool update_gps()
{
    bool force_update = force_update_all || (debug_main & 1) != 0;

    if (gps_data_ready(gps)) {
        gps_values.sat = gps->status.sat;
        gps_values.date = gps->status.date;
        gps_values.time = gps->status.time;
        gps_values.lat = gps->status.lat;
        gps_values.ns = gps->status.ns;
        gps_values.lng = gps->status.lng;
        gps_values.ew = gps->status.ew;
        gps_values.altitude = gps->status.altitude;
        gps_values.speed = gps->status.speed;
        gps_values.mode_3d = gps->status.mode_3d;
        gps_values.sats = gps->status.sats;
        gps_values.pdop = gps->gsa.pdop;
        gps_values.hdop = gps->gsa.hdop;
        gps_values.vdop = gps->gsa.vdop;
        gps_values.status = gps->status.status;
        gps_values.data_cnt = gps->status.data_cnt;
        gps_values.error_cnt = gps->status.error_cnt;
        if (force_update || memcmp(&last_values.gps, &gps_values, sizeof(sensors_data_gps_t)) != 0) {
            memcpy(&last_values.gps, &gps_values, sizeof(sensors_data_gps_t));
            return true;
        }
    }
    return false;
}

static bool update_bmx280(int num, bmx280_t **sensor, sensors_data_bmx280_t *last_values)
{
    if (sensor == NULL && (debug_main & 0x2000) == 0) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(bmx280_init(sensor, bus_handle, num != 0));
        if (*sensor == NULL) return false;
    }

    esp_err_t err;
    bool force_update = force_update_all || (debug_main & 1) != 0;

    if ((err = bmx280_readout(*sensor)) != ESP_OK) {
        ESP_LOGE(TAG, "BME280_%d: err=%d", num, err);
        return false;
    }
    if (force_update || memcmp(last_values, &(*sensor)->values, sizeof(sensors_data_bmx280_t)) != 0) {
        memcpy(last_values, &(*sensor)->values, sizeof(sensors_data_bmx280_t));
        return true;
    }
    return false;
}

static bool update_mhz19()
{
    bool force_update = force_update_all || (debug_main & 1) != 0;

    if (mhz19_data_ready(mhz19)) {
        if (force_update || memcmp(&last_values.mhz19, &mhz19->values, sizeof(sensors_data_mhz19_t)) != 0) {
            memcpy(&last_values.mhz19, &mhz19->values, sizeof(sensors_data_mhz19_t));
            return true;
        }
    }
    return false;
}

static bool update_scd4x()
{
    esp_err_t err;

    if (scd4x == NULL && (debug_main & 0x2000) == 0) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_init(&scd4x, bus_handle));
        if (scd4x == NULL) return false;
    }
    if (scd4x_st_machine_status == SCD4X_ST_IDLE || scd4x_st_machine_status > SCD4X_ST_MEASURE_3MIN) {
        if (scd4x_st_machine_status > SCD4X_ST_MEASURE_3MIN) {
            ESP_LOGW(TAG, "SCD4x is busy with status %d. Unable to read values.", scd4x_st_machine_status);
        }
        return false;
    }
    if (scd4x->auto_adjust > 0 && scd4x->auto_adjust-- == 1) {
        bmx280_t *bmx280;
        esp_err_t err;

        scd4x->auto_adjust = 255;
        if (bmx280lo->values.temperature < bmx280hi->values.temperature) bmx280 = bmx280lo;
        else bmx280 = bmx280hi;

        float temp_offset = scd4x->values.temperature - bmx280->values.temperature + scd4x->temperature_offset;

        ESP_LOGI(TAG, "Temp_Offset for SCD4x set to %f", temp_offset);
        if ((err = scd4x_stop_periodic_measurement(scd4x)) == ESP_OK) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_set_temperature_offset(scd4x, temp_offset));
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_set_sensor_altitude(scd4x, bmx280->values.altitude));
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_set_ambient_pressure(scd4x, bmx280->values.pressure));
            if ((err = scd4x_start_periodic_measurement(scd4x)) != ESP_OK) {
                ESP_LOGE(TAG, "Failed start periodic measurement: %d", err);
            }
        } else {
            ESP_LOGE(TAG, "Failed to stop periodic measurement: %d", err);
        }
        scd4x_calibrate = true;
        ESP_LOGI(TAG, "SCD4X Adjust: TempOffset=%f °C  Alt=%f m  Press=%f hPa",
                    temp_offset, bmx280->values.altitude, bmx280->values.pressure);
    }
    if (!scd4x_get_data_ready_status(scd4x)) {
        return force_update_all;
    }
    if ((err = scd4x_read_measurement(scd4x)) != ESP_OK) {
        ESP_LOGE(TAG, "SCD4X read error %d", err);
        return false;
    }

    bool force_update = force_update_all || (debug_main & 1) != 0;

    if (force_update || memcmp(&last_values.scd4x, &scd4x->values, sizeof(sensors_data_scd4x_t)) != 0) {
        memcpy(&last_values.scd4x, &scd4x->values, sizeof(sensors_data_scd4x_t));
        return true;
    }
    return false;
}

static bool update_yys()
{
    bool force_update = force_update_all || (debug_main & 1) != 0;

    if (yys_data_ready(yys_sensor)) {
        if (force_update || memcmp(&last_values.yys, &yys_sensor->values, sizeof(sensors_data_yys_t)) != 0) {
            memcpy(&last_values.yys, &yys_sensor->values, sizeof(sensors_data_yys_t));
            return true;
        }
    }
    return false;
}

static bool update_sps30()
{
    if (sps30 == NULL && (debug_main & 0x2000) == 0) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(sps30_init(&sps30, bus_handle));
        if (sps30 == NULL) return false;
    }
    if (!sps30->enabled) {
        return false;
    }

    esp_err_t err;

    if ((err = sps30_read_device_status_register(sps30)) == ESP_OK) {
        if (sps30->values.status != 0) {
            ESP_LOGW("SPS30", "STATUS=%08X", (unsigned int)sps30->values.status);
        }
    } else {
        ESP_LOGE("SPS30", "Failed to read status");
    }
    if (!sps30_read_data_ready(sps30)) {
        if (sps30_not_ready_cnt++ > 5) {
            ESP_LOGE(TAG, "SPS30: not ready");
        }
        return false;
    }
    sps30_not_ready_cnt = 0;
    if ((err = sps30_read_measurement(sps30)) != ESP_OK) {
        ESP_LOGE(TAG, "SPS30: Failed to read measurement values err=%d", err);
        return false;
    }

    bool force_update = force_update_all || (debug_main & 1) != 0;

    if (force_update || memcmp(&last_values.sps30, &sps30->values, sizeof(sensors_data_sps30_t)) != 0) {
        memcpy(&last_values.sps30, &sps30->values, sizeof(sensors_data_sps30_t));
        return true;
    }
    return true;
}

static bool update_adxl345()
{
    esp_err_t err;

    if (adxl345 == NULL && (debug_main & 0x2000) == 0) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(adxl345_init(&adxl345, bus_handle));
        if (adxl345 == NULL) return false;
    }
    if ((err = adxl345_read_data(adxl345)) != ESP_OK) {
        ESP_LOGE(TAG, "ADXL345 read error %d", err);
        return false;
    }

    bool force_update = force_update_all || (debug_main & 1) != 0;

    if (force_update || memcmp(&last_values.adxl345, &adxl345->values, sizeof(sensors_data_adxl345_t)) != 0) {
        memcpy(&last_values.adxl345, &adxl345->values, sizeof(sensors_data_adxl345_t));
        return true;
    }
    return false;
}

static bool update_qmc5883l()
{
    esp_err_t err;

    if (qmc5883l == NULL && (debug_main & 0x2000) == 0) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(qmc5883l_init(&qmc5883l, bus_handle));
        if (qmc5883l == NULL) return false;
    }
    if ((err = qmc5883l_read_data(qmc5883l)) != ESP_OK) {
        ESP_LOGE(TAG, "ADXL345 read error %d", err);
        return false;
    }

    bool force_update = force_update_all || (debug_main & 1) != 0;

    if (force_update || memcmp(&last_values.qmc5883l, &qmc5883l->values, sizeof(sensors_data_qmc5883l_t)) != 0) {
        memcpy(&last_values.qmc5883l, &qmc5883l->values, sizeof(sensors_data_qmc5883l_t));
        return true;
    }
    return false;
}

static void update_gps_status()
{
    if (gps_status == NULL) return;
    if (gps_status->old_data_cnt == gps_status->data_cnt) {
        ui_set_tab_color(2, LV_PALETTE_GREY);
    } else if (gps_values.date == 0 || gps_values.lat == 0 || gps_values.altitude == 0) {
        ui_set_tab_color(2, LV_PALETTE_RED);
    } else if (gps_values.sats < 4) {
        ui_set_tab_color(2, LV_PALETTE_YELLOW);
    } else {
        ui_set_tab_color(2, LV_PALETTE_GREEN);
    }
    gps_status->old_data_cnt = gps_status->data_cnt;
    ui_set_dop_value(ui->lbl_gps_pdop, gps_values.pdop);
    ui_set_dop_value(ui->lbl_gps_hdop, gps_values.hdop);
    ui_set_dop_value(ui->lbl_gps_vdop, gps_values.vdop);
}

static void sensors_update()
{
    // Update/Read data
    gps_update |= update_gps();
    bmx280lo_update |= update_bmx280(0, &bmx280lo, &last_values.bmx280lo);
    bmx280hi_update |= update_bmx280(1, &bmx280hi, &last_values.bmx280hi);
    mhz19_update |= update_mhz19();
    scd4x_update |= update_scd4x();
    yys_update |= update_yys();
    sps30_update |= update_sps30();
    adxl345_update |= update_adxl345();
    qmc5883l_update |= update_qmc5883l();
}


static void sensors_recording()
{
    if (!gps_update && !bmx280lo_update && !bmx280hi_update && !mhz19_update && !scd4x_calibrate &&
        !scd4x_update && !yys_update && !sps30_update && !adxl345_update && !qmc5883l_update) {
        return;
    }
    bool log_values = (debug_main & 2) != 0;
    uint16_t record_pos = status.record_pos;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    get_current_date_time(&year, &month, &day, &hour, &min, &sec);
    if (status.record_pos == 0) {
        data[status.record_pos++] = DATA_HEADER_ID0;
        data[status.record_pos++] = DATA_HEADER_ID1;
        data[status.record_pos++] = DATA_HEADER_VERSION;
        // Add current date
        data[status.record_pos++] = E_SENSOR_DATE;
        data[status.record_pos++] = year;
        data[status.record_pos++] = month;
        data[status.record_pos++] = day;
        if (log_values) {
            ESP_LOGI(TAG, "DATE: %02d.%02d.%02d", year, month, day);
        }
    }
    // Add current time
    data[status.record_pos++] = E_SENSOR_TIME;
    data[status.record_pos++] = hour;
    data[status.record_pos++] = min;
    data[status.record_pos++] = sec;
    if (log_values) {
        ESP_LOGI(TAG, "TIME: %02d:%02d.%02d", hour, min, sec);
    }
    if (gps_update) {
        data_add(E_SENSOR_GPS, &last_values.gps, sizeof(sensors_data_gps_t));
        if (log_values || (debug_main & 4) != 0) {
            ESP_LOGI(TAG, "GPS: %s date=%lu time=%lu lat=%f %c lng=%f %c alt=%.1f spd=%.1f mode_3d=%c sats=%d pdop=%.1f hdop=%.1f vdop=%.1f st=%d dc=%d err=%d",
                    gps_values.sat, gps_values.date, gps_values.time, gps_values.lat, gps_values.ns, gps_values.lng,
                    gps_values.ew, gps_values.altitude, gps_values.speed, gps_values.mode_3d, gps_values.sats,
                    gps_values.pdop, gps_values.hdop, gps_values.vdop, gps_values.status, gps_values.data_cnt, gps_values.error_cnt);
        }
    }
    if (bmx280lo_update) {
        data_add(E_SENSOR_BMX280_LO, &last_values.bmx280lo, sizeof(sensors_data_bmx280_t));
        if (log_values || (debug_main & 8) != 0) {
            sensors_data_bmx280_t *values = &last_values.bmx280lo;
            ESP_LOGI(TAG, "BMX280LO: temp=%.1f °C  hum=%.1f  press=%.1f hPa  alt=%.1f m",
                    values->temperature, values->humidity, values->pressure, values->altitude);
        }
    }
    if (bmx280hi_update) {
        data_add(E_SENSOR_BMX280_HI, &last_values.bmx280hi, sizeof(sensors_data_bmx280_t));
        if (log_values || (debug_main & 8) != 0) {
            sensors_data_bmx280_t *values = &last_values.bmx280hi;
            ESP_LOGI(TAG, "BMX280HI: temp=%.1f °C  hum=%.1f  press=%.1f hPa  alt=%.1f m",
                    values->temperature, values->humidity, values->pressure, values->altitude);
        }
    }
    if (mhz19_update) {
        data_add(E_SENSOR_MHZ19, &last_values.mhz19, sizeof(sensors_data_mhz19_t));
        if (log_values || (debug_main & 16) != 0) {
            sensors_data_mhz19_t *values = &last_values.mhz19;
            ESP_LOGI(TAG, "MHZ19: co2=%d ppm  temp=%d °C  st=%d", values->co2, values->temp, values->status);
        }
    }
    if (scd4x_calibrate) {
        scd4x_cal_values_t values = {
            .temperature_offset = scd4x->temperature_offset,
            .altitude = scd4x->altitude,
            .pressure =scd4x->pressure
        };
        data_add(E_SENSOR_SCD4XCAL, &values, sizeof(scd4x_cal_values_t));
        if (log_values || (debug_main & 16) != 0) {
            ESP_LOGI(TAG, "SCD4XCAL: temp_offs=%f  alt=%d  press=%d", values.temperature_offset, values.altitude, values.pressure);
        }
    }
    if (scd4x_update) {
        data_add(E_SENSOR_SCD4X, &last_values.scd4x, sizeof(sensors_data_scd4x_t));
        if (log_values || (debug_main & 16) != 0) {
            sensors_data_scd4x_t *values = &last_values.scd4x;
            ESP_LOGI(TAG, "SCD4X: co2=%d ppm  temp=%.1f °C  hum=%.1f %%", values->co2, values->temperature, values->humidity);
        }
    }
    if (yys_update) {
        data_add(E_SENSOR_YYS, &last_values.yys, sizeof(sensors_data_yys_t));
        if (log_values || (debug_main & 32) != 0) {
            ESP_LOGI(TAG, "YYS: o2=%.1f %%  co=%.1f ppm  h2s=%.1f ppm  ch4=%.1f ppm",
                    yys_get_o2(yys_sensor), yys_get_co(yys_sensor),
                    yys_get_h2s(yys_sensor), yys_get_ch4(yys_sensor));
        }
    }
    if (sps30_update) {
        data_add(E_SENSOR_SPS30, &last_values.sps30, sizeof(sensors_data_sps30_t));
        if (log_values || (debug_main & 64) != 0) {
            sensors_data_sps30_t *values = &last_values.sps30;
            ESP_LOGI(TAG, "SPS30: STATUS=%08X", (unsigned int)values->status);
            ESP_LOGI(TAG, "PM0.5 =%.1f #/cm3", values->nc_0p5);
            ESP_LOGI(TAG, "PM1.0 =%.1f ug/cm3 P1.0 =%.1f #/cm3", values->mc_1p0, values->nc_1p0);
            ESP_LOGI(TAG, "PM2.5 =%.1f ug/cm3 P2.5 =%.1f #/cm3", values->mc_2p5, values->nc_2p5);
            ESP_LOGI(TAG, "PM4.0 =%.1f ug/cm3 P4.0 =%.1f #/cm3", values->mc_4p0, values->nc_4p0);
            ESP_LOGI(TAG, "PM10.0=%.1f ug/cm3 P10.0=%.1f #/cm3", values->mc_10p0, values->nc_10p0);
            ESP_LOGI(TAG, "TypPartSz=%.3f um", values->typical_particle_size);
        }
    }
    if (adxl345_update) {
        data_add(E_SENSOR_ADXL345, &last_values.adxl345, sizeof(sensors_data_adxl345_t));
        if (log_values || (debug_main & 128) != 0) {
            sensors_data_adxl345_t *values = &last_values.adxl345;
            ESP_LOGI(TAG, "ADXL345: x=%f g  y=%f g  z=%f g  abs=%f g  offs=%f %f %f",
                    values->accel_x, values->accel_y, values->accel_z, values->accel_abs,
                    values->accel_offset_x, values->accel_offset_y, values->accel_offset_z);
        }
    }
    if (qmc5883l_update) {
        data_add(E_SENSOR_QMC5883L, &last_values.qmc5883l, sizeof(sensors_data_qmc5883l_t));
        if (log_values || (debug_main & 128) != 0) {
            sensors_data_qmc5883l_t *values = &last_values.qmc5883l;
            ESP_LOGI(TAG, "QMC5883L: x=%f gauss  y=%f gauss  z=%f gauss  range=%d  st=%d",
                values->mag_x, values->mag_y, values->mag_z, values->range, values->status);
        }
    }
    if (log_values) {
        ESP_LOGI(TAG, "WRITTEN: %d bytes", status.record_pos - record_pos);
    }
}

void show_startup_uptime_cnt()
{
    char buf[40];

    sprintf(buf, "startup=%u  uptime=%d", (unsigned int)status.startup_cnt, (unsigned int)status.uptime_cnt);
    ui_set_label_text(ui->lbl_counter, buf);
}

esp_err_t update_startup_cnt(uint32_t startup_cnt_inc, uint32_t uptime_cnt_inc)
{
    char *path = MOUNT_POINT"/startup.cnt";
    startup_cnt_t cnt = {0};

    read_bin_file(path, &cnt, sizeof(cnt));
    cnt.startup_cnt += startup_cnt_inc;
    cnt.uptime_cnt += uptime_cnt_inc;
    status.startup_cnt = cnt.startup_cnt;
    status.uptime_cnt = cnt.uptime_cnt;
    ESP_LOGI(TAG, "startup_cnt=%d  uptime_cnt=%d", cnt.startup_cnt, cnt.uptime_cnt);
    return write_bin_file(path, &cnt, sizeof(cnt));
}

esp_err_t update_save_time_file()
{
    uptime_record_t *data;
    uint32_t size = 250 * sizeof(uptime_record_t);
    uint16_t file_ext = (status.uptime_cnt / 250) % 1000;
    uint16_t pos = status.uptime_cnt % 250;
    char path[32];

    data = pvPortMalloc(size);
    sprintf(path, "%s/uptime.%03u", MOUNT_POINT, file_ext);
    read_bin_file(path, data, size);
    if (data[pos].start_time != status.start_time) {
        if (data[pos].start_time == 0 && ++pos >= 250) {
            pos = 0;
            file_ext++;
            sprintf(path, "%s/uptime.%03u", MOUNT_POINT, file_ext);
        }
        data[pos].start_time = status.start_time;
    }
    data[pos].save_time = status.save_time;
    esp_err_t err = write_bin_file(path, data, size);
    vPortFree(data);
    return err;
}

void set_data_filename()
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    get_current_date_time(&year, &month, &day, &hour, &min, &sec);
    sprintf(status.filename, "%02d%02d%02d_%02d%02d%02d.dat", year, month, day, hour, min, sec);
    ESP_LOGI(TAG, "New Data Filename=%s", status.filename);
}


uint32_t calculate_crc32(const uint8_t *data, size_t len)
{
    // Standard ZIP/PNG/zlib compatible CRC32
    return esp_rom_crc32_le(0xFFFFFFFF, data, len) ^ 0xFFFFFFFF;
}


esp_err_t set_sys_time(struct tm *timeinfo, bool set_rtc_time)
{
    time_t utc_time = mktime(timeinfo);
    if (utc_time == -1) {
        ESP_LOGE(TAG, "mktime failed");
        return ESP_FAIL;
    }

    // Set system time
    struct timeval tv;
    tv.tv_sec = utc_time;
    tv.tv_usec = 0;
    if (settimeofday(&tv, NULL) != 0) {
        ESP_LOGE(TAG, "settimeofday failed");
        return ESP_FAIL;
    }
    if (!set_rtc_time) return ESP_OK;
    return rtc_set_datetime(rtc->rtc, timeinfo);
}

void show_sd_card_info(int file_cnt)
{
    char buf[64];
    uint64_t bytes_total, bytes_free;

    sd_card_get_info(buf, &bytes_total, &bytes_free);
    ui_set_label_text(ui->lbl_sd_card, buf);
    sprintf(buf, "%llu MB", bytes_free / (1024 * 1024));
    ui_set_label_text(ui->lbl_sd_free, buf);
    sprintf(buf, "%llu kB", (bytes_total - bytes_free) / 1024);
    ui_set_label_text(ui->lbl_sd_used, buf);
    if (file_cnt < 0) {
        status.file_cnt = sd_card_get_file_count(MOUNT_POINT"/data");
        file_cnt = status.file_cnt;
    }
    sprintf(buf, "%d data files", file_cnt);
    ui_set_label_text(ui->lbl_sd_files, buf);
}

esp_err_t ensure_sd_card_mounted()
{
    if (sd_card_mounted(true)) return ESP_OK;
    esp_err_t err;
    ESP_LOGW(TAG, "SD-Card NOT mounted. Try to mount...");
    if ((err = sd_card_mount_fs()) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD-Card.");
        ui_set_tab_color(4, LV_PALETTE_RED);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "SD-Card mount success.");
    if ((err = ensure_dir(MOUNT_POINT"/data")) != ESP_OK) {
        ESP_LOGE(TAG, "SD-Card has no data folder!");
        ui_set_tab_color(4, LV_PALETTE_RED);
        return ESP_FAIL;
    }
    ui_set_tab_color(4, LV_PALETTE_GREEN);
    show_sd_card_info(-1);
    ui_sd_record_set_value(config->auto_record);
    return ESP_OK;
}

esp_err_t write_data_file()
{
    char buf[64];
    esp_err_t err;

    int pos = strlen(MOUNT_POINT);
    strcpy(buf, MOUNT_POINT);
    sprintf(&buf[pos], "/data/%s", status.filename);
    ESP_LOGI(TAG, "Write %d bytes to File %s", status.record_pos, buf);
    if ((err = write_bin_file(buf, data, status.record_pos)) != ESP_OK) return err;
    status.record_pos = 0;
    status.file_cnt++;
    sprintf(buf, "%d data files", status.file_cnt);
    ui_set_label_text(ui->lbl_sd_files, buf);
    ui_sd_set_fill_level(0.0);
    set_data_filename();
    if (status.save_time == 0) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(update_startup_cnt(0, 1));
    }
    return ESP_OK;
}

static void update_task(void *arg)
{
    char buf[64];
    uint32_t loop_cnt = 0;
    bool old_wifi_connected = false;
    esp_err_t err;

    ESP_LOGI(TAG, "Start update_task");
    esp_task_wdt_add(NULL);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ensure_dir(MOUNT_POINT"/data"));
    ui_set_switch_state(ui->sw_wifi_auto, config->wifi_auto_connect);
    ui_set_switch_state(ui->sw_tcp_server_auto, config->tcp_auto_start);
    ui_set_switch_state(ui->sw_ftp_server_auto, config->ftp_auto_start);
    ui_set_switch_state(ui->sw_auto_record, config->auto_record);
    show_sd_card_info(-1);

    while (true) {
        if (loop_cnt > 1) {
            if (wifi_connected) {
                if (!tcp_server_running && config->tcp_auto_start) {
                    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_tcp_server_enable(true));
                }
                if (!ftp_server_running() && config->ftp_auto_start) {
                    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_ftp_server_enable(true));
                }
            } else {
                if (config->wifi_auto_connect) {
                    if (config->wifi_auto_connect_idx < 4) {
                        ensure_wifi_init(true);
                    }
                }
                if (tcp_server_running) {
                    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_tcp_server_enable(false));
                }
                if (ftp_server_running()) {
                    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_ftp_server_enable(false));
                }
            }
        }
        // Check if SD card is mounted. Try to mount if not mounted
        loop_cnt++;
        if (loop_cnt % 10 == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(ensure_sd_card_mounted());
        } else if (loop_cnt > 0 && !status.recording && config->auto_record) {
            if (ensure_sd_card_mounted() == ESP_OK) {
                ui_sd_record_set_value(true);
            }
        }
        if ((debug_main & 0x200) == 0) {
            sensors_update();
            scd4x_state_machine(scd4x);
            if (status.recording) {
                sensors_recording();
            }
        }
        if ((debug_main & 0x400) == 0) {
            ui_update(status.force_update);
        }
        status.force_update = false;

        status.rssi = wifi_get_rssi();
        if (wifi_netif_enabled()) {
            sprintf(buf, "%d dBm", status.rssi);
            ui_set_label_text(ui->lbl_wifi_rssi, buf);
        } else {
            ui_set_label_text(ui->lbl_wifi_rssi, "-");
        }
        if (wifi_connected) {
            if (!old_wifi_connected) {
                old_wifi_connected = true;
            }
            //mqtt_publish_values();
            tcp_server_publish_values();
            if (!ftp_server_running() && config->ftp_auto_start) {
                ftp_server_start();
            }
        } else {
            if (tcp_server_running) tcp_server_stop();
            if (ftp_server_running()) ftp_server_stop();
        }

        // Dump data for debugging
        dump_values(false);

        esp_task_wdt_reset();

        // Show system time
        time_t now = time(NULL);
        if (now - status.start_time > loop_cnt + 36000) {
            ESP_LOGW(TAG, "start_time deviation more than 10 days. SYNC");
            status.start_time = now;
        }

        // On tap temporarily set backlight to max
        if (lcd_touch_time > 0) {
            if (now - lcd_touch_time > 5) {
                lcd_touch_time = 0;
                if (config->lcd_pwr != status.lcd_pwr) {
                    ui_lcd_set_tmp_lcd_pwr(config->lcd_pwr);
                }
            } else if (config->lcd_pwr > 0 && config->lcd_pwr == status.lcd_pwr) {
                ui_lcd_set_tmp_lcd_pwr(0);
            }
        }

        update_gps_status();

        if (status.record_pos > DATA_MAX_SIZE) {
            // Write end marker and checksum
            data[status.record_pos++] = DATA_HEADER_ID1;
            data[status.record_pos++] = DATA_HEADER_ID0;
            uint32_t cksum = calculate_crc32(data, status.record_pos);
            data_add_uint32(cksum);

            for (int i = 0; i < 2; i++) {
                if ((err = write_data_file()) == ESP_OK) {
                    status.save_time = now;
                    ui_set_time_value(ui->lbl_savetime, &status.save_time);
                    update_save_time_file();
                    break;
                }
                ESP_LOGE(TAG, "Failed to write data file. Try to mount FS.");
                ESP_ERROR_CHECK_WITHOUT_ABORT(ensure_sd_card_mounted());
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            if (ensure_sd_card_mounted() != ESP_OK) {
                ESP_LOGE(TAG, "Failed to mount FS. Giving up! Stop recording.");
                ui_set_switch_state(ui->sw_record, false);
                ui_sd_record_set_value(false);
            }
        } else {
            if ((debug_main & 0x800) == 0) {
                // Update SD-Card tab
                float fill_level = 100.0 * (float) status.record_pos / (float)DATA_MAX_SIZE;
                ui_sd_set_fill_level(fill_level);
                // Update Cfg tab
                ui_set_time_value(ui->lbl_time, &now);
                ui_set_duration_value(ui->lbl_uptime, (uint32_t)(now - status.start_time));
            }
            // Free HEAP
            size_t total_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
            size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
            int frag = 100 - (largest_block * 100 / (total_free + 1));
            if (total_free < 10000 || frag > 50 || (debug_main & 0x100) != 0) {
                ESP_LOGW(TAG, "Heap: free=%u largest_block=%u frag=%d%%",
                    total_free, largest_block, frag);
            }
            if ((debug_main & 0x800) == 0) {
                sprintf(buf, "%u B  frag=%d%%", total_free, frag);
                ui_set_label_text(ui->lbl_heap, buf);
            }
            // Wait up to 1s
            for (int i = 0; i < 10; i++) {
                if (status.force_update) break;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        // Reset update flags
        gps_update = false;
        bmx280lo_update = false;
        bmx280hi_update = false;
        mhz19_update = false;
        scd4x_calibrate = false;
        scd4x_update = false;
        yys_update = false;
        sps30_update = false;
        adxl345_update = false;
        qmc5883l_update = false;
        if (gps_status != NULL) gps_status->status = 0;
    }
}

void dump_values(bool force)
{
    if (gps != NULL) gps_dump_values(gps, force);
    if (bmx280lo != NULL) bmx280_dump_values(bmx280lo, force);
    if (bmx280hi != NULL) bmx280_dump_values(bmx280hi, force);
    if (scd4x != NULL) scd4x_dump_values(scd4x, force);
    if (mhz19 != NULL) mhz19_dump_values(mhz19, force);
    if (yys_sensor != NULL) yys_dump_values(yys_sensor, force);
    if (sps30 != NULL) sps30_dump_values(sps30, force);
    if (qmc5883l != NULL) qmc5883l_dump_values(qmc5883l, force);
    if (adxl345 != NULL) adxl345_dump_values(adxl345, force);
}

void app_main(void)
{
    // Wait 100ms to give sensors time to power up.
    vTaskDelay(pdMS_TO_TICKS(100));

    /*esp_task_wdt_config_t twdt_config = {
        .timeout_ms     = 8000,     // 8 seconds is safe
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Subscribe all idle cores (ESP32-C6 is single-core, so 1)
        .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&twdt_config);*/
    //esp_task_wdt_deinit();

    ESP_ERROR_CHECK(nvs_init());
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    bus_handle = i2c_bus_init(I2C_PIN_NUM_SDA, I2C_PIN_NUM_SCL);

    // We must call lvgl_port_init before initializing SD-Card, because sd_card driver uses LVGL lock.
    ESP_ERROR_CHECK(lcd_lvgl_port_init());

    // SD-Card (SPI Mode)
    spi_host_id = sd_card_init(SDCARD_PIN_NUM_CS, SPI_PIN_NUM_SCLK, SPI_PIN_NUM_MOSI, SPI_PIN_NUM_MISO);

    config_read();
    config_show();

    // LCD (SPI Mode)
    lcd = lcd_init(spi_host_id, LCD_PIN_NUM_CS, LCD_PIN_NUM_DC, LCD_PIN_NUM_RST, LCD_PIN_NUM_LED, LCD_PIN_NUM_T_CS);
    ui = ui_init(lcd);
    ui_register_callbacks(ui);

    ESP_ERROR_CHECK_WITHOUT_ABORT(update_startup_cnt(1, 0));
    show_startup_uptime_cnt();

    led_init();
    sensors_init();
    rtc_init(&rtc, &bus_handle);

    if (!sd_card_mounted(true)) {
        ui_set_tab_color(4, LV_PALETTE_RED);
    }
    console_init();

    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_lcd_set_pwr_mode(config->lcd_pwr));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_gps_set_pwr_mode(config->gps_pwr));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_scd4x_set_pwr_mode(config->scd4x_pwr));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_wifi_set_pwr_mode(config->wifi_pwr));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ui_mode_set_pwr_mode(config->mode_pwr));

    // Set system and startup time
    struct tm timeinfo;
    char buf[32];
    esp_err_t err;

    if ((err = (rtc_get_datetime(rtc->rtc, &timeinfo))) == ESP_OK) {
        uint16_t year = 1900 + timeinfo.tm_year;
        uint8_t mon = timeinfo.tm_mon + 1;

        sprintf(buf, "%d.%02d.%02d %02d:%02d:%02d",
            year, mon, timeinfo.tm_mday,
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        ESP_LOGI(TAG, "Set system time to: %s", buf);
        ESP_ERROR_CHECK_WITHOUT_ABORT(set_sys_time(&timeinfo, false));
    } else {
        ESP_LOGE(TAG, "Failed to get date and time from RTC: err=%d", err);
    }
    status.start_time = time(NULL);

    xTaskCreate(update_task, "update_task", 4096, NULL, UPDATE_TASK_PRIORITY, NULL);
    //update_task(NULL);
}
