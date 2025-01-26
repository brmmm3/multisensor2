#include "bmx280.h"
#include "gps.h"
#include "main.h"
#include "scd4x.h"

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

static struct {
    struct arg_int *gps;    // Debug GPS
    struct arg_int *bme;    // Debug BME280
    struct arg_int *scd;    // Debug SCD41
    struct arg_int *mhz;    // Debug MHZ19
    struct arg_int *yys;    // Debug YYS
    struct arg_int *sps;    // Debug SPS30
    struct arg_int *qmc;    // Debug QMC5883L
    struct arg_int *adx;    // Debug ADXL345
    struct arg_end *end;
} debug_cmd_args;

static struct {
    struct arg_int *temp;   // Configure Temperature offset
    struct arg_int *alt;    // Configure altitude
    struct arg_int *press;  // Configure pressure
    struct arg_end *end;
} scd4x_cmd_args;


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

void btn_pressed(lv_event_t *e)
{
    esp_err_t err;
    uint16_t frc;
    const char *key = lv_event_get_user_data(e);

    if (!strcmp(key, "CAL")) {
        ESP_LOGI(TAG, "Calibrating sensors...");
        if ((err = mhz19_calibrate_zero(mhz19)) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to calibrate MHZ19 with error %d", err);
        }
        if ((err = scd4x_stop_periodic_measurement(scd4x)) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop SCD41 with error %d", err);
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
            frc = scd4x_perform_forced_recalibration(scd4x, mhz19->co2);
            ESP_LOGI(TAG, "FRC=%d", frc);
            vTaskDelay(pdMS_TO_TICKS(400));
            if ((err = scd4x_start_periodic_measurement(scd4x)) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start SCD41 with error %d", err);
            }
        }
        ESP_LOGI(TAG, "Calibrating sensors DONE");
    }
}

void lcd_update()
{
    sps30_values_t *sps30_values = &sps30->values;
    scd4x_values_t *scd4x_values = &scd4x->values;
    uint32_t day;
    uint32_t month;
    uint32_t year;
    uint32_t hours;
    uint32_t minutes;
    uint32_t seconds;
    char buf[100];

    lv_lock_acquire();
    if (bmx280lo_update) {
        bmx280lo_update = false;
        sprintf(buf, "P=%.1f hPa  A=%.1f m\nT=%.1f °C  H=%.1f %%",
                bmx280lo->values.pressure, bmx280lo->values.altitude, bmx280lo->values.temperature, bmx280lo->values.humidity);
        lv_label_set_text(ui->lbl_bmx280lo, buf);
    }
    if (bmx280hi_update) {
        bmx280hi_update = false;
        sprintf(buf, "P=%.1f hPa  A=%.1f m\nT=%.1f °C  H=%.1f %%",
                bmx280hi->values.pressure, bmx280hi->values.altitude, bmx280hi->values.temperature, bmx280hi->values.humidity);
        lv_label_set_text(ui->lbl_bmx280hi, buf);
    }
    if (scd4x_update) {
        scd4x_update = false;
        sprintf(buf, "CO2=%d ppm\nT=%.1f °C  H=%.1f %%", scd4x_values->co2, scd4x_values->temperature, scd4x_values->humidity);
        lv_label_set_text(ui->lbl_scd4x, buf);
    }
    if (mhz19_update) {
        mhz19_update = false;
        sprintf(buf, "CO2=%d ppm\nT=%d °C ", mhz19->co2, mhz19->temp);
        lv_label_set_text(ui->lbl_mhz19, buf);
    }
    if (yys_update) {
        yys_update = false;
        sprintf(buf, "O2=%.1f %%  CO=%.1f ppm\nH2S=%.1f ppm", yys_get_o2(yys_sensors), yys_get_co(yys_sensors), yys_get_h2s(yys_sensors));
        lv_label_set_text(ui->lbl_yys, buf);
    }
    // Update GPS
    if (gps_update) {
        gps_update = false;
        day = gps_status->date / 10000;
        month = gps_status->date / 100 - day * 100;
        year = gps_status->date % 100;
        sprintf(buf, "%02lu.%02lu.20%02lu", day, month, year);
        lv_label_set_text(ui->lbl_gps_date, buf);
        hours = gps_status->time / 10000;
        minutes = gps_status->time / 100 - hours * 100;
        seconds = gps_status->time % 100;
        sprintf(buf, "%02lu:%02lu:%02lu", hours, minutes, seconds);
        lv_label_set_text(ui->lbl_gps_time, buf);
        sprintf(buf, "%f %c", gps_status->lat, gps_status->ns);
        lv_label_set_text(ui->lbl_gps_lat, buf);
        sprintf(buf, "%f %c", gps_status->lng, gps_status->ew);
        lv_label_set_text(ui->lbl_gps_lng, buf);
        sprintf(buf, "%.1f m", gps_status->altitude);
        lv_label_set_text(ui->lbl_gps_alt, buf);
        sprintf(buf, "%.1f km/h", gps_status->speed);
        lv_label_set_text(ui->lbl_gps_speed, buf);
        sprintf(buf, "%d  ST=%d 2/3D=%s", gps_status->sats, gps_status->status, gps_status->mode_3d == 2 ? "2D" : gps_status->mode_3d == 3 ? "3D": "-");
        lv_label_set_text(ui->lbl_gps_sats, buf);
    }
    // Update SPS30
    if (sps30_update) {
        sps30_update = false;
        sprintf(buf, "%.1f #/cm3", sps30_values->nc_0p5);
        lv_label_set_text(ui->lbl_sps30_1, buf);
        sprintf(buf, "%.1f ug/cm3 (%.1f #/cm3)", sps30_values->mc_1p0, sps30_values->nc_1p0);
        lv_label_set_text(ui->lbl_sps30_2, buf);
        sprintf(buf, "%.1f ug/cm3 (%.1f #/cm3)", sps30_values->mc_2p5, sps30_values->nc_2p5);
        lv_label_set_text(ui->lbl_sps30_3, buf);
        sprintf(buf, "%.1f ug/cm3 (%.1f #/cm3)", sps30_values->mc_4p0, sps30_values->nc_4p0);
        lv_label_set_text(ui->lbl_sps30_4, buf);
        sprintf(buf, "%.1f ug/cm3 (%.1f #/cm3)", sps30_values->mc_10p0, sps30_values->nc_10p0);
        lv_label_set_text(ui->lbl_sps30_5, buf);
        sprintf(buf, "%.3f um", sps30_values->typical_particle_size);
        lv_label_set_text(ui->lbl_sps30_6, buf);
    }
    // Update QMC5883L
    if (qmc5883l_update) {
        qmc5883l_update = false;
        sprintf(buf, "X=%5.2f  Y=%5.2f  Z=%5.2f", qmc5883l->mag_x, qmc5883l->mag_y, qmc5883l->mag_z);
        lv_label_set_text(ui->lbl_qmc5883L, buf);
    }
    // Update ADXL345
    if (adxl345_update) {
        adxl345_update = false;
        sprintf(buf, "%5.2f g  Moving=%d", adxl345->accel_abs, adxl345->moving_cnt);
        lv_label_set_text(ui->lbl_adxl345, buf);
    }
    lv_lock_release();
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

static void hw_task(void *arg)
{
    esp_err_t err;
    uint16_t v16;

    ESP_LOGI(TAG, "Start main loop.");
    while (true) {
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
        }
        if (scd4x_get_data_ready_status(scd4x)) {
            if ((err = scd4x_read_measurement(scd4x)) == ESP_OK) {
                scd4x_update = true;
            }
            err = scd4x_start_periodic_measurement(scd4x);
        } else {
            err = scd4x_start_periodic_measurement(scd4x);
        }
        mhz19_update = mhz19_data_ready(mhz19);
        yys_update = yys_data_ready(yys_sensors);
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

        // Update LCD
        lcd_update();

        // Dump data for debugging
        dump_data();

        wdt_hal_write_protect_disable(&rtc_wdt_ctx);
        wdt_hal_feed(&rtc_wdt_ctx);
        wdt_hal_write_protect_enable(&rtc_wdt_ctx);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void hw_init()
{
    // Wait 100ms to give sensors time to power up.
    vTaskDelay(pdMS_TO_TICKS(100));

    led_init();
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    bus_handle = i2c_bus_init(I2C_PIN_NUM_SDA, I2C_PIN_NUM_SCL);
    // SD-Card (SPI Mode)
    spi_host_id = sd_card_init(SDCARD_PIN_NUM_CS, SPI_PIN_NUM_SCLK, SPI_PIN_NUM_MOSI, SPI_PIN_NUM_MISO);
    // LCD (SPI Mode)
    lcd = lcd_init(spi_host_id, LCD_PIN_NUM_CS, LCD_PIN_NUM_DC, LCD_PIN_NUM_RST, LCD_PIN_NUM_LED, LCD_PIN_NUM_T_CS);
    ui = ui_init(lcd, btn_pressed);
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
    // RTC
    rtc_init(&rtc, &bus_handle);

    // WiFi
    ESP_ERROR_CHECK(wifi_init());
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

    xTaskCreate(hw_task, "hw_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
}

static int process_debug_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&debug_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, debug_cmd_args.end, argv[0]);
        return 1;
    }
    if (debug_cmd_args.gps->count == 1) {
        gps->debug = debug_cmd_args.gps->ival[0];
    } else if (debug_cmd_args.bme->count == 1) {
        bmx280lo->debug = debug_cmd_args.bme->ival[0];
        bmx280hi->debug = debug_cmd_args.bme->ival[0];
    } else if (debug_cmd_args.scd->count == 1) {
        scd4x->debug = debug_cmd_args.scd->ival[0];
    } else if (debug_cmd_args.mhz->count == 1) {
        mhz19->debug = debug_cmd_args.mhz->ival[0];
    } else if (debug_cmd_args.yys->count == 1) {
        yys_sensors->o2_sensor->debug = debug_cmd_args.yys->ival[0];
        yys_sensors->co_sensor->debug = yys_sensors->o2_sensor->debug;
        yys_sensors->h2s_sensor->debug = yys_sensors->o2_sensor->debug;
    } else if (debug_cmd_args.sps->count == 1) {
        sps30->debug = debug_cmd_args.sps->ival[0];
    } else if (debug_cmd_args.qmc->count == 1) {
        qmc5883l->debug = debug_cmd_args.qmc->ival[0];
    } else if (debug_cmd_args.adx->count == 1) {
        adxl345->debug = debug_cmd_args.adx->ival[0];
    } else {
        ESP_LOGE(TAG, "no valid arguments");
        return 1;
    }
    return 0;
}

static void register_debug_cmd()
{
    debug_cmd_args.gps = arg_int0("g", "gps", "<0-31>", "Configure GPS debugging");
    debug_cmd_args.bme = arg_int0("b", "bme", "<0-15>", "Configure BME280 debugging");
    debug_cmd_args.scd = arg_int0("s", "scd", "<0-15>", "Configure SCD41 debugging");
    debug_cmd_args.mhz = arg_int0("m", "mhz", "<0-15>", "Configure MHZ19 debugging");
    debug_cmd_args.yys = arg_int0("y", "yys", "<0-15>", "Configure YYS debugging");
    debug_cmd_args.sps = arg_int0("p", "sps", "<0-15>", "Configure SPS30 debugging");
    debug_cmd_args.qmc = arg_int0("q", "qmc", "<0-15>", "Configure QMC5883L debugging");
    debug_cmd_args.adx = arg_int0("a", "adx", "<0-15>", "Configure ADXL345 debugging");
    debug_cmd_args.end = arg_end(8);

    const esp_console_cmd_t cmd = {
        .command = "debug",
        .help = "configure debugging",
        .hint = NULL,
        .func = &process_debug_cmd,
        .argtable = &debug_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static int process_scd4x_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&scd4x_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, scd4x_cmd_args.end, argv[0]);
        return 1;
    }
    if (scd4x_cmd_args.temp->count == 1) {
        scd4x_set_temperature_offset(scd4x, (float)scd4x_cmd_args.temp->ival[0]);
    } else if (scd4x_cmd_args.alt->count == 1) {
        scd4x_set_sensor_altitude(scd4x, scd4x_cmd_args.alt->ival[0]);
    } else if (scd4x_cmd_args.press->count == 1) {
        scd4x_set_ambient_pressure(scd4x, scd4x_cmd_args.press->ival[0]);
    } else {
        ESP_LOGE(TAG, "no valid arguments");
        return 1;
    }
    return 0;
}

static void register_scd4x_cmd()
{
    scd4x_cmd_args.temp = arg_int0("t", "temp", "<temp>", "Temperature offset");
    scd4x_cmd_args.alt = arg_int0("a", "alt", "<alt>", "Altitude");
    scd4x_cmd_args.press = arg_int0("p", "press", "<press>", "Pressure");
    scd4x_cmd_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "scd",
        .help = "SCD4x",
        .hint = NULL,
        .func = &process_scd4x_cmd,
        .argtable = &scd4x_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void app_main(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    hw_init();

    repl_config.prompt = "MS>";
    repl_config.max_cmdline_length = 256;
    /* Register commands */
    esp_console_register_help_command();
    register_debug_cmd();
    register_scd4x_cmd();

    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
