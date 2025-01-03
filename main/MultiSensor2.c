#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hal/wdt_hal.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"

#include "hal/gpio_types.h"
#include "rom/gpio.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "gps.h"
#include "lcd.h"
#include "bmx280.h"
#include "scd4x.h"
#include "mhz19.h"
#include "sps30.h"
#include "yys.h"

static const char *SENSOR_TAG = "MS2";

// I2C
#define I2C_PORT_AUTO   -1
#define I2C_PIN_NUM_SDA GPIO_NUM_6
#define I2C_PIN_NUM_SCL GPIO_NUM_7

// UART
#define UART_BUFFER_SIZE 256

#define LED_PIN_NUM     GPIO_NUM_8


i2c_master_bus_handle_t i2c_bus_init(uint8_t sda_io, uint8_t scl_io)
{
    ESP_LOGI(SENSOR_TAG, "Initialize I2C bus");
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
    ESP_LOGI(SENSOR_TAG,"I2C master bus created");
    return bus_handle;
}

/*void scd4x_read_values(scd4x_t *scd4x)
{
    scd4x_sensors_values_t values;

    vTaskDelay(pdMS_TO_TICKS(10));
    if (scd4x_read_measurement(scd4x, &values) != ESP_OK) {
        ESP_LOGE(SENSOR_TAG, "Sensors read measurement error!");
    } else {
        float co2_level = values.co2;
        float temperature = values.temperature;
        float humidity = values.humidity;

        ESP_LOG_BUFFER_HEX_LEVEL(SENSOR_TAG, &values, sizeof(values), ESP_LOG_DEBUG);
        ESP_LOGI(SENSOR_TAG, "CO₂ %4.0f ppm - Temperature %2.1f °%c - Humidity %2.1f%%",
                    co2_level, temperature, scd4x->scale, humidity);
    }
}*/

void app_main(void)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    i2c_master_bus_handle_t bus_handle = i2c_bus_init(I2C_PIN_NUM_SDA, I2C_PIN_NUM_SCL);

    esp_lcd_panel_handle_t panel_handle = lcd_init(bus_handle);

    lcd_print(panel_handle, 0, 0, "Hello World");

    //bmx280_t *bmx280 = NULL;
    //ESP_ERROR_CHECK(bmx280_device_init(&bmx280, bus_handle));
    //ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_CYCLE));
 
    //scd4x_t *scd4x = NULL;
    //scd4x_device_init(&scd4x, bus_handle);
 
    mhz19_t *mhz19 = NULL;
    esp_err_t err = mhz19_init(&mhz19);
    ESP_LOGI(SENSOR_TAG, "mhz19_init=%d", err);
    /*sps30_t *sps30 = NULL;
    err = sps30_device_init(&sps30, bus_handle);
    ESP_LOGI(SENSOR_TAG, "sps30_device_init=%d", err);
    err = sps30_wake_up(sps30);
    ESP_LOGI(SENSOR_TAG, "sps30_wake_up=%d", err);
    ESP_LOGI(SENSOR_TAG, "SPS30 Serial = %s", sps30->serial);
    err = sps30_get_product(sps30);
    ESP_LOGI(SENSOR_TAG, "sps30_get_product=%d", err);
    err = sps30_get_firmware_version(sps30);
    ESP_LOGI(SENSOR_TAG, "sps30_get_firmware_version=%d", err);
    ESP_LOGI(SENSOR_TAG, "SPS30 Firmware Version = %d", sps30->firmware_version);
    err = sps30_start_measurement(sps30);
    ESP_LOGI(SENSOR_TAG, "sps30_start_measurement=%d", err);*/

    //gps_sensor_t gps_sensor = gps_init();

    //yys_sensors_t yys_sensors = yys_init();

    wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();

    ESP_LOGI(SENSOR_TAG, "Start main loop.");

    uint16_t v16;

    while (1) {
        /*bmx280_readout(bmx280);
        lcd_print(panel_handle, 0, 1, "T=%.2f °C", bmx280->values.temperature);
        lcd_print(panel_handle, 0, 2, "H=%.2f %%", bmx280->values.humidity);
        lcd_print(panel_handle, 0, 3, "P=%.2f hPa", bmx280->values.pressure);
        lcd_print(panel_handle, 0, 4, "A=%.2f m", bmx280->values.altitude);*/

        /*if (xQueueReceive(gps_sensor.queue, &v16, 1))
        {
            ESP_LOGI(SENSOR_TAG, "GPS %04X", v16);
        }*/

        /*if (xQueueReceive(yys_sensors.o2_sensor->queue, &v16, 1))
        {
            lcd_print(panel_handle, 0, 5, "O2=%04X", v16);
            ESP_LOGI(SENSOR_TAG, "O2 %04X", v16);
        }

        if (xQueueReceive(yys_sensors.co_sensor->queue, &v16, 1))
        {
            lcd_print(panel_handle, 0, 6, "CO=%04X", v16);
            ESP_LOGI(SENSOR_TAG, "CO %04X", v16);
        }

        if (xQueueReceive(yys_sensors.h2s_sensor->queue, &v16, 1))
        {
            lcd_print(panel_handle, 0, 7, "H2S=%04X", v16);
            ESP_LOGI(SENSOR_TAG, "H2S %04X", v16);
        }*/
        //read_scd4x_values();

        /*bool rdy = sps30_read_data_ready(sps30);
        ESP_LOGI("MS2", "SPS30Rdy=%d", rdy);
        err = sps30_read_device_status_register(sps30);
        ESP_LOGI("MS2", "STATUS=%d", err);
        if (rdy) {
            err = sps30_read_measurement(sps30);
            ESP_LOGI(SENSOR_TAG, "sps30_read_measurement=%d", err);
        }*/

        wdt_hal_write_protect_disable(&rtc_wdt_ctx);
        wdt_hal_feed(&rtc_wdt_ctx);
        wdt_hal_write_protect_enable(&rtc_wdt_ctx);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
