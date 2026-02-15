#include <string.h>
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "rmt_uart.h"
#include "ze08.h"

static const char *TAG = "ZE08";

// UART
#define UART_BUFFER_SIZE 256

static esp_timer_handle_t ze08_message_end_timer;

void ze08_check_and_save_data(ze08_sensor_t *sensor)
{
    uint8_t *buf = sensor->buffer;
    uint8_t cksum = 0x3c + 0x04 + buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7] + buf[8];

    if (sensor->debug & 2) {
        ESP_LOGI(sensor->name, "#CNT %d", sensor->cnt);
    }
    if (cksum == buf[9]) {
        sensor->values.ch2o = (uint16_t)buf[0] << 8 | (uint16_t)buf[1];
        sensor->data_cnt++;
        sensor->data_ready = true;
    } else if (sensor->debug & 4) {
        ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, 9, ESP_LOG_ERROR);
        ESP_LOGE(sensor->name, "#CK %02X %02X", cksum, buf[9]);
    }
    sensor->cnt = 0xff;
}

void ze08_message_end_timeout(void *args)
{
    ze08_check_and_save_data((ze08_sensor_t *)args);
}

static void rx_task_ze08_sensor(void *args)
{
    ze08_sensor_t *sensor = (ze08_sensor_t *)args;

    uint8_t rx_pin = sensor->rx_pin;
    uint8_t *buf = sensor->buffer;
    static uint8_t last_byte = 0;

    rmt_uart_config_t uart_config = {
        .baud_rate = 9600,                  // Your baud rate
        .mode = RMT_UART_MODE_RX_ONLY,      // Receiver only
        .data_bits = RMT_UART_DATA_8_BITS,
        .parity = RMT_UART_PARITY_DISABLE,
        .stop_bits = RMT_UART_STOP_BITS_1,
        .rx_io_num = rx_pin,                // Your RX GPIO pin
        .buffer_size = 50                   // RX buffer size (bytes)
    };

    ESP_ERROR_CHECK(rmt_uart_init(0, &uart_config));

    uint8_t rx_buf[32];
    size_t length = 0;

    // Start timeout timer if message end is not detected
    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &ze08_message_end_timeout,
        .arg = args,
        .name = "ze08_message_end_timeout",
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = false
    };
    // Fire timeout after 100ms
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &ze08_message_end_timer));

    while (true) {
        length = rmt_uart_read_bytes(0, rx_buf, sizeof(rx_buf), portMAX_DELAY);
        uint8_t i = 0;
        while (length-- > 0) {
            uint8_t rx_byte = rx_buf[i++];

            if (sensor->debug & 2) {
                ESP_LOGI(sensor->name, "#%d RXB=%02X", sensor->cnt, rx_byte);
            }
            if (sensor->cnt < 12) {
                buf[sensor->cnt++] = rx_byte;
            } else if (last_byte == 0x3c && rx_byte == 0x04) {
                // START sequence received
                sensor->cnt = 0;
                ESP_ERROR_CHECK(esp_timer_start_once(ze08_message_end_timer, 100000));
            }
            last_byte = rx_byte;
        }
        if (sensor->cnt == 10) {
            esp_timer_stop(ze08_message_end_timer);
            //esp_timer_delete(ze08_message_end_timer);
            ze08_check_and_save_data(sensor);
            last_byte = 0xff;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

esp_err_t ze08_init(ze08_sensor_t **sensor, uint8_t rx_pin, uint8_t tx_pin)
{
    ze08_sensor_t *ze08_sensor = calloc(1, sizeof(ze08_sensor_t));

    ESP_LOGI(TAG, "Initialize ZE08-C2HO");
    // YYS Multi Sensor
    ze08_sensor->name = "ZE08-C2HO";
    ze08_sensor->baudrate = 9600;
    ze08_sensor->rx_pin = rx_pin;
    ze08_sensor->buffer = malloc(12);
    ze08_sensor->cnt = 0xff;
    *sensor = ze08_sensor;
    xTaskCreate(rx_task_ze08_sensor, "rx_task_ze08_sensor", 4096, (void *)ze08_sensor, configMAX_PRIORITIES - 1, NULL);
    ESP_LOGI(TAG, "ZE08-C2HO initialized");
    return ESP_OK;
}

bool ze08_data_ready(ze08_sensor_t *sensor)
{
    if (sensor == NULL) return false;
    if (sensor->data_ready) {
        sensor->data_ready = false;
        return true;
    }
    return false;
}

uint16_t ze08_get_ch2o_raw(ze08_sensor_t *sensor)
{
    return sensor->values.ch2o;
}

float ze08_get_ch2o_ppm(ze08_sensor_t *sensor)
{
    return (float)sensor->values.ch2o * 0.001;  // e.g. 6 ppb = 0.006 ppm
}

float ze08_get_ch2o_mg(ze08_sensor_t *sensor)
{
    return (float)sensor->values.ch2o * 0.00125;  // e.g. 6 ppb = 0.0075 mg/m³
}

void ze08_dump_values(ze08_sensor_t *sensor, bool force)
{
    if (force || sensor->debug & 1) {
        ESP_LOGI(TAG, "CH2O=%u ppb = %f ppm  DCNT=%d  ERR=%d",
                 ze08_get_ch2o_raw(sensor), ze08_get_ch2o_ppm(sensor),
                 sensor->data_cnt, sensor->error_cnt);
    }
}
