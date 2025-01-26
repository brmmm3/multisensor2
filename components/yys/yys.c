#include <string.h>
#include "esp_err.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "yys.h"

static const char *TAG = "YYS";

// UART
#define UART_BUFFER_SIZE 256


static void rx_task_yys_sensor(void *arg)
{
    yys_sensor_t *sensor = (yys_sensor_t *)arg;

    sw_serial_t *serial = sensor->sw_serial;
    uint8_t rx_pin = serial->rx_pin;
    uint8_t *buf = sensor->buffer;
    uint8_t rx_byte;
    static uint8_t last_byte = 0;

    // Sensor is connected to a SW serial interface
    gpio_pad_select_gpio(rx_pin);
    gpio_set_direction(rx_pin, GPIO_MODE_INPUT);
    gpio_pulldown_dis(rx_pin);
    gpio_pullup_en(rx_pin);
    gpio_set_intr_type(rx_pin, GPIO_INTR_ANYEDGE);
    ESP_ERROR_CHECK(gpio_isr_handler_add(sensor->sw_serial->rx_pin, sw_serial_irq_handler, (void *)sensor->sw_serial));
    while (1) {
        if (xQueueReceive(serial->queue, &rx_byte, 1)) {
            // A complete byte has been received
            //ESP_LOGI(sensor->name, "#%d RXB=%02X", sensor->cnt, rxByte);
            if (last_byte == 0xff && rx_byte == 0x86) {
                buf[0] = 0xff;
                sensor->cnt = 1;
            }
            last_byte = rx_byte;
            if (sensor->cnt > 0 || rx_byte == 0xff) {
                buf[sensor->cnt++] = rx_byte;
            }
        } else if (serial->bit_cnt > 0) {
            uint32_t time = (uint32_t)esp_timer_get_time();
            uint32_t dt;

            if (time > serial->time) {
                dt = time - serial->time;
            } else {
                dt = serial->time - time;
            }
            serial->time = time;

            if (dt > 950) {
                // An incomplete byte has been received. Make it complete.
                uint8_t level = gpio_get_level(serial->rx_pin) << 7;

                //ESP_LOGI(sensor->name, "#c=%d dt=%lu bc=%d lvl=%02X", serial->cnt, dt, serial->bit_cnt, level);
                if (level != 0) {
                    uint8_t tmp_val = serial->tmp_val;
                    uint8_t bit_cnt = serial->bit_cnt;

                    while (bit_cnt < 10) {
                        tmp_val = (tmp_val >> 1) | level;
                        bit_cnt++;
                    }
                    //ESP_LOGI(sensor->name, "#%d VAL=%02X", sensor->cnt, tmp_val);
                    buf[sensor->cnt++] = tmp_val;
                }
                serial->bit_cnt = 0;
            }
        }
        if (sensor->cnt == 9) {
            uint8_t cksum = ~(buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7]);

            if (cksum == buf[8]) {
                sensor->value = (uint16_t)buf[2] << 8 | (uint16_t)buf[3];
                sensor->data_cnt++;
            } else if (sensor->debug) {
                ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, 9, ESP_LOG_ERROR);
                ESP_LOGE(sensor->name, "#CK %02X", cksum);
            }
            sensor->cnt = 0;
        }
    }
}

esp_err_t yys_init(yys_sensors_t **sensors, uint8_t o2_pin_num, uint8_t co_pin_num, uint8_t h2s_pin_num)
{
    sw_serial_t *o2_serial = calloc(sizeof(sw_serial_t), 1);
    yys_sensor_t *o2_sensor = calloc(sizeof(yys_sensor_t), 1);
    sw_serial_t *co_serial = calloc(sizeof(sw_serial_t), 1);
    yys_sensor_t *co_sensor = calloc(sizeof(yys_sensor_t), 1);
    sw_serial_t *h2s_serial = calloc(sizeof(sw_serial_t), 1);
    yys_sensor_t *h2s_sensor = calloc(sizeof(yys_sensor_t), 1);

    ESP_LOGI("YYS", "Initialize YYS");
    // O2 Sensor
    o2_serial->rx_pin = o2_pin_num;
    o2_serial->baudrate = 9600;
    o2_serial->queue = xQueueCreate(32, 1);
    o2_sensor->name = "O2";
    o2_sensor->buffer = malloc(16);
    o2_sensor->sw_serial = o2_serial;
    // CO Sensor
    co_serial->rx_pin = co_pin_num;
    co_serial->baudrate = 9600;
    co_serial->queue = xQueueCreate(32, 1);
    co_sensor->name = "CO";
    co_sensor->buffer = malloc(16);
    co_sensor->sw_serial = co_serial;
    // H2S Sensor
    h2s_serial->rx_pin = h2s_pin_num;
    h2s_serial->baudrate = 9600;
    h2s_serial->queue = xQueueCreate(32, 1);
    h2s_sensor->name = "H2S";
    h2s_sensor->buffer = malloc(16);
    h2s_sensor->sw_serial = h2s_serial;
    // Sensors
    *sensors = malloc(sizeof(yys_sensors_t));
    (*sensors)->o2_sensor = o2_sensor;
    (*sensors)->co_sensor = co_sensor;
    (*sensors)->h2s_sensor = h2s_sensor;
    xTaskCreate(rx_task_yys_sensor, "rx_task_yys_sensor_O2", 4096, (void *)o2_sensor, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(rx_task_yys_sensor, "rx_task_yys_sensor_CO", 4096, (void *)co_sensor, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(rx_task_yys_sensor, "rx_task_yys_sensor_H2S", 4096, (void *)h2s_sensor, configMAX_PRIORITIES - 1, NULL);
    ESP_LOGI("YYS", "YYS initialized");
    return ESP_OK;
}

bool yys_data_ready(yys_sensors_t *sensor)
{
    if (sensor->co_sensor->data_cnt > 0) {
        sensor->co_sensor->data_cnt = 0;
        return true;
    }
    if (sensor->o2_sensor->data_cnt > 0) {
        sensor->o2_sensor->data_cnt = 0;
        return true;
    }
    if (sensor->h2s_sensor->data_cnt > 0) {
        sensor->h2s_sensor->data_cnt = 0;
        return true;
    }
    return false;
}

uint16_t yys_get_co_raw(yys_sensors_t *sensor)
{
    return sensor->co_sensor->value;
}

uint16_t yys_get_o2_raw(yys_sensors_t *sensor)
{
    return sensor->o2_sensor->value;
}

uint16_t yys_get_h2s_raw(yys_sensors_t *sensor)
{
    return sensor->h2s_sensor->value;
}

float yys_get_co(yys_sensors_t *sensor)
{
    return (float)sensor->co_sensor->value;  // e.g. 6 = 6ppm
}

float yys_get_o2(yys_sensors_t *sensor)
{
    return (float)sensor->o2_sensor->value * 0.1;  // e.g. 209 = 20.9%
}

float yys_get_h2s(yys_sensors_t *sensor)
{
    return (float)sensor->h2s_sensor->value * 0.1;  // e.g. 672 = 67.2ppm
}

void yys_dump(yys_sensors_t *sensors)
{
    if (sensors->o2_sensor->debug & 1) {
        ESP_LOGI(TAG, "O2(err=%d)=%f %%  CO(err=%d)=%f ppm  H2S(err=%d)=%f ppm",
                 sensors->o2_sensor->error_cnt, yys_get_o2(sensors),
                 sensors->co_sensor->error_cnt, yys_get_co(sensors),
                 sensors->h2s_sensor->error_cnt, yys_get_h2s(sensors));
    }
}
