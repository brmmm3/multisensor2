#include "rom/gpio.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "yys.h"

// UART
#define UART_BUFFER_SIZE 256

sw_serial_t o2_serial = {
    .rx_pin = 13,
    .baudrate = 9600,
    .time = 0,
    .tmp_val = 0,
    .bit_cnt = 0,
    .queue = NULL,
    .cnt = 0
};

yys_sensor_t o2_sensor = {
    .name = "O2",
    .buffer = NULL,
    .cnt = 0,
    .sw_serial = &o2_serial
};

sw_serial_t co_serial = {
    .rx_pin = 11,
    .baudrate = 9600,
    .time = 0,
    .tmp_val = 0,
    .bit_cnt = 0,
    .queue = NULL,
    .cnt = 0
};

yys_sensor_t co_sensor = {
    .name = "CO",
    .buffer = NULL,
    .cnt = 0,
    .sw_serial = &co_serial
};

sw_serial_t h2s_serial = {
    .rx_pin = 12,
    .baudrate = 9600,
    .time = 0,
    .tmp_val = 0,
    .bit_cnt = 0,
    .queue = NULL,
    .cnt = 0
};

yys_sensor_t h2s_sensor = {
    .name = "H2S",
    .buffer = NULL,
    .cnt = 0,
    .sw_serial = &h2s_serial
};

yys_sensors_t sensors = {
    .co_sensor = &co_sensor,
    .o2_sensor = &o2_sensor,
    .h2s_sensor = &h2s_sensor
};

static void rx_task_yys_sensor(void *arg)
{
    yys_sensor_t *sensor = (yys_sensor_t *)arg;

    sw_serial_t *serial = sensor->sw_serial;
    uint8_t rx_pin = serial->rx_pin;
    uint8_t *buf = sensor->buffer;

    // Sensor is connected to a SW serial interface
    gpio_pad_select_gpio(rx_pin);
    gpio_set_direction(rx_pin, GPIO_MODE_INPUT);
    gpio_pulldown_dis(rx_pin);
    gpio_pullup_en(rx_pin);
    gpio_set_intr_type(rx_pin, GPIO_INTR_ANYEDGE);

    ESP_ERROR_CHECK(gpio_isr_handler_add(sensor->sw_serial->rx_pin, sw_serial_irq_handler, (void *)sensor->sw_serial));

    uint8_t rxByte;

    while (1) {
        if (xQueueReceive(serial->queue, &rxByte, 1)) {
            // A complete byte has been received
            //ESP_LOGI(sensor->name, "#%d RXB=%02X", sensor->cnt, rxByte);
            buf[sensor->cnt++] = rxByte;
        } else if (serial->bit_cnt > 0) {
            uint32_t time = (uint32_t)esp_timer_get_time();
            uint32_t dt;

            if (time > serial->time) {
                dt = time - serial->time;
            } else {
                dt = serial->time - time;
            }

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

            //ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, 9, ESP_LOG_INFO);
            if (cksum == buf[8]) {
                sensor->value = (uint16_t)buf[2] << 8 | (uint16_t)buf[3];
            } else {
                ESP_LOGE(sensor->name, "#CK %02X", cksum);
            }
            sensor->cnt = 0;
        }
    }
}

yys_sensors_t yys_init(uint8_t o2_pin_num, uint8_t co_pin_num, uint8_t h2s_pin_num)
{
    ESP_LOGI("YYS", "Initialize YYS");
    o2_serial.rx_pin = o2_pin_num;
    o2_serial.queue = xQueueCreate(32, 1);
    o2_sensor.buffer = malloc(16);
    xTaskCreate(rx_task_yys_sensor, "rx_task_yys_sensor_O2", 4096, (void *)&o2_sensor, configMAX_PRIORITIES - 1, NULL);

    co_serial.rx_pin = co_pin_num;
    co_serial.queue = xQueueCreate(32, 1);
    co_sensor.buffer = malloc(16);
    xTaskCreate(rx_task_yys_sensor, "rx_task_yys_sensor_CO", 4096, (void *)&co_sensor, configMAX_PRIORITIES - 1, NULL);

    h2s_serial.rx_pin = h2s_pin_num;
    h2s_serial.queue = xQueueCreate(32, 1);
    h2s_sensor.buffer = malloc(16);
    xTaskCreate(rx_task_yys_sensor, "rx_task_yys_sensor_H2S", 4096, (void *)&h2s_sensor, configMAX_PRIORITIES - 1, NULL);
    ESP_LOGI("YYS", "YYS initialized");
    return sensors;
}

uint16_t yys_get_co(yys_sensors_t *sensor)
{
    return sensor->co_sensor->value;
}

uint16_t yys_get_o2(yys_sensors_t *sensor)
{
    return sensor->o2_sensor->value;
}

uint16_t yys_get_h2s(yys_sensors_t *sensor)
{
    return sensor->h2s_sensor->value;
}
