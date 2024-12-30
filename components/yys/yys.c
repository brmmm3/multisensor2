#include "rom/gpio.h"
#include "driver/gpio.h"

#include "yys.h"

TaskHandle_t uartRxO2Handle = NULL;
TaskHandle_t uartRxCOHandle = NULL;
TaskHandle_t uartRxH2SHandle = NULL;

static void rx_task_yys_sensor(void *arg)
{
    yys_sensor_t *sensor = (yys_sensor_t *)arg;

    if (sensor->sw_serial != NULL) {
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
                    uint16_t data = (uint16_t)buf[2] << 8 | (uint16_t)buf[3];

                    xQueueSend(sensor->queue, &data, portMAX_DELAY);
                } else {
                    ESP_LOGE(sensor->name, "#CK %02X", cksum);
                }
                sensor->cnt = 0;
            }
        }
    } else {
        // Sensor is connected to a HW serial interface
        hw_serial_t *serial = sensor->hw_serial;
        uint8_t *buf = sensor->buffer;

        uart_init(serial->uart_num, serial->rx_pin);
        while (1) {
            const int rxBytes = uart_read_bytes(serial->uart_num, buf, UART_BUFFER_SIZE, 1);
            /*if (rxBytes > 0) {
                ESP_LOGI(sensor->name, "#RXB=%d", rxBytes);
            }*/
            if (rxBytes > 0 && buf[0] == 0xff && buf[1] == 0x86) {
                //ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, rxBytes, ESP_LOG_INFO);
                uint8_t cksum = ~(buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7]);
                if (cksum == buf[8]) {
                    uint16_t data = (uint16_t)buf[2] << 8 | (uint16_t)buf[3];

                    xQueueSend(sensor->queue, &data, portMAX_DELAY);
                } else {
                    ESP_LOGE(sensor->name, "#CK=%02X", cksum);
                }
            }
        }
    }
}

void yys_init()
{
    sw_serial_t o2_serial = {
        .rx_pin = 13,
        .baudrate = 9600,
        .time = 0,
        .tmp_val = 0,
        .bit_cnt = 0,
        .queue = xQueueCreate(32, 1),
        .cnt = 0
    };
    yys_sensor_t o2_sensor = {
        .name = "O2",
        .buffer = malloc(16),
        .cnt = 0,
        .queue = xQueueCreate(16, 2),
        .hw_serial = 0,
        .sw_serial = &o2_serial
    };
    xTaskCreate(rx_task_yys_sensor, "rx_task_yys_sensor_O2", 4096, (void *)&o2_sensor, configMAX_PRIORITIES - 1, &uartRxO2Handle);

    sw_serial_t co_serial = {
        .rx_pin = 11,
        .baudrate = 9600,
        .time = 0,
        .tmp_val = 0,
        .bit_cnt = 0,
        .queue = xQueueCreate(32, 1),
        .cnt = 0
    };
    yys_sensor_t co_sensor = {
        .name = "CO",
        .buffer = malloc(16),
        .cnt = 0,
        .queue = xQueueCreate(16, 2),
        .hw_serial = 0,
        .sw_serial = &co_serial
    };
    xTaskCreate(rx_task_yys_sensor, "rx_task_yys_sensor_CO", 4096, (void *)&co_sensor, configMAX_PRIORITIES - 1, &uartRxCOHandle);

    sw_serial_t h2s_serial = {
        .rx_pin = 12,
        .baudrate = 9600,
        .time = 0,
        .tmp_val = 0,
        .bit_cnt = 0,
        .queue = xQueueCreate(32, 1),
        .cnt = 0
    };
    yys_sensor_t h2s_sensor = {
        .name = "H2S",
        .buffer = malloc(16),
        .cnt = 0,
        .queue = xQueueCreate(16, 2),
        .hw_serial = 0,
        .sw_serial = &h2s_serial
    };
    xTaskCreate(rx_task_yys_sensor, "rx_task_yys_sensor_H2S", 4096, (void *)&h2s_sensor, configMAX_PRIORITIES - 1, &uartRxH2SHandle);
}