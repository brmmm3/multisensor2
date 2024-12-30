#include "esp_timer.h"
#include "driver/gpio.h"
#include "sw_serial.h"

void IRAM_ATTR sw_serial_irq_handler(void *args)
{
    sw_serial_t *serial = (sw_serial_t *)args;
    uint32_t time = (uint32_t)esp_timer_get_time();
    uint8_t level = gpio_get_level(serial->rx_pin);

    serial->cnt++;
    if (serial->bit_cnt == 0) {
        if (level == 0) {
            serial->bit_cnt = 1;
            serial->time = time;
            serial->tmp_val = 0;
        }
        //ESP_DRAM_LOGI(DRAM_STR("I"), "S %d", level);
        return;
    }

    uint32_t dt;

    if (time > serial->time) {
        dt = time - serial->time;
    } else {
        dt = serial->time - time;
    }
    // Sensors are sending with 9600Baud. The duration of 1 bit is 104us.
    //ESP_DRAM_LOGI(DRAM_STR("I"), "%lu c=%d L=%d b=%d v=%02X", dt, serial->cnt, level, serial->bit_cnt, serial->tmp_val);
    if (dt > 937) {
        if (level == 0) {
            serial->bit_cnt = 1;
            serial->time = time;
            serial->tmp_val = 0xff;
        } else {
            serial->bit_cnt = 0;
            serial->tmp_val >>= 10 - serial->bit_cnt;
        }
        xQueueSendFromISR(serial->queue, &serial->tmp_val, NULL);
        //ESP_DRAM_LOGI(DRAM_STR("I"), "S2 %lu %d %02X", dt, level, serial->tmp_val);
        serial->tmp_val = 0;
        return;
    }

    //ESP_DRAM_LOGI(DRAM_STR(""), "%lu %d %d", dt, level, serial->bit_cnt);
    level = (1 - level) << 7;
    while (dt > 0) {
        serial->tmp_val = (serial->tmp_val >> 1) | level;
        serial->bit_cnt += 1;
        if (serial->bit_cnt == 10) {
            if (level == 0) {
                serial->bit_cnt = 0;
            } else {
                serial->bit_cnt = 1;
            }
            xQueueSendFromISR(serial->queue, &serial->tmp_val, NULL);
            //ESP_DRAM_LOGI(DRAM_STR(""), "=%d =%02X", serial->bit_cnt, serial->tmp_val);
            serial->tmp_val = 0;
            break;
        }
        if (dt > 110) {
            dt -= 104;
        } else {
            break;
        }
    }
    //ESP_DRAM_LOGI(DRAM_STR("I"), "%lu b=%d V=%02X", dt, serial->bit_cnt, serial->tmp_val);
    serial->time = time;
}
