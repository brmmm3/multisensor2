#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "sw_serial.h"

#define TAG "SW"

esp_timer_handle_t sw_serial_byte_end_timer;

void sw_serial_byte_end_timeout(void *args)
{
    sw_serial_t *serial = (sw_serial_t *)args;
    uint8_t level = gpio_get_level(serial->rx_pin);

    if (level == 1) {
        // STOP bit must be 1.
        if (serial->debug & 16) {
            ESP_LOGI(TAG, "to %d", serial->bit_cnt);
        }
        while (serial->bit_cnt < 10) {
            serial->tmp_val = (serial->tmp_val >> 1) | level;
            serial->bit_cnt += 1;
        }
        xQueueSend(serial->queue, &serial->tmp_val, 1);
        if (serial->debug & 8) {
            ESP_LOGI(TAG, "TO %02X", serial->tmp_val);
        }
    } else if (serial->debug & 8) {
        ESP_LOGE(TAG, "TO %d %02X", serial->bit_cnt, serial->tmp_val);
    }
    serial->bit_cnt = 0;
}

void sw_serial_irq_handler(void *args)
{
    sw_serial_t *serial = (sw_serial_t *)args;
    uint32_t time = (uint32_t)esp_timer_get_time();
    uint8_t level = gpio_get_level(serial->rx_pin);

    serial->cnt++;
    if (serial->bit_cnt == 0) {
        if (level == 0) {
            // START bit detected
            serial->time = time;
            serial->bit_cnt = 1;
            serial->tmp_val = 0;
            // Start timeout timer if byte end is not detected by an edge trigger
            // Detect byte end with after a timeout. This fires if at least the last bit is high.
            ESP_ERROR_CHECK(esp_timer_start_once(sw_serial_byte_end_timer, 1000));
        }
        if (serial->debug & 1) {
            ESP_DRAM_LOGI(DRAM_STR(""), "S %d", level);
        }
        return;
    }

    uint32_t dt;

    if (time > serial->time) {
        dt = time - serial->time;
    } else {
        dt = serial->time - time;
    }

    if (serial->debug & 2) {
        ESP_DRAM_LOGI(DRAM_STR(""), "%lu %d %d", dt, serial->bit_cnt, level);
    }
    // Interrupt is edge triggered. So the bits of the current byte are inverse to the curret IO level.
    level = (1 - level) << 7;
    while (dt >= 100) {
        serial->tmp_val = (serial->tmp_val >> 1) | level;
        serial->bit_cnt += 1;
        // 8N1 -> 10 bits
        if (serial->bit_cnt == 10) {
            esp_timer_stop(sw_serial_byte_end_timer);
            //esp_timer_delete(sw_serial_byte_end_timer);
            xQueueSendFromISR(serial->queue, &serial->tmp_val, NULL);
            if (serial->debug & 4) {
                ESP_DRAM_LOGI(DRAM_STR(""), "E %02X", serial->tmp_val);
            }
            serial->bit_cnt = 0;
            return;
        }
        dt -= 100;
    }
    serial->time = time;
    if (serial->debug & 4) {
        ESP_DRAM_LOGI(DRAM_STR(""), "*%lu %d %02X", dt, serial->bit_cnt, serial->tmp_val);
    }
}

void sw_serial_init(void *args)
{
    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &sw_serial_byte_end_timeout,
        .arg = args,
        .name = "sw_serial_byte_end_timer",
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = false
    };
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &sw_serial_byte_end_timer));
}
