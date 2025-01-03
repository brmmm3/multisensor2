#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "hw_serial.h"

// UART
#define UART_BUFFER_SIZE 256

static const char *HW_SERIAL_TAG = "HWS";


void uart_init(uint8_t uart_num, int rx_pin, int tx_pin)
{
    ESP_LOGI(HW_SERIAL_TAG, "Initialize UART %d on rx=%d tx=%d", uart_num, rx_pin, tx_pin);

    uint8_t source_clk = UART_SCLK_DEFAULT;
    if (uart_num == LP_UART_NUM_0)
    {
        source_clk = LP_UART_SCLK_DEFAULT;
    }
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = source_clk,
    };
    int intr_alloc_flags = ESP_INTR_FLAG_IRAM;

    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUFFER_SIZE << 1, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, -1, -1));
    ESP_LOGI(HW_SERIAL_TAG, "UART %d initialized", uart_num);
}
