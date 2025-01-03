#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "mhz19.h"

static const char *MHZ19_TAG = "MHZ19";

// [MHZ19] Changes operation mode and performs MCU reset
#define MHZ19_RESET          0x78
// [MHZ19] ABC (Automatic Baseline Correction) Mode ON/OFF - Turns ABC logic on or off (b[3] == 0xA0 - on, 0x00 - off)
#define MHZ19_SET_ABC        0x79
// [MHZ19] Get ABC logic status  (1 - enabled, 0 - disabled)
#define MHZ19_GET_ABC        0x7D
// [MHZ19] Get Raw CO2
#define MHZ19_GET_RAW_CO2    0x84
// [MHZ19] Get Temperature float, CO2 Unlimited
#define MHZ19_GET_TEMP_FLOAT 0x85
// [MHZ19] Get Temperature integer, CO2 limited / clipped
#define MHZ19_GET_TEMP_INT   0x86
// [MHZ19] Zero Calibration
#define MHZ19_ZERO_CALIB     0x87
// [MHZ19] Span Calibration
#define MHZ19_SPAN_CALIB     0x88
// [MHZ19] Set Range
#define MHZ19_SET_RANGE      0x99
// [MHZ19] Get Range
#define MHZ19_GET_RANGE      0x9B
// [MHZ19] Get Background CO2
#define MHZ19_GET_BG_CO2     0x9C
// [MHZ19] Get Firmware Version
#define MHZ19_GET_FW_VERSION 0xA0
// [MHZ19] Get Last Response
#define MHZ19_GET_LAST_RESP  0xA2
// [MHZ19] Get Temperature Calibration
#define MHZ19_GET_TEMP_CALIB 0xA3

#define CMD_SIZE      6
#define REQUEST_SIZE  8
#define RESPONSE_SIZE 9

uint8_t cmd_reset[CMD_SIZE] = {MHZ19_RESET, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t cmd_get_fw_version[CMD_SIZE] = {MHZ19_GET_FW_VERSION, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t cmd_get_abc[CMD_SIZE] = {MHZ19_GET_ABC, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t cmd_get_values[CMD_SIZE] = {MHZ19_GET_TEMP_INT, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t cmd_zero_calib[CMD_SIZE] = {MHZ19_ZERO_CALIB, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t cmd_get_range[CMD_SIZE] = {MHZ19_GET_RANGE, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t cmd_set_range[5][6] = {
    {0x99, 0x00, 0x00, 0x00, 0x03, 0xE8},
    {0x99, 0x00, 0x00, 0x00, 0x07, 0xD0},
    {0x99, 0x00, 0x00, 0x00, 0x0B, 0xB8},
    {0x99, 0x00, 0x00, 0x00, 0x13, 0x88},
    {0x99, 0x00, 0x00, 0x00, 0x27, 0x10}};

hw_serial_t co2_serial = {
    .uart_num = LP_UART_NUM_0,
    .rx_pin = 4,
    .tx_pin = 5,
    .baudrate = 9600,
    .queue = NULL
};

mhz19_t co2_sensor = {
    .name = "CO2",
    .queue = NULL,
    .pending = 0,
    .task = NULL,
    .hw_serial = &co2_serial,
    .co2 = 0xffff,
    .temp = 0xff,
    .status = 0xff,
    .range = MHZ19_RANGE_INVALID,
    .fw_version = {0, 0, 0, 0, 0, 0, 0}

};

uint8_t mhz19_checksum(uint8_t *data, uint8_t size)
{
	uint8_t sum = 0x00;
	for (int i = 1; i < size; i++)
	{
		sum += data[i];
	}
	sum = 0xff - sum + 0x01;
	return sum;
}

esp_err_t mhz19_write_command(uint8_t *cmd, uint8_t cmd_size, uint8_t *response, uint8_t *response_size)
{
    esp_err_t err = uart_write_bytes(co2_serial.uart_num, cmd, cmd_size);
    if (err < 0) return err;
    uint8_t cksum = mhz19_checksum(cmd, cmd_size);
    err = uart_write_bytes(co2_serial.uart_num, &cksum, 1);
    if (err < 0) return err;
    err = uart_flush(co2_serial.uart_num);
    if (err < 0) return err;
    if (response != NULL) {
        *response_size = uart_read_bytes(co2_serial.uart_num, response, RESPONSE_SIZE, 10);
    }
    return ESP_OK;
}

static void rx_task_co2_sensor(void *arg)
{
    mhz19_t *sensor = (mhz19_t *)arg;

    hw_serial_t *serial = sensor->hw_serial;
    uint8_t cmd[CMD_SIZE];
    uint8_t buf[RESPONSE_SIZE];
    uint8_t cmd_byte;
    uint8_t response_cnt;
    bool ready = false;

    ESP_LOGI(MHZ19_TAG, "rx_task_co2_sensor STARTED on UART %d", serial->uart_num);
    while (1) {
        if (ready) {
            buf[0] = 0xff;
            buf[1] = 0x01;
            buf[CMD_SIZE + 2] = 0;
            if (xQueueReceive(sensor->queue, &cmd, 100)) {
                memcpy(&buf[2], cmd, CMD_SIZE);
            } else {
                memcpy(&buf[2], &cmd_get_values, CMD_SIZE);
            }
            cmd_byte = buf[2];
            mhz19_write_command(buf, REQUEST_SIZE, buf, &response_cnt);
            ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, response_cnt, ESP_LOG_INFO);
            if (buf[0] == 0xff && buf[1] == cmd_byte && mhz19_checksum(buf, RESPONSE_SIZE - 1) == buf[RESPONSE_SIZE - 1]) {
                switch (cmd_byte) {
                    case MHZ19_GET_TEMP_INT:
                    sensor->co2 = buf[2] * 256 + buf[3];
                    sensor->temp = buf[4] - 40;
                    sensor->status = buf[5];
                    ESP_LOGI(MHZ19_TAG, "MHZ19 CO2=%d temp=%d status=%d", sensor->co2, sensor->temp, sensor->status);
                    break;
                    case MHZ19_GET_FW_VERSION:
                    memcpy(sensor->fw_version, &buf[2], 6);
                    sensor->fw_version[6] = 0;
                    ESP_LOGI(MHZ19_TAG, "MHZ19 FW: %s", sensor->fw_version);
                    break;
                    case MHZ19_GET_RANGE:
                    if (buf[4] == 0x03 && buf[5] == 0xe8) sensor->range = MHZ19_RANGE_1000;
                    else if (buf[4] == 0x07 && buf[5] == 0xd0) sensor->range = MHZ19_RANGE_2000;
                    else if (buf[4] == 0x0b && buf[5] == 0xb8) sensor->range = MHZ19_RANGE_3000;
                    else if (buf[4] == 0x13 && buf[5] == 0x88) sensor->range = MHZ19_RANGE_5000;
                    else if (buf[4] == 0x27 && buf[5] == 0x10) sensor->range = MHZ19_RANGE_5000;
                    else sensor->range = MHZ19_RANGE_INVALID;
                    ESP_LOGI(MHZ19_TAG, "MHZ19 RANGE: %d", sensor->range);
                    break;
                }
            } else {
                sensor->co2 = -1;
                sensor->temp = -1;
                sensor->status = -1;
            }
            if (sensor->pending > 0) sensor->pending--;
        } else {
            response_cnt = uart_read_bytes(serial->uart_num, buf, sizeof(buf), 100);
            if (response_cnt == 0) {
                ready = true;
                continue;
            }
            ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, response_cnt, ESP_LOG_INFO);
        }
    }
}

uint8_t mhz19_pending(mhz19_t *sensor)
{
    return sensor->pending;
}

esp_err_t mhz19_set_auto_calibration(mhz19_t *sensor, bool auto_calib)
{
    uint8_t cmd[CMD_SIZE] = {MHZ19_SET_ABC, auto_calib ? 0xA0: 0x00, 0x00, 0x00, 0x00, 0x00};
    sensor->auto_calib = auto_calib;
    sensor->pending++;
    if (xQueueSend(sensor->queue, &cmd, 1)) return ESP_OK;
    return ESP_FAIL;
}

esp_err_t mhz19_calibrate_zero(mhz19_t *sensor)
{
    sensor->pending++;
    if (xQueueSend(sensor->queue, &cmd_zero_calib, 1)) return ESP_OK;
    return ESP_FAIL;
}

esp_err_t mhz19_calibrate_span(mhz19_t *sensor, uint16_t ppm)
{
	if (ppm < 1000) return ESP_FAIL;
    uint8_t cmd[CMD_SIZE] = {MHZ19_SPAN_CALIB, 0x00, 0x00, (uint8_t)(ppm >> 8), (uint8_t)ppm, 0x00};
    sensor->pending++;
    if (xQueueSend(sensor->queue, &cmd, 1)) return ESP_OK;
    return ESP_FAIL;
}

enum MHZ19_RANGE mhz19_get_range(mhz19_t *sensor)
{
    sensor->pending++;
    if (!xQueueSend(sensor->queue, &cmd_get_range, 1)) return MHZ19_RANGE_INVALID;
    return sensor->range;
}

esp_err_t mhz19_set_range(mhz19_t *sensor, enum MHZ19_RANGE range)
{
    sensor->pending++;
    if (xQueueSend(sensor->queue, &cmd_set_range[range], 1)) return ESP_OK;
    return ESP_FAIL;
}

esp_err_t mhz19_init(mhz19_t **sensor)
{
    ESP_LOGI(MHZ19_TAG, "Initialize MHZ19");
    // Command queue
    co2_sensor.queue = xQueueCreate(6, 6);

    uart_init(co2_serial.uart_num, co2_serial.rx_pin, co2_serial.tx_pin);

    // Send initialization sequence
    mhz19_set_auto_calibration(&co2_sensor, false);
    xQueueSend(co2_sensor.queue, &cmd_get_fw_version, 1);
    mhz19_set_range(&co2_sensor, MHZ19_RANGE_5000);
    xQueueSend(co2_sensor.queue, &cmd_get_range, 1);

    xTaskCreate(rx_task_co2_sensor, "rx_task_co2_sensor", 4096, (void *)&co2_sensor, configMAX_PRIORITIES - 1, co2_sensor.task);
    *sensor = &co2_sensor;
    ESP_LOGI(MHZ19_TAG, "MHZ19 initialized");
    return ESP_OK;
}
