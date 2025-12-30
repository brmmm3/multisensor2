#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "hw_serial.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "mhz19.h"

static const char *TAG = "MHZ19";

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

static uint8_t cmd_reset[CMD_SIZE] = {MHZ19_RESET, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t cmd_get_fw_version[CMD_SIZE] = {MHZ19_GET_FW_VERSION, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t cmd_get_abc[CMD_SIZE] = {MHZ19_GET_ABC, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t cmd_get_values[CMD_SIZE] = {MHZ19_GET_TEMP_INT, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t cmd_zero_calib[CMD_SIZE] = {MHZ19_ZERO_CALIB, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t cmd_get_range[CMD_SIZE] = {MHZ19_GET_RANGE, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t cmd_set_range[5][6] = {
    {0x99, 0x00, 0x00, 0x00, 0x03, 0xE8},
    {0x99, 0x00, 0x00, 0x00, 0x07, 0xD0},
    {0x99, 0x00, 0x00, 0x00, 0x0B, 0xB8},
    {0x99, 0x00, 0x00, 0x00, 0x13, 0x88},
    {0x99, 0x00, 0x00, 0x00, 0x27, 0x10}};


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

esp_err_t mhz19_write_command(hw_serial_t *serial, uint8_t *cmd, uint8_t cmd_size, uint8_t *response, uint8_t *response_size)
{
    esp_err_t err = uart_write_bytes(serial->uart_num, cmd, cmd_size);
    if (err < 0) return err;
    uint8_t cksum = mhz19_checksum(cmd, cmd_size);
    err = uart_write_bytes(serial->uart_num, &cksum, 1);
    if (err < 0) return err;
    err = uart_flush(serial->uart_num);
    if (err < 0) return err;
    if (response != NULL) {
        *response_size = uart_read_bytes(serial->uart_num, response, RESPONSE_SIZE, 10);
    }
    return ESP_OK;
}

static void rx_task_mhz19_sensor(void *arg)
{
    ESP_LOGI(TAG, "rx_task_co2_sensor STARTED on UART %p", arg);
    mhz19_t *sensor = (mhz19_t *)arg;

    hw_serial_t *serial = sensor->hw_serial;
    uint8_t cmd[CMD_SIZE];
    uint8_t buf[RESPONSE_SIZE];
    uint8_t cmd_byte;
    uint8_t response_cnt;
    bool ready = false;

    ESP_LOGI(TAG, "rx_task_co2_sensor STARTED on UART %d", serial->uart_num);
    while (true) {
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
            mhz19_write_command(serial, buf, REQUEST_SIZE, buf, &response_cnt);
            if (buf[0] == 0xff && buf[1] == cmd_byte && mhz19_checksum(buf, RESPONSE_SIZE - 1) == buf[RESPONSE_SIZE - 1]) {
                switch (cmd_byte) {
                    case MHZ19_SET_ABC:
                    ESP_LOGI(TAG, "MHZ19 SET ABC=%d", buf[2]);
                    break;
                    case MHZ19_GET_TEMP_INT:
                    sensor->co2 = buf[2] * 256 + buf[3];
                    sensor->temp = buf[4] - 40;
                    sensor->status = buf[5];
                    sensor->data_cnt++;
                    break;
                    case MHZ19_GET_FW_VERSION:
                    memcpy(sensor->fw_version, &buf[2], 6);
                    sensor->fw_version[6] = 0;
                    ESP_LOGI(TAG, "MHZ19 FW: %s", sensor->fw_version);
                    break;
                    case MHZ19_SET_RANGE:
                    ESP_LOGI(TAG, "MHZ19 SET RANGE=%d", buf[2]);
                    break;
                    case MHZ19_GET_RANGE:
                    if (buf[4] == 0x03 && buf[5] == 0xe8) sensor->range = MHZ19_RANGE_1000;
                    else if (buf[4] == 0x07 && buf[5] == 0xd0) sensor->range = MHZ19_RANGE_2000;
                    else if (buf[4] == 0x0b && buf[5] == 0xb8) sensor->range = MHZ19_RANGE_3000;
                    else if (buf[4] == 0x13 && buf[5] == 0x88) sensor->range = MHZ19_RANGE_5000;
                    else if (buf[4] == 0x27 && buf[5] == 0x10) sensor->range = MHZ19_RANGE_10000;
                    else sensor->range = MHZ19_RANGE_INVALID;
                    ESP_LOGI(TAG, "MHZ19 RANGE: %d", sensor->range);
                    break;
                    default:
                    ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, response_cnt, ESP_LOG_INFO);
                }
            } else {
                sensor->co2 = -1;
                sensor->temp = -1;
                sensor->status = -1;
                ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, response_cnt, ESP_LOG_INFO);
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

bool mhz19_data_ready(mhz19_t *sensor)
{
    if (sensor->data_cnt > 0) {
        sensor->data_cnt = 0;
        return true;
    }
    return false;
}

esp_err_t mhz19_set_auto_calibration(mhz19_t *sensor, bool auto_calib)
{
    uint8_t cmd[CMD_SIZE] = {MHZ19_SET_ABC, auto_calib ? 0xA0: 0x00, 0x00, 0x00, 0x00, 0x00};
    sensor->auto_calib = auto_calib;
    sensor->pending++;
    if (xQueueSend(sensor->queue, &cmd, 1)) return ESP_OK;
    return ESP_FAIL;
}

esp_err_t mhz19_get_auto_calibration(mhz19_t *sensor)
{
    sensor->pending++;
    if (xQueueSend(sensor->queue, &cmd_get_abc, 1)) return ESP_OK;
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

esp_err_t mhz19_reset(mhz19_t *sensor)
{
    sensor->pending++;
    if (xQueueSend(sensor->queue, &cmd_reset, 1)) return ESP_OK;
    return ESP_FAIL;
}

esp_err_t mhz19_init(mhz19_t **sensor, uint8_t uart_num, uint8_t rx_pin, uint8_t tx_pin)
{
    hw_serial_t *mhz19_serial = malloc(sizeof(hw_serial_t));
    mhz19_t *mhz19_sensor = malloc(sizeof(mhz19_t));

    ESP_LOGI(TAG, "Initialize MHZ19");
    // Serial
    mhz19_serial->uart_num = uart_num;
    mhz19_serial->rx_pin = rx_pin;
    mhz19_serial->tx_pin = tx_pin;
    mhz19_serial->baudrate = 9600;
    mhz19_serial->queue = NULL;
    // Sensor
    mhz19_sensor->name = "CO2";
    mhz19_sensor->queue = xQueueCreate(6, 6);
    mhz19_sensor->pending = 0;
    mhz19_sensor->hw_serial = mhz19_serial;
    mhz19_sensor->co2 = 0xffff;
    mhz19_sensor->temp = 0xff;
    mhz19_sensor->data_cnt = 0;
    mhz19_sensor->status = 0xff;
    mhz19_sensor->range = MHZ19_RANGE_INVALID;
    mhz19_sensor->fw_version[0] = 0;
    mhz19_sensor->debug = 0;
    *sensor = mhz19_sensor;

    uart_init(mhz19_serial->uart_num, mhz19_serial->rx_pin, mhz19_serial->tx_pin);

    // Send initialization sequence
    mhz19_set_auto_calibration(mhz19_sensor, false);
    xQueueSend(mhz19_sensor->queue, &cmd_get_fw_version, 1);
    mhz19_set_range(mhz19_sensor, MHZ19_RANGE_5000);
    xQueueSend(mhz19_sensor->queue, &cmd_get_range, 1);

    xTaskCreate(rx_task_mhz19_sensor, "rx_task_mhz19_sensor", 4096, (void *)mhz19_sensor, configMAX_PRIORITIES - 1, NULL);

    ESP_LOGI(TAG, "MHZ19 initialized");
    return ESP_OK;
}

void mhz19_dump(mhz19_t *sensor)
{
    if (sensor->debug & 1) {
        ESP_LOGI(TAG, "co2=%d ppm  temp=%d °C", sensor->co2, sensor->temp);
    }
}
