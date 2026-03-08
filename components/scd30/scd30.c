/* MIT License
*
* Copyright (c) 2026 Martin Bammer
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>

#include "scd30.h"

static const char *TAG = "SCD30";

#define SCD30_SENSOR_ADDR 0x61

// C2 00 10 PP pp CRC => PPpp = Pressure for compensation (700..1400). 0=deactivate compensation
static uint8_t start_periodic_measurement[]             = {0x00, 0x10};
// C2 01 04 => No CRC
static uint8_t stop_periodic_measurement[]              = {0x01, 0x04};
// C2 46 00 | C3 II ii CRC => IIii = Returned interval
static uint8_t get_measurement_interval[]               = {0x46, 0x00};
// C2 46 00 II ii CRC => IIii = Interval in seconds 2..1800
static uint8_t set_measurement_interval[]               = {0x46, 0x00};
// C2 02 02 | C3 RR rr CRC => Ready status
static uint8_t get_data_ready_status[]                  = {0x02, 0x02};
// C2 03 00 | C3 MCC mcc CRC LCC lcc CRC MTT mtt CRC LTT ltt CRC MRH mrh CRC LRH lrh CRC
static uint8_t read_measurement[]                       = {0x03, 0x00};
// C2 53 06 CRC C3 ASC asc CRC
static uint8_t get_automatic_self_calibration_enabled[] = {0x53, 0x06};
// C2 53 06 ASC asc CRC
static uint8_t set_automatic_self_calibration_enabled[] = {0x53, 0x06};
// C2 52 04 C3 FRC frc CRC
static uint8_t get_forced_recalibration_value[]         = {0x52, 0x04};
// C2 52 04 FRC frc CRC
static uint8_t perform_forced_recalibration[]           = {0x52, 0x04};
// C2 54 03 C3 SHT sht CRC
static uint8_t get_temperature_offset[]                 = {0x54, 0x03};
// C2 54 03 SHT sht CRC
static uint8_t set_temperature_offset[]                 = {0x54, 0x03};
// C2 51 02 C3 ALT alt CRC
static uint8_t get_sensor_altitude[]                    = {0x51, 0x02};
// C2 51 02 ALT alt CRC
static uint8_t set_sensor_altitude[]                    = {0x51, 0x02};
// C2 D1 00 C3 FW fw CRC
static uint8_t get_firmware_version[]                   = {0xd1, 0x00};
// C2 D3 04
static uint8_t soft_reset[]                             = {0xd3, 0x04};


static uint8_t crc8(const uint8_t *data, size_t count)
{
    uint8_t res = 0xff;

    for (size_t i = 0; i < count; ++i)
    {
        res ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (res & 0x80)
                res = (res << 1) ^ 0x31;
            else
                res = (res << 1);
        }
    }
    return res;
}

static esp_err_t execute_cmd(scd30_t *sensor, uint8_t *cmd, uint32_t timeout_ms,
                             uint16_t out_data, size_t out_words, uint16_t *in_data, size_t in_words)
{
    uint8_t buf[19];
    uint8_t pos;
    esp_err_t err;

    buf[0] = cmd[0];
    buf[1] = cmd[1];
    if (out_words) {
        buf[2] = out_data >> 8;
        buf[3] = out_data;
        buf[4] = crc8(&buf[2], 2);
    }
    err = i2c_master_transmit(sensor->dev_handle, buf, 2 + 3 * out_words, timeout_ms);
    if (err != ESP_OK) return err;
    if (in_words == 0) return ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(5));
    err = i2c_master_receive(sensor->dev_handle, buf, 3 * in_words, timeout_ms);
    if (err != ESP_OK) return err;
    pos = 0;
    while (in_words-- > 0) {
        if (crc8(&buf[pos], 2) != buf[pos + 2]) return ESP_FAIL;
        *in_data = (uint16_t)buf[pos] << 8 | (uint16_t)buf[pos + 1];
        pos += 3;
        in_data++;
    }
    return ESP_OK;
}

scd30_t *scd30_create_master(i2c_master_bus_handle_t bus_handle)
{
    scd30_t *sensor = pvPortMalloc(sizeof(scd30_t));
    memset(sensor, 0, sizeof(scd30_t));

    if (sensor != NULL) {
        sensor->bus_handle = bus_handle;
        sensor->dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for SCD30.");
        scd30_close(sensor);
        return NULL;
    }
    return sensor;
}

esp_err_t scd30_device_create(scd30_t *sensor)
{
    ESP_LOGI(TAG, "device_create for SCD30 sensors on ADDR 0x%X", SCD30_SENSOR_ADDR);
    sensor->dev_config.device_address = SCD30_SENSOR_ADDR;
    sensor->dev_config.scl_speed_hz = CONFIG_SCD30_I2C_CLK_SPEED_HZ;
    sensor->dev_config.flags.disable_ack_check = true;
    // Add device to the I2C bus
    esp_err_t err = i2c_master_bus_add_device(sensor->bus_handle, &sensor->dev_config, &sensor->dev_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "device_create success on 0x%X", SCD30_SENSOR_ADDR);
        return err;
    }
    ESP_LOGE(TAG, "device_create error on 0x%X", SCD30_SENSOR_ADDR);
    return err;
}

esp_err_t scd30_probe(scd30_t *sensor)
{
    esp_err_t err = ESP_OK;
    int i;

    ESP_LOGI(TAG, "Probing for SCD30 sensor on I2C %X", sensor->dev_config.device_address);
    for (i = 0; i < 10; i++) {
        err = i2c_master_probe(sensor->bus_handle, sensor->dev_config.device_address, CONFIG_SCD30_TIMEOUT);
        if (err == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (err == ESP_OK) ESP_LOGI(TAG, "Probing for SCD30 SUCCESS after %u retries", i);
    else ESP_LOGE(TAG, "Probing for SCD30 FAILED with err=%d", err);
    return err;
}

esp_err_t scd30_init(scd30_t **sensor_ptr, i2c_master_bus_handle_t bus_handle)
{
    scd30_t *sensor = NULL;
    esp_err_t err;

    ESP_LOGI(TAG, "Initialize SCD30");
    sensor = scd30_create_master(bus_handle);
    if (sensor == NULL) { 
        ESP_LOGE(TAG, "Could not create SCD30 driver.");
        return ESP_FAIL;
    }
    *sensor_ptr = sensor;
    if ((err = scd30_device_create(sensor)) != ESP_OK) return err;

    ESP_ERROR_CHECK_WITHOUT_ABORT(scd30_probe(sensor));
    ESP_ERROR_CHECK_WITHOUT_ABORT(scd30_soft_reset(sensor));
    sensor->fw_version = scd30_get_firmware_version(sensor);
    ESP_LOGI(TAG, "Firmware version=%d", sensor->fw_version);
    ESP_LOGI(TAG, "AutoCal=%d", scd30_get_automatic_self_calibration(sensor));
    ESP_ERROR_CHECK_WITHOUT_ABORT(scd30_set_automatic_self_calibration(sensor, false));
    ESP_ERROR_CHECK_WITHOUT_ABORT(scd30_start_continuous_measurement(sensor, 0));
    return err;
}

esp_err_t scd30_soft_reset(scd30_t *sensor)
{
    sensor->last_error = execute_cmd(sensor, soft_reset, CONFIG_SCD30_TIMEOUT, 0, 0, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    return sensor->last_error;
}

void scd30_close(scd30_t *sensor)
{
    if (sensor != NULL && sensor->dev_handle != NULL)
        i2c_master_bus_rm_device(sensor->dev_handle);
    vPortFree(sensor);
}

esp_err_t scd30_start_continuous_measurement(scd30_t *sensor, uint16_t p_comp)
{
    sensor->last_error = execute_cmd(sensor, start_periodic_measurement, CONFIG_SCD30_TIMEOUT, p_comp, 1, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    return sensor->last_error;
}

esp_err_t scd30_stop_continuous_measurement(scd30_t *sensor)
{
    sensor->last_error = execute_cmd(sensor, stop_periodic_measurement, CONFIG_SCD30_TIMEOUT, 0, 0, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    return sensor->last_error;
}

uint16_t scd30_get_measurement_interval(scd30_t *sensor)
{
    uint16_t interval_seconds;

    sensor->last_error = execute_cmd(sensor, get_measurement_interval, CONFIG_SCD30_TIMEOUT, 0, 0, &interval_seconds, 1);
    if (sensor->last_error == ESP_OK) return interval_seconds;
    return 0;
}

esp_err_t scd30_set_measurement_interval(scd30_t *sensor, uint16_t interval_seconds)
{
    sensor->last_error = execute_cmd(sensor, set_measurement_interval, CONFIG_SCD30_TIMEOUT, interval_seconds, 1, NULL, 0);
    return sensor->last_error;
}

bool scd30_get_data_ready_status(scd30_t *sensor)
{
    uint16_t status;

    sensor->last_error = execute_cmd(sensor, get_data_ready_status, CONFIG_SCD30_TIMEOUT, 0, 0, &status, 1);
    if (sensor->last_error == ESP_OK) return status != 0;
    return false;
}

esp_err_t scd30_read_measurement(scd30_t *sensor)
{
    union
    {
        uint32_t u32;
        float f;
    } tmp;
    uint16_t buf[6];

    sensor->last_error = execute_cmd(sensor, read_measurement, CONFIG_SCD30_TIMEOUT, 0, 0, buf, 6);
    if (sensor->last_error != ESP_OK) return sensor->last_error;
    tmp.u32 = ((uint32_t)buf[0] << 16) | (uint32_t)buf[1];
    sensor->values.co2 = (uint16_t)tmp.f;
    tmp.u32 = ((uint32_t)buf[2] << 16) | (uint32_t)buf[3];
    sensor->values.temperature = tmp.f;
    tmp.u32 = ((uint32_t)buf[4] << 16) | (uint32_t)buf[5];
    sensor->values.humidity = tmp.f;
    return ESP_OK;
}

bool scd30_get_automatic_self_calibration(scd30_t *sensor)
{
    uint16_t enabled;

    sensor->last_error = execute_cmd(sensor, get_automatic_self_calibration_enabled, CONFIG_SCD30_TIMEOUT, 0, 0, &enabled, 1);
    if (sensor->last_error == ESP_OK) return enabled != 0;
    return false;
}

esp_err_t scd30_set_automatic_self_calibration(scd30_t *sensor, bool enabled)
{
    sensor->last_error = execute_cmd(sensor, set_automatic_self_calibration_enabled, CONFIG_SCD30_TIMEOUT, enabled, 1, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    return sensor->last_error;
}

uint16_t scd30_get_forced_recalibration_value(scd30_t *sensor)
{
    uint16_t correction_value;

    sensor->last_error = execute_cmd(sensor, get_forced_recalibration_value, CONFIG_SCD30_TIMEOUT, 0, 0, &correction_value, 1);
    if (sensor->last_error == ESP_OK) return correction_value;
    return 0;
}

esp_err_t scd30_set_forced_recalibration_value(scd30_t *sensor, uint16_t target_co2_concentration)
{
    sensor->last_error = execute_cmd(sensor, perform_forced_recalibration, CONFIG_SCD30_TIMEOUT, target_co2_concentration, 1, NULL, 0);
    return sensor->last_error;
}

uint16_t scd30_get_temperature_offset_int(scd30_t *sensor)
{
    uint16_t t_offset;

    sensor->last_error = execute_cmd(sensor, get_temperature_offset, CONFIG_SCD30_TIMEOUT, 0, 0, &t_offset, 1);
    if (sensor->last_error == ESP_OK) return t_offset;
    return 0;
}

float scd30_get_temperature_offset(scd30_t *sensor)
{
    uint16_t t_offset = scd30_get_temperature_offset_int(sensor);

    return (float)t_offset / 100;
}

esp_err_t scd30_set_temperature_offset_ticks(scd30_t *sensor, uint16_t t_offset)
{
    sensor->last_error = execute_cmd(sensor, set_temperature_offset, CONFIG_SCD30_TIMEOUT, t_offset, 1, NULL, 0);
    return sensor->last_error;
}

esp_err_t scd30_set_temperature_offset(scd30_t *sensor, float t_offset)
{
    uint16_t raw = (uint16_t)(t_offset * 100);

    sensor->last_error = scd30_set_temperature_offset_ticks(sensor, raw);
    return sensor->last_error;
}

uint16_t scd30_get_sensor_altitude(scd30_t *sensor)
{
    uint16_t altitude;

    sensor->last_error = execute_cmd(sensor, get_sensor_altitude, CONFIG_SCD30_TIMEOUT, 0, 0, &altitude, 1);
    if (sensor->last_error == ESP_OK) return altitude;
    return 0;
}

esp_err_t scd30_set_sensor_altitude(scd30_t *sensor, uint16_t altitude)
{
    sensor->last_error = execute_cmd(sensor, set_sensor_altitude, CONFIG_SCD30_TIMEOUT, altitude, 1, NULL, 0);
    return sensor->last_error;
}

uint16_t scd30_get_firmware_version(scd30_t *sensor)
{
    uint16_t firmware_version;

    sensor->last_error = execute_cmd(sensor, get_firmware_version, CONFIG_SCD30_TIMEOUT, 0, 0, &firmware_version, 1);
    if (sensor->last_error == ESP_OK) return firmware_version;
    return 0;
}

void scd30_dump_values(scd30_t *sensor, bool force)
{
    if (force || sensor->debug & 1) {
        scd30_values_t *values = &sensor->values;

        ESP_LOGI(TAG, "co2=%.1f ppm  temp=%.1f °C  hum=%.1f %%",
                 values->co2, values->temperature, values->humidity);
    }
}
