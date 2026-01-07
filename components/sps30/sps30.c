/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>
#include "esp_log_level.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "sps30.h"
#include "util.h"

static const char *TAG = "SPS30";

#define SPS30_I2C_ADDRESS       0x69
#define SPS30_ADDR_SIZE         (0x02)

static uint8_t cmd_start_measurement[]    = {0x00, 0x10};
static uint8_t cmd_stop_measurement[]     = {0x01, 0x04};
static uint8_t cmd_get_data_ready[]       = {0x02, 0x02};
static uint8_t cmd_read_measurement[]     = {0x03, 0x00};
static uint8_t cmd_sleep[]                = {0x10, 0x01};
static uint8_t cmd_wake_up[]              = {0x11, 0x03};
static uint8_t cmd_start_fan_cleaning[]   = {0x56, 0x07};
static uint8_t cmd_autoclean_interval[]   = {0x80, 0x04};
static uint8_t cmd_get_product[]          = {0xd0, 0x02};
static uint8_t cmd_get_serial_number[]    = {0xd0, 0x33};
static uint8_t cmd_get_firmware_version[] = {0xd1, 0x00};
static uint8_t cmd_get_device_status[]    = {0xd2, 0x06};
static uint8_t cmd_clear_device_status[]  = {0xd2, 0x10};
static uint8_t cmd_reset[]                = {0xd3, 0x04};

static uint8_t buffer[60];

#define SPS_CMD_START_STOP_DELAY_USEC 20000
#define SPS_CMD_DELAY_USEC 5000
#define SPS_CMD_DELAY_WRITE_FLASH_USEC 20000

#define SPS30_SERIAL_NUM_WORDS ((SPS30_SERIAL_MAX_LEN) / 2)


sps30_t *sps30_create_master(i2c_master_bus_handle_t bus_handle)
{
    sps30_t *sensor = calloc(1, sizeof(sps30_t));

    if (sensor == NULL) {
        sps30_close(sensor);
        return NULL;
    }
    sensor->bus_handle = bus_handle;
    sensor->dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    sensor->dev_cfg.device_address = SPS30_I2C_ADDRESS;
    sensor->dev_cfg.scl_speed_hz = CONFIG_SPS30_I2C_CLK_SPEED_HZ;
    sensor->dev_handle = NULL;
    sensor->enabled = true;
    return sensor;
}

/**
 * Create sensor device.
 * @param sps30 Driver Sturcture.
 * @param dev_addr Chip address.
 */
esp_err_t sps30_device_create(sps30_t *sensor)
{
    sensor->dev_cfg.device_address = SPS30_I2C_ADDRESS;

    // Add device to the I2C bus
    esp_err_t err = i2c_master_bus_add_device(sensor->bus_handle, &sensor->dev_cfg, &sensor->dev_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create device on I2C address 0x%02x", SPS30_I2C_ADDRESS);
        return err;
    }
    return ESP_OK;
}

esp_err_t sps30_device_init(sps30_t **sensor_ptr, i2c_master_bus_handle_t bus_handle)
{
    *sensor_ptr = sps30_create_master(bus_handle);
    if (*sensor_ptr == NULL) {
        ESP_LOGE(TAG, "Failed to create master on I2C bus");
        return ESP_FAIL;
    }
    return sps30_device_create(*sensor_ptr);
}

void sps30_close(sps30_t *sensor)
{
    if (sensor != NULL && sensor->dev_handle != NULL) {
        i2c_master_bus_rm_device(sensor->dev_handle);
    }
    free(sensor);
}

/**
 * Read from sensor.
 * @param sps30 Driver Sturcture.
 * @param addr Register address.
 * @param dout Data to read.
 * @param size The number of bytes to read.
 * @returns Error codes.
 */
static esp_err_t sps30_read(sps30_t *sensor, uint8_t *addr, uint8_t *dout, size_t size)
{
    esp_err_t err = i2c_master_transmit(sensor->dev_handle, addr, SPS30_ADDR_SIZE, CONFIG_SPS30_TIMEOUT);

    if (err != ESP_OK) return err;
    return i2c_master_receive(sensor->dev_handle, dout, size, CONFIG_SPS30_TIMEOUT);
}

static esp_err_t sps30_write(sps30_t *sensor, uint8_t *addr, uint8_t *din, size_t size)
{
    if (din == NULL || size == 0) {
        return i2c_master_transmit(sensor->dev_handle, addr, SPS30_ADDR_SIZE, CONFIG_SPS30_TIMEOUT);
    }

    i2c_master_transmit_multi_buffer_info_t buffer[2] = {
        {.write_buffer = addr, .buffer_size = SPS30_ADDR_SIZE},
        {.write_buffer = din, .buffer_size = size},
    };

    return i2c_master_multi_buffer_transmit(sensor->dev_handle, buffer, 2, CONFIG_SPS30_TIMEOUT);
}

const char* sps30_get_driver_version(void) {
    return "1.0.0";
}

esp_err_t sps30_probe(sps30_t *sensor)
{
    esp_err_t err;

    // Try to wake up, but ignore failure if it is not in sleep mode
    sps30_wake_up(sensor);
    if ((err = sps30_get_serial(sensor)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to probe sensor");
        return err;
    }
    return ESP_OK;
}

esp_err_t sps30_get_device_info(sps30_t *sensor)
{
    uint8_t cnt;
    esp_err_t err = sps30_read(sensor, cmd_get_product, buffer, 12);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read I2C data for device info");
        return err;
    }
    cnt = sps30_bytes_to_data(buffer, 12, (uint8_t *)sensor->device_info);
    sensor->device_info[cnt] = '\0';
    if (cnt < SPS30_DEV_INFO_MAX_LEN) {
        ESP_LOGE(TAG, "Failed to read complete device info");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t sps30_get_serial(sps30_t *sensor)
{
    uint8_t cnt;
    esp_err_t err = sps30_read(sensor, cmd_get_serial_number, buffer, 48);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read I2C data for serial number");
        return err;
    }
    cnt = sps30_bytes_to_data(buffer, 48, (uint8_t *)sensor->serial);
    sensor->serial[cnt] = '\0';
    if (cnt < SPS30_SERIAL_MAX_LEN) {
        ESP_LOGE(TAG, "Failed to read complete serial number");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t sps30_get_firmware_version(sps30_t *sensor)
{
    esp_err_t err = sps30_read(sensor, cmd_get_firmware_version, buffer, 3);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read I2C data for firmware version");
        return err;
    }
    if (!sps30_is_data_valid(buffer, 3)) {
        ESP_LOGE(TAG, "Checksum error of received I2C data for firmware version");
        return ESP_FAIL;
    }
    sensor->fw_version = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    return err;
}

esp_err_t sps30_start_measurement(sps30_t *sensor)
{
    uint8_t cmd_args[] = { 0x03 /* 3=float, 5=uint16 */, 0x00, 0x00 };
    esp_err_t err;

    cmd_args[2] = sps30_calc_cksum(cmd_args, 2);
    ESP_LOG_BUFFER_HEXDUMP(TAG, cmd_args, 3, ESP_LOG_INFO);
    err = sps30_write(sensor, cmd_start_measurement, cmd_args, sizeof(cmd_args));
    vTaskDelay(pdMS_TO_TICKS(20));
    return err;
}

esp_err_t sps30_stop_measurement(sps30_t *sensor)
{
    esp_err_t err = sps30_write(sensor, cmd_stop_measurement, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(20));
    return err;
}

bool sps30_read_data_ready(sps30_t *sensor)
{
    ESP_ERROR_CHECK(sps30_read(sensor, cmd_get_data_ready, buffer, 3));
    return buffer[1] == 0x01;
}

esp_err_t sps30_read_measurement(sps30_t *sensor)
{
    esp_err_t err = sps30_read(sensor, cmd_read_measurement, buffer, 60);

    if (err != ESP_OK) return err;
    if (!sps30_is_data_valid(buffer, 60)) return ESP_FAIL;
    sensor->values.mc_1p0 = sps30_bytes_to_float(buffer);
    sensor->values.mc_2p5 = sps30_bytes_to_float(&buffer[6]);
    sensor->values.mc_4p0 = sps30_bytes_to_float(&buffer[12]);
    sensor->values.mc_10p0 = sps30_bytes_to_float(&buffer[18]);
    sensor->values.nc_0p5 = sps30_bytes_to_float(&buffer[24]);
    sensor->values.nc_1p0 = sps30_bytes_to_float(&buffer[30]);
    sensor->values.nc_2p5 = sps30_bytes_to_float(&buffer[36]);
    sensor->values.nc_4p0 = sps30_bytes_to_float(&buffer[42]);
    sensor->values.nc_10p0 = sps30_bytes_to_float(&buffer[48]);
    sensor->values.typical_particle_size = sps30_bytes_to_float(&buffer[54]);
    return ESP_OK;
}

esp_err_t sps30_get_fan_auto_cleaning_interval(sps30_t *sensor)
{
    esp_err_t err = sps30_read(sensor, cmd_autoclean_interval, buffer, 6);

    if (err != ESP_OK) return err;
    if (!sps30_is_data_valid(buffer, 6)) return ESP_FAIL;
    sensor->autoclean_interval = sps30_bytes_to_uint32(buffer);
    return ESP_OK;
}

esp_err_t sps30_set_fan_auto_cleaning_interval(sps30_t *sensor, uint32_t autoclean_interval)
{
    uint8_t *data = sps30_uint32_to_bytes(autoclean_interval);
    esp_err_t err = sps30_write(sensor, cmd_stop_measurement, data, 6);

    vTaskDelay(pdMS_TO_TICKS(20));
    return err;
}

esp_err_t sps30_start_manual_fan_cleaning(sps30_t *sensor)
{
    esp_err_t err = sps30_write(sensor, cmd_start_fan_cleaning, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(20));
    return err;
}

esp_err_t sps30_reset(sps30_t *sensor)
{
    esp_err_t err = sps30_write(sensor, cmd_reset, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(100));
    return err;
}

esp_err_t sps30_sleep(sps30_t *sensor)
{
    esp_err_t err = sps30_write(sensor, cmd_sleep, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(5));
    return err;
}

esp_err_t sps30_wake_up(sps30_t *sensor)
{
    esp_err_t err;

    /* wake-up must be sent twice within 100ms, ignore first return value */
    sps30_write(sensor, cmd_wake_up, NULL, 0);
    err = sps30_write(sensor, cmd_wake_up, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    return err;
}

esp_err_t sps30_read_device_status_register(sps30_t *sensor)
{
    esp_err_t err = sps30_read(sensor, cmd_get_device_status, buffer, 6);

    if (err != ESP_OK) return err;
    if (!sps30_is_data_valid(buffer, 6)) return ESP_FAIL;
    sensor->values.status = sps30_bytes_to_uint32(buffer);
    return ESP_OK;
}

esp_err_t sps30_clear_device_status_register(sps30_t *sensor)
{
    esp_err_t err = sps30_write(sensor, cmd_clear_device_status, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(5));
    return err;
}

esp_err_t sps30_init_do(sps30_t *sensor)
{
    esp_err_t err;

    if ((err = sps30_probe(sensor)) != ESP_OK) return err;
    if ((err = sps30_get_device_info(sensor)) != ESP_OK) return err;
    if ((err = sps30_get_firmware_version(sensor)) != ESP_OK) return err;
    if ((err = sps30_start_measurement(sensor)) != ESP_OK) return err;
    if ((err = sps30_reset(sensor)) != ESP_OK) return err;
    if ((err = sps30_start_measurement(sensor)) != ESP_OK) return err;
    ESP_LOGI(TAG, "SPS30 initialized");
    return ESP_OK;
}

esp_err_t sps30_init(sps30_t **sensor_ptr, i2c_master_bus_handle_t bus_handle)
{
    sps30_t *sensor = NULL;
    esp_err_t err;

    ESP_LOGI(TAG, "Initialize SPS30");
    if ((err = sps30_device_init(&sensor, bus_handle)) != ESP_OK) return err;
    for (int i = 0; i < 5; i++) {
        if ((err = sps30_init_do(sensor)) == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (err != ESP_OK) return err;
    sps30_dump_info(sensor);
    *sensor_ptr = sensor;
    ESP_LOGI(TAG, "SPS30 initialized");
    return ESP_OK;
}

void sps30_dump_info(sps30_t *sensor)
{
    ESP_LOGI(TAG, "DevInfo=%s  Serial=%s  FW=%d.%d",
        sensor->device_info, sensor->serial,
        sensor->fw_version >> 8, sensor->fw_version & 0xff);
    if (strncmp(sensor->device_info, "00080000", 8)) {
        ESP_LOGE(TAG, "returned invalid DevInfo");
    }
}

void sps30_dump_values(sps30_t *sensor, bool force)
{
    if (force || sensor->debug & 1) {
        sps30_values_t *values = &sensor->values;

        ESP_LOGI(TAG, "STATUS=%08X", (unsigned int)sensor->values.status);
        ESP_LOGI(TAG, "PM0.5 =%.1f #/cm3", values->nc_0p5);
        ESP_LOGI(TAG, "PM1.0 =%.1f ug/cm3 P1.0 =%.1f #/cm3", values->mc_1p0, values->nc_1p0);
        ESP_LOGI(TAG, "PM2.5 =%.1f ug/cm3 P2.5 =%.1f #/cm3", values->mc_2p5, values->nc_2p5);
        ESP_LOGI(TAG, "PM4.0 =%.1f ug/cm3 P4.0 =%.1f #/cm3", values->mc_4p0, values->nc_4p0);
        ESP_LOGI(TAG, "PM10.0=%.1f ug/cm3 P10.0=%.1f #/cm3", values->mc_10p0, values->nc_10p0);
        ESP_LOGI(TAG, "TypPartSz=%.3f um", values->typical_particle_size);
    }
}
