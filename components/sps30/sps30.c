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


sps30_t* sps30_create_master(i2c_master_bus_handle_t bus_handle)
{
    sps30_t *sps30 = malloc(sizeof(sps30_t));
    if (sps30 == NULL) {
        sps30_close(sps30);
        return NULL;
    }
    memset(sps30, 0, sizeof(sps30_t));
    sps30->bus_handle = bus_handle;
    sps30->dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    sps30->dev_cfg.device_address = SPS30_I2C_ADDRESS;
    sps30->dev_cfg.scl_speed_hz = CONFIG_SPS30_I2C_CLK_SPEED_HZ;
    sps30->dev_handle = NULL;
    return sps30;
}

/**
 * Create sensor device.
 * @param sps30 Driver Sturcture.
 * @param dev_addr Chip address.
 */
esp_err_t sps30_device_create(sps30_t *sps30)
{
    sps30->dev_cfg.device_address = SPS30_I2C_ADDRESS;

    // Add device to the I2C bus
    esp_err_t err = i2c_master_bus_add_device(sps30->bus_handle, &sps30->dev_cfg, &sps30->dev_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create device on I2C address 0x%02x", SPS30_I2C_ADDRESS);
        return err;
    }
    return ESP_OK;
}

esp_err_t sps30_device_init(sps30_t **sps30, i2c_master_bus_handle_t bus_handle)
{
    *sps30 = sps30_create_master(bus_handle);
    if (*sps30 == NULL) {
        ESP_LOGE(TAG, "Failed to create master on I2C bus");
        return ESP_FAIL;
    }
    return sps30_device_create(*sps30);
}

void sps30_close(sps30_t *sps30)
{
    if (sps30 != NULL && sps30->dev_handle != NULL) {
        i2c_master_bus_rm_device(sps30->dev_handle);
    }
    free(sps30);
}

/**
 * Read from sensor.
 * @param sps30 Driver Sturcture.
 * @param addr Register address.
 * @param dout Data to read.
 * @param size The number of bytes to read.
 * @returns Error codes.
 */
static esp_err_t sps30_read(sps30_t *sps30, uint8_t *addr, uint8_t *dout, size_t size)
{
    esp_err_t err = i2c_master_transmit(sps30->dev_handle, addr, SPS30_ADDR_SIZE, CONFIG_SPS30_TIMEOUT);
    if (err != ESP_OK) return err;
    return i2c_master_receive(sps30->dev_handle, dout, size, CONFIG_SPS30_TIMEOUT);
}

static esp_err_t sps30_write(sps30_t* sps30, uint8_t *addr, uint8_t *din, size_t size)
{
    if (din == NULL || size == 0) {
        return i2c_master_transmit(sps30->dev_handle, addr, SPS30_ADDR_SIZE, CONFIG_SPS30_TIMEOUT);
    }
    i2c_master_transmit_multi_buffer_info_t buffer[2] = {
        {.write_buffer = addr, .buffer_size = SPS30_ADDR_SIZE},
        {.write_buffer = din, .buffer_size = size},
    };
    return i2c_master_multi_buffer_transmit(sps30->dev_handle, buffer, 2, CONFIG_SPS30_TIMEOUT);
}

const char* sps30_get_driver_version(void) {
    return "1.0.0";
}

esp_err_t sps30_probe(sps30_t* sps30)
{
    esp_err_t err;

    // Try to wake up, but ignore failure if it is not in sleep mode
    sps30_wake_up(sps30);
    if ((err = sps30_get_serial(sps30)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to probe sensor");
        return err;
    }
    return ESP_OK;
}

esp_err_t sps30_get_device_info(sps30_t *sps30)
{
    uint8_t cnt;
    esp_err_t err = sps30_read(sps30, cmd_get_product, buffer, 12);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read I2C data for device info");
        return err;
    }
    cnt = bytes_to_data(buffer, 12, (uint8_t *)sps30->device_info);
    sps30->device_info[cnt] = '\0';
    if (cnt < SPS30_DEV_INFO_MAX_LEN) {
        ESP_LOGE(TAG, "Failed to read complete device info");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t sps30_get_serial(sps30_t *sps30)
{
    uint8_t cnt;
    esp_err_t err = sps30_read(sps30, cmd_get_serial_number, buffer, 48);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read I2C data for serial number");
        return err;
    }
    cnt = bytes_to_data(buffer, 48, (uint8_t *)sps30->serial);
    sps30->serial[cnt] = '\0';
    if (cnt < SPS30_SERIAL_MAX_LEN) {
        ESP_LOGE(TAG, "Failed to read complete serial number");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t sps30_get_firmware_version(sps30_t *sps30)
{
    esp_err_t err = sps30_read(sps30, cmd_get_firmware_version, buffer, 3);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read I2C data for firmware version");
        return err;
    }
    if (!is_data_valid(buffer, 3)) {
        ESP_LOGE(TAG, "Checksum error of received I2C data for firmware version");
        return ESP_FAIL;
    }
    sps30->firmware_version = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    return err;
}

esp_err_t sps30_start_measurement(sps30_t *sps30)
{
    uint8_t cmd_args[] = { 0x03 /* 3=float, 5=uint16 */, 0x00, 0x00 };
    esp_err_t err;

    cmd_args[2] = calc_cksum(cmd_args, 2);
    ESP_LOG_BUFFER_HEXDUMP(TAG, cmd_args, 3, ESP_LOG_INFO);
    err = sps30_write(sps30, cmd_start_measurement, cmd_args, sizeof(cmd_args));
    vTaskDelay(pdMS_TO_TICKS(20));
    return err;
}

esp_err_t sps30_stop_measurement(sps30_t *sps30)
{
    esp_err_t err = sps30_write(sps30, cmd_stop_measurement, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(20));
    return err;
}

bool sps30_read_data_ready(sps30_t *sps30)
{
    ESP_ERROR_CHECK(sps30_read(sps30, cmd_get_data_ready, buffer, 3));
    return buffer[1] == 0x01;
}

esp_err_t sps30_read_measurement(sps30_t *sps30)
{
    esp_err_t err = sps30_read(sps30, cmd_read_measurement, buffer, 60);

    if (err != ESP_OK) return err;
    if (!is_data_valid(buffer, 60)) return ESP_FAIL;
    sps30->values.mc_1p0 = bytes_to_float(buffer);
    sps30->values.mc_2p5 = bytes_to_float(&buffer[6]);
    sps30->values.mc_4p0 = bytes_to_float(&buffer[12]);
    sps30->values.mc_10p0 = bytes_to_float(&buffer[18]);
    sps30->values.nc_0p5 = bytes_to_float(&buffer[24]);
    sps30->values.nc_1p0 = bytes_to_float(&buffer[30]);
    sps30->values.nc_2p5 = bytes_to_float(&buffer[36]);
    sps30->values.nc_4p0 = bytes_to_float(&buffer[42]);
    sps30->values.nc_10p0 = bytes_to_float(&buffer[48]);
    sps30->values.typical_particle_size = bytes_to_float(&buffer[54]);
    return ESP_OK;
}

esp_err_t sps30_get_fan_auto_cleaning_interval(sps30_t *sps30)
{
    esp_err_t err = sps30_read(sps30, cmd_autoclean_interval, buffer, 6);

    if (err != ESP_OK) return err;
    if (!is_data_valid(buffer, 6)) return ESP_FAIL;
    sps30->autoclean_interval = bytes_to_uint32(buffer);
    return ESP_OK;
}

esp_err_t sps30_set_fan_auto_cleaning_interval(sps30_t *sps30, uint32_t autoclean_interval)
{
    uint8_t *data = uint32_to_bytes(autoclean_interval);
    esp_err_t err = sps30_write(sps30, cmd_stop_measurement, data, 6);

    vTaskDelay(pdMS_TO_TICKS(20));
    return err;
}

esp_err_t sps30_start_manual_fan_cleaning(sps30_t *sps30)
{
    esp_err_t err = sps30_write(sps30, cmd_start_fan_cleaning, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(20));
    return err;
}

esp_err_t sps30_reset(sps30_t* sps30)
{
    esp_err_t err = sps30_write(sps30, cmd_reset, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(100));
    return err;
}

esp_err_t sps30_sleep(sps30_t* sps30)
{
    esp_err_t err = sps30_write(sps30, cmd_sleep, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(5));
    return err;
}

esp_err_t sps30_wake_up(sps30_t* sps30)
{
    esp_err_t err;

    /* wake-up must be sent twice within 100ms, ignore first return value */
    sps30_write(sps30, cmd_wake_up, NULL, 0);
    err = sps30_write(sps30, cmd_wake_up, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    return err;
}

esp_err_t sps30_read_device_status_register(sps30_t* sps30)
{
    esp_err_t err = sps30_read(sps30, cmd_get_device_status, buffer, 6);

    if (err != ESP_OK) return err;
    if (!is_data_valid(buffer, 6)) return ESP_FAIL;
    sps30->status = bytes_to_uint32(buffer);
    return ESP_OK;
}

esp_err_t sps30_clear_device_status_register(sps30_t* sps30)
{
    esp_err_t err = sps30_write(sps30, cmd_clear_device_status, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(5));
    return err;
}

sps30_t *sps30_init(i2c_master_bus_handle_t bus_handle)
{
    sps30_t *sps30 = NULL;
    esp_err_t err;

    if ((err = sps30_device_init(&sps30, bus_handle)) != ESP_OK) return NULL;
    if ((err = sps30_probe(sps30)) != ESP_OK) return NULL;
    if ((err = sps30_get_device_info(sps30)) != ESP_OK) return NULL;
    if ((err = sps30_get_firmware_version(sps30)) != ESP_OK) return NULL;
    if ((err = sps30_start_measurement(sps30)) != ESP_OK) return NULL;
    if ((err = sps30_reset(sps30)) != ESP_OK) return NULL;
    if ((err = sps30_start_measurement(sps30)) != ESP_OK) return NULL;
    return sps30;
}