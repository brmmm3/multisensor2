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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "sps30.h"
#include "util.h"

static const char *SPS30_TAG = "SPS30";

uint8_t cmd_start_measurement[]             = {0x00, 0x10};
uint8_t cmd_stop_measurement[]              = {0x01, 0x04};
uint8_t cmd_get_data_ready[]                = {0x02, 0x02};
uint8_t cmd_read_measurement[]              = {0x03, 0x00};
uint8_t cmd_sleep[]                         = {0x10, 0x01};
uint8_t cmd_wake_up[]                       = {0x11, 0x03};
uint8_t cmd_start_fan_cleaning[]            = {0x56, 0x07};
uint8_t cmd_autoclean_interval[]            = {0x80, 0x04};
uint8_t cmd_get_product[]                   = {0xd0, 0x02};
uint8_t cmd_get_serial_number[]             = {0xd0, 0x33};
uint8_t cmd_get_firmware_version[]          = {0xd1, 0x00};
uint8_t cmd_get_device_status[]             = {0xd2, 0x06};
uint8_t cmd_clear_device_status[]           = {0xd2, 0x10};
uint8_t cmd_reset[]                         = {0xd3, 0x04};

#define SPS_CMD_START_STOP_DELAY_USEC 20000
#define SPS_CMD_DELAY_USEC 5000
#define SPS_CMD_DELAY_WRITE_FLASH_USEC 20000

#define SPS30_SERIAL_NUM_WORDS ((SPS30_MAX_SERIAL_LEN) / 2)

sps30_t* sps30_create_master(i2c_master_bus_handle_t bus_handle)
{
    sps30_t* sps30 = malloc(sizeof(sps30_t));
    if (sps30)
    {
        memset(sps30, 0, sizeof(sps30_t));
        sps30->bus_handle = bus_handle;
        sps30->dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        sps30->dev_cfg.device_address = SPS30_I2C_ADDRESS;
        sps30->dev_cfg.scl_speed_hz = CONFIG_SPS30_I2C_CLK_SPEED_HZ;
        sps30->dev_handle = NULL;
    }
    else
    {
        ESP_LOGE(SPS30_TAG, "Failed to allocate memory for sps30.");
        sps30_close(sps30);
        return NULL;
    }
    return sps30;
}

/**
 * Create sensor device.
 * @param sps30 Driver Sturcture.
 * @param dev_addr Chip address.
 */
esp_err_t sps30_device_create(sps30_t *sps30)
{
    ESP_LOGI(SPS30_TAG, "device_create for SPS30 sensors on ADDR %X", SPS30_I2C_ADDRESS);
    sps30->dev_cfg.device_address = SPS30_I2C_ADDRESS;
    // Add device to the I2C bus
    esp_err_t err = i2c_master_bus_add_device(sps30->bus_handle, &sps30->dev_cfg, &sps30->dev_handle);
    if (err == ESP_OK)
    {
        ESP_LOGI(SPS30_TAG, "device_create success on 0x%x", SPS30_I2C_ADDRESS);
        return err;
    }
    else
    {
        ESP_LOGE(SPS30_TAG, "device_create error on 0x%x", SPS30_I2C_ADDRESS);
        return err;
    }
}

esp_err_t sps30_device_init(sps30_t** sps30, i2c_master_bus_handle_t bus_handle)
{
    ESP_LOGI(SPS30_TAG, "Initialize SPS30");
    *sps30 = sps30_create_master(bus_handle);
    if (!*sps30) { 
        ESP_LOGE(SPS30_TAG, "Could not create SPS30 driver.");
        return ESP_FAIL;
    }
    return sps30_device_create(*sps30);
}

void sps30_close(sps30_t *sps30)
{
    if (sps30 != NULL && sps30->dev_handle != NULL)
        i2c_master_bus_rm_device(sps30->dev_handle);
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
    return i2c_master_transmit_receive(sps30->dev_handle, addr, SPS30_ADDR_SIZE, dout, size, CONFIG_SPS30_TIMEOUT);
}

static esp_err_t sps30_write(sps30_t* sps30, uint8_t *addr, uint8_t *din, size_t size)
{
    if (din == NULL || size == 0)
    {
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
    // Try to wake up, but ignore failure if it is not in sleep mode
    ESP_LOGI(SPS30_TAG, "Probing for SPS30 sensor on I2C %X", sps30->dev_cfg.device_address);
    esp_err_t err = sps30_wake_up(sps30);
    if (err != ESP_OK) return err;
    return sps30_get_serial(sps30);
}

esp_err_t sps30_get_firmware_version(sps30_t *sps30)
{
    uint8_t buf[3];
    esp_err_t err = sps30_read(sps30, cmd_get_firmware_version, buf, 3);
    ESP_LOGI("SPS", "%02X %02X %02X", buf[0], buf[1], buf[2]);
    sps30->firmware_version = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    return err;
}

esp_err_t sps30_get_product(sps30_t *sps30)
{
    ESP_LOGI("SPS", "sps30_get_product");
    uint8_t buf[12];
    esp_err_t err = sps30_read(sps30, cmd_get_product, buf, 12);
    ESP_LOGI("SPS", "sps30_get_product=%d", err);
    ESP_LOGI("SPS", "%02X %02X %02X %02X", buf[0], buf[1], buf[2], buf[3]);
    if (err == ESP_OK)
    {
        /* ensure a final '\0'. The firmware should always set this so this is just
        * in case something goes wrong.
        */
        sps30->serial[SPS30_MAX_SERIAL_LEN - 1] = '\0';
    }
    return err;
}

esp_err_t sps30_get_serial(sps30_t *sps30)
{
    ESP_LOGI("SPS", "sps30_get_serial");
    uint8_t buf[48];
    esp_err_t err = sps30_read(sps30, cmd_get_serial_number, buf, 48);
    ESP_LOGI("SPS", "sps30_get_serial=%d", err);
    ESP_LOGI("SPS", "%02X %02X %02X %02X", buf[0], buf[1], buf[2], buf[3]);
    if (err == ESP_OK)
    {
        /* ensure a final '\0'. The firmware should always set this so this is just
        * in case something goes wrong.
        */
        sps30->serial[SPS30_MAX_SERIAL_LEN - 1] = '\0';
    }
    return err;
}

esp_err_t sps30_start_measurement(sps30_t *sps30)
{
    uint8_t cmd_args[] = { 0x05 /* 3=float, 5=uint16 */, 0x00, 0x00};
    cmd_args[2] = calc_cksum(cmd_args, 2);
    esp_err_t err = sps30_write(sps30, cmd_start_measurement, cmd_args, sizeof(cmd_args));
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
    uint8_t buf[3];
    ESP_ERROR_CHECK(sps30_read(sps30, cmd_get_data_ready, buf, 3));
    ESP_LOGI("SPS", "sps30_read_data_ready %02X %02X %02X", buf[0], buf[1], buf[2]);
    return buf[1] == 0x01;
}

esp_err_t sps30_read_measurement(sps30_t *sps30)
{
    uint8_t data[54];

    esp_err_t err = sps30_read(sps30, cmd_read_measurement, (uint8_t *)&data[0], sizeof(data));
    ESP_LOGI("SPS", "sps30_read_measurement %d", err);
    if (err != ESP_OK)
    {
        return err;
    }

    sps30->values.mc_1p0 = bytes_to_float((uint8_t *)&data[0]);
    sps30->values.mc_2p5 = bytes_to_float((uint8_t *)&data[6]);
    sps30->values.mc_4p0 = bytes_to_float((uint8_t *)&data[12]);
    sps30->values.mc_10p0 = bytes_to_float((uint8_t *)&data[18]);
    sps30->values.nc_0p5 = bytes_to_float((uint8_t *)&data[24]);
    sps30->values.nc_1p0 = bytes_to_float((uint8_t *)&data[30]);
    sps30->values.nc_2p5 = bytes_to_float((uint8_t *)&data[36]);
    sps30->values.nc_4p0 = bytes_to_float((uint8_t *)&data[42]);
    sps30->values.nc_10p0 = bytes_to_float((uint8_t *)&data[48]);
    sps30->values.typical_particle_size = bytes_to_float((uint8_t *)&data[9]);

    return ESP_OK;
}

esp_err_t sps30_get_fan_auto_cleaning_interval(sps30_t *sps30)
{
    uint8_t data[6];
    esp_err_t err = sps30_read(sps30, cmd_autoclean_interval, data, sizeof(data));
    if (err != ESP_OK)
    {
        return err;
    }
    sps30->autoclean_interval = bytes_to_uint32(data);
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
    return sps30_write(sps30, cmd_reset, NULL, 0);
}

esp_err_t sps30_sleep(sps30_t* sps30)
{
    esp_err_t err = sps30_write(sps30, cmd_sleep, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    return err;
}

esp_err_t sps30_wake_up(sps30_t* sps30)
{
    /* wake-up must be sent twice within 100ms, ignore first return value */
    sps30_write(sps30, cmd_wake_up, NULL, 0);
    esp_err_t err = sps30_write(sps30, cmd_wake_up, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    return err;
}

esp_err_t sps30_read_device_status_register(sps30_t* sps30)
{
    uint8_t buf[6];

    esp_err_t err = sps30_read(sps30, cmd_read_measurement, buf, sizeof(buf));
    ESP_LOGI("SPS", "STATUS %02X %02X %02X %02X %02X %02X", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
    if (err != ESP_OK)
    {
        return err;
    }
    sps30->status = bytes_to_uint32(buf);
    return ESP_OK;
}
