#include <stdint.h>
#include <string.h>
#include <math.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "xmc5883l.h"

static const char *TAG = "XMC5883L";

#define HMC5883L_I2C_ADDR       0x1E    // HMC5883L I2C address
#define QMC5883L_I2C_ADDR       0x0D    // QMC5883L I2C address

#define HMC5883L_DEV_ID         0x00333448 // Chip ID, "H43"

#define HMC5883L_IDA_REG        0x0A    // Identification register A
#define HMC5883L_IDB_REG        0x0B    // Identification register A
#define HMC5883L_IDC_REG        0x0C    // Identification register A

#define QMC5883L_DATAX0_REG     0x00    // Data X low byte
#define QMC5883L_DATAX1_REG     0x01    // Data X high byte
#define QMC5883L_DATAY0_REG     0x02    // Data Y low byte
#define QMC5883L_DATAY1_REG     0x03    // Data Y high byte
#define QMC5883L_DATAZ0_REG     0x04    // Data Z low byte
#define QMC5883L_DATAZ1_REG     0x05    // Data Z high byte
#define QMC5883L_STATUS_REG     0x06    // Status
#define QMC5883L_TEMP0_REG      0x07    // Temperature low byte
#define QMC5883L_TEMP1_REG      0x08    // Temperature high byte
#define QMC5883L_CTL0_REG       0x09    // Control low byte
#define QMC5883L_CTL1_REG       0x0A    // Control high byte
#define QMC5883L_PERIOD_REG     0x0B    // Period
#define QMC5883L_DEVICE_ID_REG  0x0D    // Device Id


void xmc5883l_close(xmc5883l_t *sensor);


esp_err_t xmc5883l_read(xmc5883l_t *sensor, uint8_t addr, uint8_t *dout, size_t size)
{
    ESP_LOGI(TAG, "xmc5883l_read %02X dout=%d size=%d", addr, *dout, size);
    //esp_err_t err = i2c_master_transmit(sensor->i2c_dev, &addr, 1, CONFIG_QMC5883L_TIMEOUT);

    //ESP_LOGI(TAG, "xmc5883l_read err1=%d", err);
    //if (err != ESP_OK) return err;
    return i2c_master_transmit_receive(sensor->i2c_dev, &addr, 1, dout, size, CONFIG_XMC5883L_TIMEOUT);
    //return i2c_master_receive(sensor->i2c_dev, dout, size, CONFIG_QMC5883L_TIMEOUT);
}

static esp_err_t xmc5883l_write(xmc5883l_t *sensor, uint8_t addr, uint8_t *din, size_t size)
{
    ESP_LOGI(TAG, "xmc5883l_write %02X data=%p size=%d", addr, din, size);
    if (din == NULL || size == 0) {
        return i2c_master_transmit(sensor->i2c_dev, &addr, 1, CONFIG_XMC5883L_TIMEOUT);
    }

    ESP_LOGI(TAG, "xmc5883l_write data=%d", *din);
    i2c_master_transmit_multi_buffer_info_t buffer[2] = {
        {.write_buffer = &addr, .buffer_size = 1},
        {.write_buffer = din, .buffer_size = size},
    };

    return i2c_master_multi_buffer_transmit(sensor->i2c_dev, buffer, 2, CONFIG_XMC5883L_TIMEOUT);
}

static esp_err_t xmc5883l_write_byte(xmc5883l_t *sensor, uint8_t addr, uint8_t data)
{
    return xmc5883l_write(sensor, addr, &data, 1);
}

esp_err_t xmc5883l_reset(xmc5883l_t *sensor)
{
    xmc5883l_write_byte(sensor, QMC5883L_CTL1_REG, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t hmc5883l_get_device_id(xmc5883l_t *sensor)
{
    uint8_t data[3];
    esp_err_t err;

    if ((err = xmc5883l_read(sensor, HMC5883L_IDA_REG, data, sizeof(data))) != ESP_OK) return err;
    sensor->device_id = (uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | (uint32_t)data[2];
    return ESP_OK;
}

esp_err_t qmc5883l_get_device_id(xmc5883l_t *sensor)
{
    uint8_t data[1];
    esp_err_t err;

    if ((err = xmc5883l_read(sensor, QMC5883L_DEVICE_ID_REG, data, sizeof(data))) != ESP_OK) return err;
    sensor->device_id = (uint32_t)data[0];
    return ESP_OK;
}

esp_err_t xmc5883l_get_device_id(xmc5883l_t *sensor)
{
    if (sensor->dev_cfg.device_address== QMC5883L_I2C_ADDR) return qmc5883l_get_device_id(sensor);
    return hmc5883l_get_device_id(sensor);
}

esp_err_t xmc5883l_get_status(xmc5883l_t *sensor)
{
    return xmc5883l_read(sensor, QMC5883L_STATUS_REG, &sensor->status, 1);
}

bool xmc5883l_data_ready(xmc5883l_t *sensor)
{
    return (sensor->status & 1) != 0;
}

bool xmc5883l_data_overflow(xmc5883l_t *sensor)
{
    return (sensor->status & 2) != 0;
}

esp_err_t xmc5883l_set_mode(xmc5883l_t *sensor, uint8_t mode, uint8_t odr, uint8_t rng, uint8_t osr)
{
    uint8_t data = mode | odr | rng | osr;

    return xmc5883l_write(sensor, QMC5883L_CTL0_REG, &data, 1);
}

esp_err_t xmc5883l_set_accel_range(xmc5883l_t *sensor, uint8_t range)
{
    uint8_t data = 0x01;
    esp_err_t err;

    if (range == 1) data = 0x51;
    err = xmc5883l_write(sensor, QMC5883L_CTL0_REG, &data, 1);
    if (err != ESP_OK) return err;
    sensor->accel_range = range;
    return ESP_OK;
}

esp_err_t xmc5883l_read_data(xmc5883l_t *sensor)
{
    esp_err_t err;
    uint8_t data[9];
    float accel_range = 2.0 / 32768.0;

    if ((err = xmc5883l_read(sensor, QMC5883L_DATAX0_REG, data, sizeof(data))) != ESP_OK) return err;
    if (sensor->accel_range == 1) accel_range = 8.0 / 32768.0;
    uint16_t bx = (uint16_t)data[0] << 8 | (uint16_t)data[1];
    uint16_t by = (uint16_t)data[2] << 8 | (uint16_t)data[3];
    uint16_t bz = (uint16_t)data[4] << 8 | (uint16_t)data[5];
    uint8_t status = data[6];
    uint16_t temp = (uint16_t)data[7] << 8 | (uint16_t)data[8];
    ESP_LOGI(TAG, "status=%X bx=%d by=%d bz=%d temp=%d", status, bx, by, bz, temp);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, sizeof(data), ESP_LOG_INFO);
    return ESP_OK;
}

/**
 * Create sensor device.
 * @param qmc5883l Driver Sturcture.
 * @param dev_addr Chip address.
 */
static esp_err_t xmc5883l_device_create(xmc5883l_t *sensor, const uint16_t dev_addr)
{
    esp_err_t err;

    ESP_LOGI(TAG, "device_create for HMC5883L/QMC5883L sensors on ADDR 0x%02X", dev_addr);
    sensor->dev_cfg.device_address = dev_addr;
    // Add device to the I2C bus
    err = i2c_master_bus_add_device(sensor->bus_handle, &sensor->dev_cfg, &sensor->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "device_create error on 0x%02X", dev_addr);
        return err;
    }
    ESP_LOGI(TAG, "device_create success on 0x%02X", dev_addr);
    return ESP_OK;
}

xmc5883l_t *xmc5883l_create_master(i2c_master_bus_handle_t bus_handle)
{
    xmc5883l_t *sensor = malloc(sizeof(xmc5883l_t));

    if (sensor) {
        memset(sensor, 0, sizeof(xmc5883l_t));
        sensor->bus_handle = bus_handle;
        sensor->dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        sensor->dev_cfg.device_address = HMC5883L_I2C_ADDR;
        sensor->dev_cfg.scl_speed_hz = CONFIG_XMC5883L_I2C_CLK_SPEED_HZ;
        sensor->i2c_dev = NULL;
        sensor->accel_x = 0.0;
        sensor->accel_y = 0.0;
        sensor->accel_z = 0.0;
        sensor->temp = 0.0;
        sensor->status = 0x00;
        sensor->accel_range = 0x00;
        sensor->device_id = 0x00;
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for HMC5883L/QMC5883L.");
        xmc5883l_close(sensor);
        return NULL;
    }
    return sensor;
}

void xmc5883l_close(xmc5883l_t *sensor)
{
    if (sensor != NULL && sensor->i2c_dev != NULL) {
        i2c_master_bus_rm_device(sensor->i2c_dev);
        sensor->i2c_dev = NULL;
    }
    free(sensor);
}

esp_err_t xmc5883l_device_init(xmc5883l_t *sensor)
{
    esp_err_t err;

    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if ((err = xmc5883l_reset(sensor)) != ESP_OK) return err;
    if ((err = xmc5883l_get_device_id(sensor)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get Device ID.");
        xmc5883l_close(sensor);
        return err;
    }
    ESP_LOGI(TAG, "Device ID=%02lX", sensor->device_id);
    if (sensor->device_id != 0xFF) {
    }
    // 0x00 (OSR=512) | 0x00 (Range +/- 2g) | 0x00 (ODR 10Hz) | 0x01 (Continuous)
    if ((err = xmc5883l_write_byte(sensor, QMC5883L_CTL0_REG, 0x01)) != ESP_OK) return err;
    return ESP_OK;
}

esp_err_t xmc5883l_init(xmc5883l_t **sensor, i2c_master_bus_handle_t bus_handle)
{
    uint8_t addr = HMC5883L_I2C_ADDR;
    esp_err_t err;

    ESP_LOGI(TAG, "Initialized XMC5883L");
    *sensor = xmc5883l_create_master(bus_handle);
    if (!*sensor) {
        ESP_LOGE(TAG, "Could not create XMC5883L driver.");
        return ESP_FAIL;
    }
    // Probe and create device
    ESP_LOGI(TAG, "Probing for HMC5883L");
    if ((err = i2c_master_probe(bus_handle, addr, CONFIG_TLV493_PROBE_TIMEOUT)) != ESP_OK) {
        addr = QMC5883L_I2C_ADDR;
        ESP_LOGI(TAG, "Probing for QMC5883L");
        if ((err = i2c_master_probe(bus_handle, addr, CONFIG_TLV493_PROBE_TIMEOUT)) != ESP_OK) return err;
    }
    ESP_LOGI(TAG, "Found TLV493 on I2C address 0x%02X", addr);
    if ((err = xmc5883l_device_create(*sensor, addr)) != ESP_OK) return err;
    // Initialize device
    if ((err = xmc5883l_device_init(*sensor)) != ESP_OK) return err;
    ESP_LOGI(TAG, "XMC5883L initialized");
    return ESP_OK;
}
