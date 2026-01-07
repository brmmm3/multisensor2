#include <stdint.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "qmc5883l.h"

static const char *TAG = "QMC5883L";

#define QMC5883L_I2C_ADDR       0x0D    // QMC5883L I2C address

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


void qmc5883l_close(qmc5883l_t *sensor);


esp_err_t qmc5883l_read(qmc5883l_t *sensor, uint8_t addr, uint8_t *dout, size_t size)
{
    //ESP_LOGI(TAG, "qmc5883l_read %02X dout=%d size=%d", addr, *dout, size);
    //esp_err_t err = i2c_master_transmit(sensor->i2c_dev, &addr, 1, CONFIG_QMC5883L_TIMEOUT);

    //ESP_LOGI(TAG, "qmc5883l_read err1=%d", err);
    //if (err != ESP_OK) return err;
    return i2c_master_transmit_receive(sensor->i2c_dev, &addr, 1, dout, size, CONFIG_QMC5883L_TIMEOUT);
    //return i2c_master_receive(sensor->i2c_dev, dout, size, CONFIG_QMC5883L_TIMEOUT);
}

static esp_err_t qmc5883l_write(qmc5883l_t *sensor, uint8_t addr, uint8_t *din, size_t size)
{
    ESP_LOGI(TAG, "qmc5883l_write %02X data=%p size=%d", addr, din, size);
    if (din == NULL || size == 0) {
        return i2c_master_transmit(sensor->i2c_dev, &addr, 1, CONFIG_QMC5883L_TIMEOUT);
    }

    ESP_LOGI(TAG, "qmc5883l_write data=%d", *din);
    i2c_master_transmit_multi_buffer_info_t buffer[2] = {
        {.write_buffer = &addr, .buffer_size = 1},
        {.write_buffer = din, .buffer_size = size},
    };

    return i2c_master_multi_buffer_transmit(sensor->i2c_dev, buffer, 2, CONFIG_QMC5883L_TIMEOUT);
}

static esp_err_t qmc5883l_write_byte(qmc5883l_t *sensor, uint8_t addr, uint8_t data)
{
    return qmc5883l_write(sensor, addr, &data, 1);
}

esp_err_t qmc5883l_reset(qmc5883l_t *sensor)
{
    qmc5883l_write_byte(sensor, QMC5883L_CTL1_REG, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t qmc5883l_get_device_id(qmc5883l_t *sensor)
{
    uint8_t data[1];
    esp_err_t err;

    if ((err = qmc5883l_read(sensor, QMC5883L_DEVICE_ID_REG, data, sizeof(data))) != ESP_OK) return err;
    sensor->device_id = (uint32_t)data[0];
    return ESP_OK;
}

esp_err_t qmc5883l_get_status(qmc5883l_t *sensor)
{
    return qmc5883l_read(sensor, QMC5883L_STATUS_REG, &sensor->values.status, 1);
}

bool qmc5883l_data_ready(qmc5883l_t *sensor)
{
    return (sensor->values.status & 1) != 0;
}

bool qmc5883l_data_overflow(qmc5883l_t *sensor)
{
    return (sensor->values.status & 2) != 0;
}

esp_err_t qmc5883l_set_mode(qmc5883l_t *sensor, uint8_t mode, uint8_t odr, uint8_t rng, uint8_t osr)
{
    uint8_t data = mode | odr | rng | osr;

    return qmc5883l_write(sensor, QMC5883L_CTL0_REG, &data, 1);
}

esp_err_t qmc5883l_set_range(qmc5883l_t *sensor, uint8_t range)
{
    uint8_t data = 0x41;
    esp_err_t err;

    if (range == 1) data = 0x51;
    if ((err = qmc5883l_write(sensor, QMC5883L_CTL0_REG, &data, 1)) != ESP_OK) return err;
    if (range == 0) sensor->values.range = 2.0 / 32768.0;
    else sensor->values.range = 8.0 / 32768.0;
    return ESP_OK;
}

esp_err_t qmc5883l_read_data(qmc5883l_t *sensor)
{
    esp_err_t err;
    uint8_t data[9];
    qmc5883l_values_t *values = &sensor->values;

    if ((err = qmc5883l_read(sensor, QMC5883L_DATAX0_REG, data, sizeof(data))) != ESP_OK) return err;
    int16_t mag_x = (int16_t)((uint16_t)data[0] | (uint16_t)data[1] << 8);
    int16_t mag_y = (int16_t)((uint16_t)data[2] | (uint16_t)data[3] << 8);
    int16_t mag_z = (int16_t)((uint16_t)data[4] | (uint16_t)data[5] << 8);
    values->mag_x = (float)mag_x * values->range;
    values->mag_y = (float)mag_y * values->range;
    values->mag_z = (float)mag_z * values->range;
    values->status = data[6];
    //ESP_LOGI(TAG, "status=%X bx=%d by=%d bz=%d", sensor->status, mag_x, mag_y, mag_z);
    //ESP_LOG_BUFFER_HEXDUMP(TAG, data, sizeof(data), ESP_LOG_INFO);
    return ESP_OK;
}

/**
 * Create sensor device.
 * @param qmc5883l Driver Sturcture.
 * @param dev_addr Chip address.
 */
static esp_err_t qmc5883l_device_create(qmc5883l_t *sensor, const uint16_t dev_addr)
{
    esp_err_t err;

    ESP_LOGI(TAG, "device_create for QMC5883L sensors on ADDR 0x%02X", dev_addr);
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

qmc5883l_t *qmc5883l_create_master(i2c_master_bus_handle_t bus_handle)
{
    qmc5883l_t *sensor = malloc(sizeof(qmc5883l_t));

    if (sensor) {
        qmc5883l_values_t *values = &sensor->values;
        memset(sensor, 0, sizeof(qmc5883l_t));
        sensor->bus_handle = bus_handle;
        sensor->dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        sensor->dev_cfg.device_address = QMC5883L_I2C_ADDR;
        sensor->dev_cfg.scl_speed_hz = CONFIG_QMC5883L_I2C_CLK_SPEED_HZ;
        sensor->i2c_dev = NULL;
        values->mag_x = 0.0;
        values->mag_y = 0.0;
        values->mag_z = 0.0;
        values->range = 2.0 / 32768.0;
        values->status = 0x00;
        sensor->device_id = 0x00;
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for QMC5883L.");
        qmc5883l_close(sensor);
        return NULL;
    }
    return sensor;
}

void qmc5883l_close(qmc5883l_t *sensor)
{
    if (sensor != NULL && sensor->i2c_dev != NULL) {
        i2c_master_bus_rm_device(sensor->i2c_dev);
        sensor->i2c_dev = NULL;
    }
    free(sensor);
}

esp_err_t qmc5883l_device_init(qmc5883l_t *sensor)
{
    esp_err_t err;

    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if ((err = qmc5883l_reset(sensor)) != ESP_OK) return err;
    if ((err = qmc5883l_get_device_id(sensor)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get Device ID.");
        qmc5883l_close(sensor);
        return err;
    }
    ESP_LOGI(TAG, "Device ID=%02lX", sensor->device_id);
    if (sensor->device_id != 0xFF) {
        // TODO
    }
    // 0x40 (OSR=256) | 0x00 (Range +/- 2g) | 0x00 (ODR 10Hz) | 0x01 (Continuous)
    // 0x40 (OSR=256) | 0x01 (Range +/- 8g) | 0x00 (ODR 10Hz) | 0x01 (Continuous)
    if ((err = qmc5883l_set_range(sensor, 1)) != ESP_OK) return err;
    return ESP_OK;
}

esp_err_t qmc5883l_init(qmc5883l_t **sensor_ptr, i2c_master_bus_handle_t bus_handle)
{
    uint8_t addr = QMC5883L_I2C_ADDR;
    esp_err_t err;

    ESP_LOGI(TAG, "Initialized QMC5883L");
    qmc5883l_t *sensor = qmc5883l_create_master(bus_handle);
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Could not create QMC5883L driver.");
        return ESP_FAIL;
    }
    // Probe and create device
    ESP_LOGI(TAG, "Probing for QMC5883L");
    if ((err = i2c_master_probe(bus_handle, addr, CONFIG_QMC5883L_PROBE_TIMEOUT)) != ESP_OK) return err;
    ESP_LOGI(TAG, "Found TLV493 on I2C address 0x%02X", addr);
    if ((err = qmc5883l_device_create(sensor, addr)) != ESP_OK) return err;
    // Initialize device
    if ((err = qmc5883l_device_init(sensor)) != ESP_OK) return err;
    ESP_LOGI(TAG, "QMC5883L initialized");
    *sensor_ptr = sensor;
    return ESP_OK;
}

void qmc5883l_dump_values(qmc5883l_t *sensor, bool force)
{
    if (force || sensor->debug & 1) {
        qmc5883l_values_t *values = &sensor->values;

        ESP_LOGI(TAG, "x=%f gauss  y=%f gauss  z=%f gauss  range=%d  status=%d",
            values->mag_x, values->mag_y, values->mag_z, values->range, values->status);
    }
}
