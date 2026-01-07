#include <stdint.h>
#include <string.h>
#include <math.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "adxl345.h"

static const char *TAG = "ADXL345";

#define ADXL345_I2C_ADDR        0x53    // ADXL345 I2C address

#define DEVID_REG               0x00    // Device Id
#define TAP_THRESH_REG          0x1D    // Threshold value for TAP interrupts
#define OFSX_REG                0x1E    // X offset
#define OFSY_REG                0x1F    // Y offset
#define OFSZ_REG                0x20    // Z offset
#define TAP_DUR_REG             0x21    // Duration value for TAP detection
#define TAP_LATENT_REG          0x22    // Latency value for TAP double detection
#define TAP_WINDOW_REG          0x23    // Window value for TAP double detection
#define TAP_THRESH_ACT_REG      0x24    // Threshold activity value for TAP double detection
#define TAP_THRESH_INACT_REG    0x25    // Threshold inactivity value for TAP double detection
#define TAP_TIME_INACT_REG      0x26    // Time inactivity value for TAP double detection
#define TAP_ACT_INACT_CTL_REG   0x27    // Activity/inactivity control for TAP double detection
#define TAP_ACT_STATUS_REG      0x2B    // Activity status for TAP double detection
#define BW_RATE_CTL_REG         0x2C    // Power and rate control
#define POWER_CTL_REG           0x2D    // Power control
#define INT_ENABLE_REG          0x2E    // Interrupt enable
#define INT_MAP_REG             0x2F    // Interrupt mapping
#define INT_SOURCE_REG          0x30    // Interrupt source
#define DATA_FORMAT_REG         0x31    // Data format
#define DATAX0_REG              0x32    // Data X low byte
#define DATAX1_REG              0x33    // Data X high byte
#define DATAY0_REG              0x34    // Data Y low byte
#define DATAY1_REG              0x35    // Data Y high byte
#define DATAZ0_REG              0x36    // Data Z low byte
#define DATAZ1_REG              0x37    // Data Z high byte
#define FIFO_CTL_REG            0x38    // FIFO control

#define ACCEL_RANGE_2           0       // Accelerometer full scale range +/- 2g
#define ACCEL_RANGE_4           1       // Accelerometer full scale range +/- 4g
#define ACCEL_RANGE_8           2       // Accelerometer full scale range +/- 8g
#define ACCEL_RANGE_16          3       // Accelerometer full scale range +/- 16g

#define GYRO_RANGE_250          0       // Gyrometer full scale range +/- 250°/s
#define GYRO_RANGE_500          1       // Gyrometer full scale range +/- 500°/s
#define GYRO_RANGE_1000         2       // Gyrometer full scale range +/- 1000°/s
#define GYRO_RANGE_2000         3       // Gyrometer full scale range +/- 2000°/s


void adxl345_close(adxl345_t *sensor);


esp_err_t adxl345_read(adxl345_t *sensor, uint8_t addr, uint8_t *dout, size_t size)
{
    return i2c_master_transmit_receive(sensor->i2c_dev, &addr, 1, dout, size, CONFIG_ADXL345_TIMEOUT);
}

static esp_err_t adxl345_write(adxl345_t *sensor, uint8_t addr, uint8_t *din, size_t size)
{
    if (din == NULL || size == 0) {
        return i2c_master_transmit(sensor->i2c_dev, &addr, 1, CONFIG_ADXL345_TIMEOUT);
    }

    i2c_master_transmit_multi_buffer_info_t buffer[2] = {
        {.write_buffer = &addr, .buffer_size = 1},
        {.write_buffer = din, .buffer_size = size},
    };

    return i2c_master_multi_buffer_transmit(sensor->i2c_dev, buffer, 2, CONFIG_ADXL345_TIMEOUT);
}

static esp_err_t adxl345_write_byte(adxl345_t *sensor, uint8_t addr, uint8_t data)
{
    return adxl345_write(sensor, addr, &data, 1);
}

esp_err_t adxl345_get_device_id(adxl345_t *sensor)
{
    return adxl345_read(sensor, DEVID_REG, &sensor->device_id, 1);
}

esp_err_t adxl345_set_accel_range(adxl345_t *sensor, uint8_t range)
{
    esp_err_t err = adxl345_write(sensor, DATA_FORMAT_REG, &range, 1);

    if (err != ESP_OK) return err;
    sensor->accel_range = range;
    return ESP_OK;
}

esp_err_t adxl345_read_data(adxl345_t *sensor)
{
    esp_err_t err;
    uint8_t data[6];
    float accel_range = 2.0 / 16384.0;

    if ((err = adxl345_read(sensor, DATAX0_REG, data, sizeof(data))) != ESP_OK) return err;
    if (sensor->accel_range == 1) accel_range = 4.0 / 16384.0;
    else if (sensor->accel_range == 2) accel_range = 8.0 / 16384.0;
    else if (sensor->accel_range == 3) accel_range = 16.0 / 16384.0;
    adxl345_values_t *values = &sensor->values;
    values->accel_x = (float)(((int16_t)data[0] << 8) | (int16_t)data[1]) * accel_range - values->accel_offset_x;
    values->accel_y = (float)(((int16_t)data[2] << 8) | (int16_t)data[3]) * accel_range - values->accel_offset_y;
    values->accel_z = (float)(((int16_t)data[4] << 8) | (int16_t)data[5]) * accel_range - values->accel_offset_z;
    values->accel_abs = sqrt(values->accel_x * values->accel_x + values->accel_y * values->accel_y + values->accel_z * values->accel_z);
    if (values->accel_abs > 1.0) {
        if (sensor->moving_cnt++ > 9) {
            sensor->moving_cnt = 0;
            err = adxl345_calibrate_offset(sensor);
        }
    } else {
        sensor->moving_cnt = 0;
    }
    //ESP_LOG_BUFFER_HEXDUMP(TAG, data, sizeof(data), ESP_LOG_INFO);
    return err;
}

esp_err_t adxl345_calibrate_offset(adxl345_t *sensor)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Calibrating");
    adxl345_values_t *values = &sensor->values;
    values->accel_offset_x = 0;
    values->accel_offset_y = 0;
    values->accel_offset_z = 0;
    if ((err = adxl345_read_data(sensor)) != ESP_OK) return err;
    values->accel_offset_x = values->accel_x;
    values->accel_offset_y = values->accel_y;
    values->accel_offset_z = values->accel_z;
    return ESP_OK;
}

/**
 * Create sensor device.
 * @param adxl345 Driver Sturcture.
 * @param dev_addr Chip address.
 */
static esp_err_t adxl345_device_create(adxl345_t *sensor, const uint16_t dev_addr)
{
    esp_err_t err;

    ESP_LOGI(TAG, "device_create for ADXL345 sensors on ADDR %X", dev_addr);
    sensor->dev_cfg.device_address = dev_addr;
    // Add device to the I2C bus
    err = i2c_master_bus_add_device(sensor->bus_handle, &sensor->dev_cfg, &sensor->i2c_dev);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "device_create success on 0x%x", dev_addr);
        return err;
    } else {
        ESP_LOGE(TAG, "device_create error on 0x%x", dev_addr);
        return err;
    }
}

adxl345_t *adxl345_create_master(i2c_master_bus_handle_t bus_handle)
{
    adxl345_t *sensor = malloc(sizeof(adxl345_t));

    if (sensor) {
        adxl345_values_t *values = &sensor->values;
        memset(sensor, 0, sizeof(adxl345_t));
        sensor->bus_handle = bus_handle;
        sensor->dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        sensor->dev_cfg.device_address = ADXL345_I2C_ADDR;
        sensor->dev_cfg.scl_speed_hz = CONFIG_ADXL345_I2C_CLK_SPEED_HZ;
        sensor->i2c_dev = NULL;
        values->accel_x = 0;
        values->accel_y = 0;
        values->accel_z = 0;
        values->accel_offset_x = 0;
        values->accel_offset_y = 0;
        values->accel_offset_z = 0;
        sensor->moving_cnt = 0;
        sensor->accel_range = 0;
        sensor->device_id = 0x00;
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for ADXL345.");
        adxl345_close(sensor);
        return NULL;
    }
    return sensor;
}

void adxl345_close(adxl345_t *sensor)
{
    if (sensor != NULL && sensor->i2c_dev != NULL) {
        i2c_master_bus_rm_device(sensor->i2c_dev);
    }
    free(sensor);
}

esp_err_t adxl345_device_init(adxl345_t *sensor)
{
    esp_err_t err;

    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if ((err = adxl345_get_device_id(sensor)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get Device ID.");
        adxl345_close(sensor);
        return err;
    }
    if (sensor->device_id != 0xE5) {
        ESP_LOGE(TAG, "Invalid Device ID=%02X", sensor->device_id);
        adxl345_close(sensor);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Device ID=%02X", sensor->device_id);
    // 0x20 (Link) | 0x10 (Auto Sleep) | 0x80 (Measure) | 0x20 (2Hz)
    if ((err = adxl345_write_byte(sensor, POWER_CTL_REG, 0x3A)) != ESP_OK) return err;
    // 0x10 (Activity) | 0x08 (Inactivity)
    if ((err = adxl345_write_byte(sensor, INT_ENABLE_REG, 0x00)) != ESP_OK) return err;
    // 0x08 (Full Res) | 0x03 (+/- 16g)
    if ((err = adxl345_write_byte(sensor, DATA_FORMAT_REG, 0x0B)) != ESP_OK) return err;
    // 0x10 (Low Power) | 0x09 (Rate 50Hz)
    if ((err = adxl345_write_byte(sensor, BW_RATE_CTL_REG, 0x19)) != ESP_OK) return err;
    return err;
}

esp_err_t adxl345_init(adxl345_t **sensor_ptr, i2c_master_bus_handle_t bus_handle)
{
    uint8_t addr = ADXL345_I2C_ADDR;
    esp_err_t err;

    ESP_LOGI(TAG, "Initialized ADXL345");
    adxl345_t *sensor = adxl345_create_master(bus_handle);
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Could not create ADXL345 driver.");
        return ESP_FAIL;
    }
    // Probe and create device
    ESP_LOGI(TAG, "Probing for ADXL345");
    if ((err = i2c_master_probe(bus_handle, addr, CONFIG_ADXL345_PROBE_TIMEOUT)) != ESP_OK) return err;
    ESP_LOGI(TAG, "Found ADXL345 on I2C address 0x%02X", addr);
    if ((err = adxl345_device_create(sensor, addr)) != ESP_OK) return err;
    // Initialize device
    if ((err = adxl345_device_init(sensor)) != ESP_OK) return err;
    ESP_LOGI(TAG, "ADXL345 initialized");
    if ((err = adxl345_calibrate_offset(sensor)) != ESP_OK) return err;
    ESP_LOGI(TAG, "ADXL345 offsets calibrated");
    *sensor_ptr = sensor;
    return ESP_OK;
}

void adxl345_dump_values(adxl345_t *sensor, bool force)
{
    if (force || sensor->debug & 1) {
        adxl345_values_t *values = &sensor->values;

        ESP_LOGI(TAG, "x=%f g  y=%f g  z=%f g  abs=%f g  offsets=%f %f %f  moving_cnt=%d",
                 values->accel_x, values->accel_y, values->accel_z, values->accel_abs,
                 values->accel_offset_x, values->accel_offset_y, values->accel_offset_z,
                 sensor->moving_cnt);
    }
}
