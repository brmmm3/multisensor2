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
#include <stdint.h>

#include "s11.h"
#include "s11_defs.h"

static const char *TAG = "S11";

// Change address to 0x63
#define S11_SENSOR_ADDR 0x63


uint16_t get_u16(uint8_t *buf)
{
  return (uint16_t)buf[0] << 8 | (uint16_t)buf[1];
}

uint32_t get_u32(uint8_t *buf)
{
  return (uint32_t)buf[0] << 24 | (uint32_t)buf[1] << 16 | (uint32_t)buf[2] << 8 | (uint32_t)buf[3];
}

static esp_err_t execute_cmd(s11_t *sensor, uint8_t cmd, uint8_t *out_data, size_t out_bytes, uint8_t *in_data, size_t in_bytes)
{
    uint8_t buf[16];
    uint8_t pos = 0;

    buf[0] = cmd;
    while (pos++ < out_bytes) {
        buf[pos] = out_data[pos - 1];
    }
    s11_wakeup(sensor);  // Wakeup is necessary before sending the command. 15ms time to send command after wakeup
    if (in_bytes == 0) {
        return i2c_master_transmit(sensor->dev_handle, buf, 1 + out_bytes, CONFIG_S11_TIMEOUT);
    }
    return i2c_master_transmit_receive(sensor->dev_handle, buf, 1 + out_bytes, in_data, in_bytes, CONFIG_S11_TIMEOUT);
}

static esp_err_t s11_read_u8(s11_t *sensor, uint8_t cmd, uint8_t *in_data)
{
    sensor->last_error = execute_cmd(sensor, cmd, NULL, 0, in_data, 1);
    return sensor->last_error;
}

static esp_err_t s11_read_u16(s11_t *sensor, uint8_t cmd, uint16_t *in_data)
{
    uint8_t buf[2];

    sensor->last_error = execute_cmd(sensor, cmd, NULL, 0, buf, 2);
    *in_data = get_u16(buf);
    return sensor->last_error;
}

static esp_err_t s11_read_u32(s11_t *sensor, uint8_t cmd, uint32_t *in_data)
{
    uint8_t buf[4];

    sensor->last_error = execute_cmd(sensor, cmd, NULL, 0, buf, 4);
    *in_data = get_u32(buf);
    return sensor->last_error;
}

static esp_err_t s11_write_u8(s11_t *sensor, uint8_t cmd, uint8_t out_data)
{
    sensor->last_error = execute_cmd(sensor, cmd, &out_data, 1, NULL, 0);
    return sensor->last_error;
}

static esp_err_t s11_write_u16(s11_t *sensor, uint8_t cmd, uint16_t out_data)
{
    uint8_t buf[2];

    buf[0] = out_data >> 8;
    buf[1] = out_data;
    sensor->last_error = execute_cmd(sensor, cmd, buf, 2, NULL, 0);
    return sensor->last_error;
}

static s11_t *s11_create_master(i2c_master_bus_handle_t bus_handle)
{
    s11_t *sensor = pvPortMalloc(sizeof(s11_t));
    memset(sensor, 0, sizeof(s11_t));

    if (sensor != NULL) {
        sensor->bus_handle = bus_handle;
        sensor->dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for S11.");
        s11_close(sensor);
        return NULL;
    }
    return sensor;
}

static esp_err_t s11_device_create(s11_t *sensor)
{
    ESP_LOGI(TAG, "device_create for S11 sensors on ADDR %X", S11_SENSOR_ADDR);
    sensor->dev_config.device_address = S11_SENSOR_ADDR;
    sensor->dev_config.scl_speed_hz = CONFIG_S11_I2C_CLK_SPEED_HZ;
    sensor->dev_config.flags.disable_ack_check = true;
    // Add device to the I2C bus
    esp_err_t err = i2c_master_bus_add_device(sensor->bus_handle, &sensor->dev_config, &sensor->dev_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "device_create SUCCESS on %02X", S11_SENSOR_ADDR);
        return err;
    }
    ESP_LOGE(TAG, "device_create FAILED on %02X", S11_SENSOR_ADDR);
    return err;
}

esp_err_t s11_init(s11_t **sensor_ptr, i2c_master_bus_handle_t bus_handle)
{
    s11_t *sensor = NULL;
    esp_err_t err;

    ESP_LOGI(TAG, "Initialize S11");
    sensor = s11_create_master(bus_handle);
    if (sensor == NULL) { 
        ESP_LOGE(TAG, "Could not create S11 driver.");
        return ESP_FAIL;
    }
    *sensor_ptr = sensor;
    if ((err = s11_device_create(sensor)) != ESP_OK) return err;
    if ((err = s11_probe(sensor)) != ESP_OK) return err;
    if ((err = s11_get_dev_info(sensor)) != ESP_OK) return err;
    if ((err = s11_get_cal_data(sensor)) != ESP_OK) return err;
    if ((err = s11_get_dev_meter_ctl(sensor)) != ESP_OK) return err;
    if ((err = s11_get_iir_filter_par(sensor)) != ESP_OK) return err;
    return ESP_OK;
}

void s11_close(s11_t *sensor)
{
    if (sensor != NULL && sensor->dev_handle != NULL) {
        i2c_master_bus_rm_device(sensor->dev_handle);
    }
    vPortFree(sensor);
}

esp_err_t s11_probe(s11_t *sensor)
{
    esp_err_t err = ESP_OK;
    int i;

    ESP_LOGI(TAG, "Probing for S11 sensor on I2C %X", sensor->dev_config.device_address);
    for (i = 0; i < 10; i++) {
        err = i2c_master_probe(sensor->bus_handle, sensor->dev_config.device_address, CONFIG_S11_TIMEOUT);
        if (err == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Probing for S11 SUCCESS after %u retries", i);
    } else {
        ESP_LOGE(TAG, "Probing for S11 FAILED with err=%d", err);
    }
    return err;
}

void s11_wakeup(s11_t *sensor)
{
    i2c_master_probe(sensor->bus_handle, sensor->dev_config.device_address, CONFIG_S11_TIMEOUT);
}

esp_err_t s11_reset(s11_t *sensor)
{
    sensor->last_error = s11_write_u8(sensor, S11_ADDR_RESET, S11_RESET);
    return sensor->last_error;
}

esp_err_t s11_get_error_status(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_ERRSTAT_MSB, &sensor->values.error_status);
    return sensor->last_error;
}

esp_err_t s11_get_firmware_rev(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_FW_REV_MSB, &sensor->dev_info.firmware_rev);
    return sensor->last_error;
}

esp_err_t s11_get_firmware_type(s11_t *sensor)
{
    sensor->last_error = s11_read_u8(sensor, S11_ADDR_FW_TYPE, &sensor->dev_info.firmware_type);
    return sensor->last_error;
}

esp_err_t s11_get_sensor_id(s11_t *sensor)
{
    sensor->last_error = s11_read_u32(sensor, S11_ADDR_SID_MMSB, &sensor->dev_info.sensor_id);
    return sensor->last_error;
}

esp_err_t s11_get_product_code(s11_t *sensor)
{
    sensor->last_error = execute_cmd(sensor, S11_ADDR_PRDCOD_BUF, NULL, 0, (uint8_t *)sensor->dev_info.product_code, 8);
    return sensor->last_error;
}

esp_err_t s11_get_dev_info(s11_t *sensor)
{
    if (s11_get_firmware_rev(sensor) != ESP_OK) return sensor->last_error;
    if (s11_get_firmware_type(sensor) != ESP_OK) return sensor->last_error;
    if (s11_get_sensor_id(sensor) != ESP_OK) return sensor->last_error;
    // FW rev. >= 4.08
    //if (s11_get_product_code(sensor) != ESP_OK) return sensor->last_error;
    return sensor->last_error;
}

esp_err_t s11_get_measurement_mode(s11_t *sensor)
{
    sensor->last_error = s11_read_u8(sensor, S11_ADDR_DEV_MEAS_MODE, &sensor->dev_settings.meas_mode);
    return sensor->last_error;
}

esp_err_t s11_set_measurement_mode(s11_t *sensor, uint8_t meas_mode)
{
    sensor->last_error = s11_write_u8(sensor, S11_ADDR_DEV_MEAS_MODE, meas_mode);
    return sensor->last_error;
}

esp_err_t s11_get_measurement_period(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_DEV_MEAS_PER_MSB, &sensor->dev_settings.measurement_period);
    return sensor->last_error;
}

esp_err_t s11_set_measurement_period(s11_t *sensor, uint16_t meas_period)
{
    sensor->last_error = s11_write_u16(sensor, S11_ADDR_DEV_MEAS_PER_MSB, meas_period);
    return sensor->last_error;
}

esp_err_t s11_get_number_samples(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_DEV_NB_SAMP_MSB, &sensor->dev_settings.meas_nb_samples);
    return sensor->last_error;
}

esp_err_t s11_set_number_samples(s11_t *sensor, uint16_t meas_nb_samples)
{
    sensor->last_error = s11_write_u16(sensor, S11_ADDR_DEV_NB_SAMP_MSB, meas_nb_samples);
    return sensor->last_error;
}

esp_err_t s11_get_abc_period(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_ABC_PER_MSB, &sensor->dev_settings.abc_period);
    return sensor->last_error;
}

esp_err_t s11_set_abc_period(s11_t *sensor, uint16_t abc_period)
{
    sensor->last_error = s11_write_u16(sensor, S11_ADDR_ABC_PER_MSB, abc_period);
    return sensor->last_error;
}

esp_err_t s11_get_abc_target(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_ABC_TARGET_MSB, &sensor->dev_settings.abc_target);
    return sensor->last_error;
}

esp_err_t s11_set_abc_target(s11_t *sensor, uint16_t abc_target)
{
    sensor->last_error = s11_write_u16(sensor, S11_ADDR_ABC_TARGET_MSB, abc_target);
    return sensor->last_error;
}

esp_err_t s11_get_iir_filter_par(s11_t *sensor)
{
    sensor->last_error = s11_read_u8(sensor, S11_ADDR_IIR_FILTER_PAR, &sensor->dev_settings.iir_filter_par);
    return sensor->last_error;
}

esp_err_t s11_set_iir_filter_par(s11_t *sensor, uint8_t iir_filter_par)
{
    sensor->last_error = s11_write_u8(sensor, S11_ADDR_IIR_FILTER_PAR, iir_filter_par);
    return sensor->last_error;
}

esp_err_t s11_get_dev_meter_ctl(s11_t *sensor)
{
    sensor->last_error = s11_read_u8(sensor, S11_ADDR_DEV_METER_CTL, &sensor->dev_settings.dev_meter_ctl);
    return sensor->last_error;
}

esp_err_t s11_set_dev_meter_ctl(s11_t *sensor, uint8_t dev_meter_ctl)
{
    sensor->last_error = s11_write_u8(sensor, S11_ADDR_DEV_METER_CTL, dev_meter_ctl);
    return sensor->last_error;
}

esp_err_t s11_get_address(s11_t *sensor)
{
    sensor->last_error = s11_read_u8(sensor, S11_ADDR_DEV_ADDR, &sensor->dev_settings.address);
    return sensor->last_error;
}

esp_err_t s11_set_address(s11_t *sensor, uint8_t address)
{
    sensor->last_error = s11_write_u8(sensor, S11_ADDR_DEV_ADDR, address);
    return sensor->last_error;
}

esp_err_t s11_get_concentration_scale_factor_nominator(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_CONC_SCL_FAC_NOM, &sensor->dev_settings.concentration_scale_factor_nominator);
    return sensor->last_error;
}

esp_err_t s11_set_concentration_scale_factor_nominator(s11_t *sensor, uint16_t concentration_scale_factor_nominator)
{
    sensor->last_error = s11_write_u16(sensor, S11_ADDR_CONC_SCL_FAC_NOM, concentration_scale_factor_nominator);
    return sensor->last_error;
}

esp_err_t s11_get_concentration_scale_factor_denominator(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_CONC_SCL_FAC_DEN, &sensor->dev_settings.concentration_scale_factor_denominator);
    return sensor->last_error;
}

esp_err_t s11_set_concentration_scale_factor_denominator(s11_t *sensor, uint16_t concentration_scale_factor_denominator)
{
    sensor->last_error = s11_write_u16(sensor, S11_ADDR_CONC_SCL_FAC_DEN, concentration_scale_factor_denominator);
    return sensor->last_error;
}

esp_err_t s11_get_scaled_calibration_target(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_SCLD_CAL_TARGET, &sensor->dev_settings.scaled_calibration_target);
    return sensor->last_error;
}

esp_err_t s11_set_scaled_calibration_target(s11_t *sensor, uint16_t scaled_calibration_target)
{
    sensor->last_error = s11_write_u16(sensor, S11_ADDR_SCLD_CAL_TARGET, scaled_calibration_target);
    return sensor->last_error;
}

esp_err_t s11_get_scaled_measured_concentration_override(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_SCLD_MEAS_CONC_OVR, &sensor->dev_settings.scaled_measured_concentration_override);
    return sensor->last_error;
}

esp_err_t s11_set_scaled_measured_concentration_override(s11_t *sensor, uint16_t scaled_measured_concentration_override)
{
    sensor->last_error = s11_write_u16(sensor, S11_ADDR_SCLD_MEAS_CONC_OVR, scaled_measured_concentration_override);
    return sensor->last_error;
}

esp_err_t s11_get_scaled_abc_target(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_SCLD_ABC_TARGET, &sensor->dev_settings.scaled_abc_target);
    return sensor->last_error;
}

esp_err_t s11_set_scaled_abc_target(s11_t *sensor, uint16_t scaled_abc_target)
{
    sensor->last_error = s11_write_u16(sensor, S11_ADDR_SCLD_ABC_TARGET, scaled_abc_target);
    return sensor->last_error;
}

esp_err_t s11_get_calibration_status(s11_t *sensor)
{
    sensor->last_error = s11_read_u8(sensor, S11_ADDR_CALIBRATION_STATUS, &sensor->dev_settings.calibration_status);
    return sensor->last_error;
}

esp_err_t s11_set_calibration_status(s11_t *sensor, uint8_t calibration_status)
{
    sensor->last_error = s11_write_u8(sensor, S11_ADDR_CALIBRATION_STATUS, calibration_status);
    return sensor->last_error;
}

esp_err_t s11_get_abc_time(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_ABC_TIME, &sensor->dev_settings.abc_time);
    return sensor->last_error;
}

esp_err_t s11_get_abc_par(s11_t *sensor)
{
    sensor->last_error = execute_cmd(sensor, S11_ADDR_ABC_PAR0, NULL, 0, (uint8_t *)&sensor->dev_settings.abc_par, 8);
    return sensor->last_error;
}

esp_err_t s11_set_abc_par(s11_t *sensor)
{
    sensor->last_error = execute_cmd(sensor, S11_ADDR_ABC_PAR0, (uint8_t *)&sensor->dev_settings.abc_par, 8, NULL, 0);
    return sensor->last_error;
}

esp_err_t s11_get_filter_par(s11_t *sensor)
{
    sensor->last_error = execute_cmd(sensor, S11_ADDR_FILTER_PAR0, NULL, 0, (uint8_t *)&sensor->dev_settings.filter_par, 14);
    return sensor->last_error;
}

esp_err_t s11_set_filter_par(s11_t *sensor)
{
    sensor->last_error = execute_cmd(sensor, S11_ADDR_FILTER_PAR0, (uint8_t *)&sensor->dev_settings.filter_par, 14, NULL, 0);
    return sensor->last_error;
}

esp_err_t s11_get_air_pressure_value(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_AIR_PRESS_VALUE, &sensor->dev_settings.air_pressure_value);
    return sensor->last_error;
}

esp_err_t s11_set_air_pressure_value(s11_t *sensor, uint16_t air_pressure_value)
{
    sensor->last_error = s11_write_u16(sensor, S11_ADDR_AIR_PRESS_VALUE, air_pressure_value);
    return sensor->last_error;
}

esp_err_t s11_get_abc_pressure_value(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_ABC_PRESS_VALUE, &sensor->dev_settings.abc_pressure_value);
    return sensor->last_error;
}

esp_err_t s11_set_abc_pressure_value(s11_t *sensor, uint16_t abc_pressure_value)
{
    sensor->last_error = s11_write_u16(sensor, S11_ADDR_ABC_PRESS_VALUE, abc_pressure_value);
    return sensor->last_error;
}

esp_err_t s11_get_sensor_temperature(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_TEMPERATURE, (uint16_t *)&sensor->values.temp);
    return sensor->last_error;
}

esp_err_t s11_get_measurement_count(s11_t *sensor)
{
    sensor->last_error = s11_read_u8(sensor, S11_ADDR_MEAS_CNT, &sensor->dev_status.measurement_count);
    return sensor->last_error;
}

esp_err_t s11_get_measurement_cycle(s11_t *sensor)
{
    sensor->last_error = s11_read_u16(sensor, S11_ADDR_MEAS_CYCLE_TIME, &sensor->dev_status.measurement_cycle_time);
    return sensor->last_error;
}

esp_err_t s11_get_co2(s11_t *sensor)
{
    if ((sensor->last_error = s11_read_u16(sensor, S11_ADDR_CO2_FP, &sensor->values.co2_fp)) != ESP_OK) return sensor->last_error;
    if ((sensor->last_error = s11_read_u16(sensor, S11_ADDR_CO2_P, &sensor->values.co2_p)) != ESP_OK) return sensor->last_error;
    if ((sensor->last_error = s11_read_u16(sensor, S11_ADDR_CO2_F, &sensor->values.co2_f)) != ESP_OK) return sensor->last_error;
    if ((sensor->last_error = s11_read_u16(sensor, S11_ADDR_CO2, &sensor->values.co2)) != ESP_OK) return sensor->last_error;
    return sensor->last_error;
}

esp_err_t s11_read_measurement(s11_t *sensor)
{
    uint8_t buf[S11_ADDR_MD_BUF_LEN];

    sensor->last_error = execute_cmd(sensor, S11_ADDR_MD_BUF, NULL, 0, buf, S11_ADDR_MD_BUF_LEN);
    //ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, sizeof(buf), ESP_LOG_INFO);
    if (sensor->last_error != ESP_OK) return sensor->last_error;
    sensor->dev_status.measurement_count = buf[S11_ADDR_MD_COUNT];
    sensor->dev_status.measurement_cycle_time = get_u16(&buf[S11_ADDR_MD_CYCLE_TIME_MSB]);
    sensor->values.co2_fp = get_u16(&buf[S11_ADDR_MD_CO2_FP_MSB]);
    sensor->values.temp = get_u16(&buf[S11_ADDR_MD_TEMP_MSB]);
    sensor->values.co2_p = get_u16(&buf[S11_ADDR_MD_CO2_P_MSB]);
    sensor->values.co2_f = get_u16(&buf[S11_ADDR_MD_CO2_F_MSB]);
    sensor->values.co2 = get_u16(&buf[S11_ADDR_MD_CO2_MSB]);
    return ESP_OK;
}

esp_err_t s11_get_cal_data(s11_t *sensor)
{
    sensor->last_error = execute_cmd(sensor, S11_ADDR_CAL_BUF, NULL, 0, (uint8_t *)&sensor->cal_data, S11_ADDR_CAL_BUF_LEN);
    ESP_LOGI(TAG, "cal_status=%02X", sensor->cal_data.cal_status);
    ESP_LOGI(TAG, "cal_cmd=%02X", sensor->cal_data.cal_cmd);
    ESP_LOGI(TAG, "cal_target=%02X", sensor->cal_data.cal_target);
    return sensor->last_error;
}

esp_err_t s11_clear_error_status(s11_t *sensor)
{
    sensor->last_error = s11_write_u8(sensor, S11_ADDR_CLR_ERR_STATUS, S11_CLEAR_ERRSTAT_CMD);
    return sensor->last_error;
}

esp_err_t s11_start_single_measurement(s11_t *sensor)
{
    sensor->last_error = s11_write_u8(sensor, S11_ADDR_SINGLE_MEAS, S11_MC_CMD_START_MEAS);
    return sensor->last_error;
}

void s11_dump_dev_info(s11_t *sensor)
{
    ESP_LOGI(TAG, "Firmware rev=%04X", sensor->dev_info.firmware_rev);
    ESP_LOGI(TAG, "Firmware type=%02X", sensor->dev_info.firmware_type);
    ESP_LOGI(TAG, "Sensor ID=%08X", sensor->dev_info.sensor_id);
    ESP_LOGI(TAG, "Meter Control=%02X", sensor->dev_settings.dev_meter_ctl);
    ESP_LOGI(TAG, "IIR Filter Par=%d", sensor->dev_settings.iir_filter_par);
}

void s11_dump_error_status(s11_t *sensor)
{
    s11_values_t *values = &sensor->values;
    char buf[100];
    char *s = buf;

    buf[0] = 0;
    if (values->error_status & S11_ERRSTAT_MSK_FATAL) s += sprintf(s, " Fatal");
    if (values->error_status & S11_ERRSTAT_MSK_I2C) s += sprintf(s, " I2C");
    if (values->error_status & S11_ERRSTAT_MSK_ALG) s += sprintf(s, " Alg");
    if (values->error_status & S11_ERRSTAT_MSK_CAL) s += sprintf(s, " Cal");
    if (values->error_status & S11_ERRSTAT_MSK_SLF_DIA) s += sprintf(s, " SlfDia");
    if (values->error_status & S11_ERRSTAT_MSK_OUT_RNG) s += sprintf(s, " OutRng");
    if (values->error_status & S11_ERRSTAT_MSK_MEM) s += sprintf(s, " Mem");
    if (values->error_status & S11_ERRSTAT_MSK_NO_MEAS) s += sprintf(s, " NoMeas");
    if (values->error_status & S11_ERRSTAT_MSK_LO_V) s += sprintf(s, " LoV");
    if (values->error_status & S11_ERRSTAT_MSK_MEAS_TO) s += sprintf(s, " MeasTo");
    if (values->error_status & S11_ERRSTAT_MSK_SIG_LVL) s += sprintf(s, " SigLvl");
    if (values->error_status & S11_ERRSTAT_MSK_SCL_FAC) s += sprintf(s, " SclFac");
    ESP_LOGI(TAG, "error_status=%04X%s", values->error_status, buf);
}

void s11_dump_values(s11_t *sensor, bool force)
{
    if (force || sensor->debug & 1) {
        s11_values_t *values = &sensor->values;

        ESP_LOGI(TAG, "co2_fp=%u ppm  co2_p=%u ppm  co2_f=%u ppm  co2=%u ppm  temp=%.1f °C",
                 values->co2_fp, values->co2_p, values->co2_f, values->co2, (float)values->temp * 0.01);
        s11_dump_error_status(sensor);
    }
}
