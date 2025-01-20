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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"

#define SPS30_DEV_INFO_MAX_LEN  8
#define SPS30_SERIAL_MAX_LEN    32

/** The fan is switched on but not running */
#define SPS30_DEVICE_FAN_ERROR(s)       ((s & (1 << 4)) != 0)
/** The laser current is out of range */
#define SPS30_DEVICE_LASER_ERROR(s)     ((s & (1 << 5)) != 0)
/** The fan speed is out of range */
#define SPS30_DEVICE_FAN_SPEED_WRN(s)   ((s & (1 << 21)) != 0)

typedef struct sps30_values_s {
    float mc_1p0;
    float mc_2p5;
    float mc_4p0;
    float mc_10p0;
    float nc_0p5;
    float nc_1p0;
    float nc_2p5;
    float nc_4p0;
    float nc_10p0;
    float typical_particle_size;
} sps30_values_t;

typedef struct sps30_s {
    // I2C master handle via port with configuration
    i2c_master_dev_handle_t dev_handle;
    // I2C master configuration
    i2c_device_config_t dev_cfg;
    // I2C master handle via port
    i2c_master_bus_handle_t bus_handle;
    // Serial number
    char device_info[SPS30_DEV_INFO_MAX_LEN + 1];
    // Serial number
    char serial[SPS30_SERIAL_MAX_LEN + 1];
    uint16_t firmware_version;
    // Auto cleaning interval
    uint32_t autoclean_interval;
    uint32_t status;
    sps30_values_t values;
} sps30_t;

esp_err_t sps30_init(sps30_t **sensor_ptr, i2c_master_bus_handle_t bus_handle);

esp_err_t sps30_device_create(sps30_t *sensor);

sps30_t *sps30_create_master(i2c_master_bus_handle_t bus_handle);

esp_err_t sps30_device_init(sps30_t** sensor_ptr, i2c_master_bus_handle_t bus_handle);

void sps30_close(sps30_t *sensor);

/**
 * sps30_get_driver_version() - Return the driver version
 * Return:  Driver version string
 */
const char* sps30_get_driver_version(void);

/**
 * sps30_probe() - check if SPS sensor is available and initialize it
 *
 * Note that Pin 4 must be pulled to ground for the sensor to operate in i2c
 * mode (this driver). When left floating, the sensor operates in UART mode
 * which is not compatible with this i2c driver.
 *
 * Return:  0 on success, an error code otherwise
 */
esp_err_t sps30_probe(sps30_t *sensor);

/**
 * sps30_get_firmware_version - read the firmware version
 * @major:  Memory where the firmware major version is written into
 * @minor:  Memory where the firmware minor version is written into
 *
 * Return:  0 on success, an error code otherwise
 */
esp_err_t sps30_get_firmware_version(sps30_t *sensor);

esp_err_t sps30_get_device_info(sps30_t *sensor);

/**
 * sps30_get_serial() - retrieve the serial number
 *
 * Note that serial must be discarded when the return code is non-zero.
 *
 * @serial: Memory where the serial number is written into as hex string (zero
 *          terminated). Must be at least SPS30_SERIAL_MAX_LEN long.
 * Return:  0 on success, an error code otherwise
 */
esp_err_t sps30_get_serial(sps30_t *sensor);

/**
 * sps30_start_measurement() - start measuring
 *
 * Once the measurement is started, measurements are retrievable once per second
 * with sps30_read_measurement.
 *
 * Return:  0 on success, an error code otherwise
 */
esp_err_t sps30_start_measurement(sps30_t *sensor);

/**
 * sps30_stop_measurement() - stop measuring
 *
 * Stops measuring and puts the sensor back into idle mode.
 *
 * Return:  0 on success, an error code otherwise
 */
esp_err_t sps30_stop_measurement(sps30_t *sensor);

/**
 * sps30_read_datda_ready() - reads the current data-ready flag
 *
 * The data-ready flag indicates that new (not yet retrieved) measurements are
 * available
 *
 * @data_ready: Memory where the data-ready flag (0|1) is stored.
 * Return:      0 on success, an error code otherwise
 */
bool sps30_read_data_ready(sps30_t *sensor);

/**
 * sps30_read_measurement() - read a measurement
 *
 * Read the last measurement.
 *
 * Return:  0 on success, an error code otherwise
 */
esp_err_t sps30_read_measurement(sps30_t *sensor);

/**
 * sps30_get_fan_auto_cleaning_interval() - read the current(*) auto-cleaning
 * interval
 *
 * Note that interval_seconds must be discarded when the return code is
 * non-zero.
 *
 * (*) Note that due to a firmware bug on FW<2.2, the reported interval is only
 * updated on sensor restart/reset. If the interval was thus updated after the
 * last reset, the old value is still reported. Power-cycle the sensor or call
 * sps30_reset() first if you need the latest value.
 *
 * @interval_seconds:   Memory where the interval in seconds is stored
 * Return:              0 on success, an error code otherwise
 */
esp_err_t sps30_get_fan_auto_cleaning_interval(sps30_t *sensor);

/**
 * sps30_set_fan_auto_cleaning_interval() - set the current auto-cleaning
 * interval
 *
 * @interval_seconds:   Value in seconds used to sets the auto-cleaning
 *                      interval, 0 to disable auto cleaning
 * Return:              0 on success, an error code otherwise
 */
esp_err_t sps30_set_fan_auto_cleaning_interval(sps30_t *sensor, uint32_t interval_seconds);

/**
 * sps30_start_manual_fan_cleaning() - Immediately trigger the fan cleaning
 *
 * Note that this command can only be run when the sensor is in measurement
 * mode, i.e. after sps30_start_measurement() without subsequent
 * sps30_stop_measurement().
 *
 * Return:          0 on success, an error code otherwise
 */
esp_err_t sps30_start_manual_fan_cleaning(sps30_t *sensor);

/**
 * sps30_reset() - reset the SGP30
 *
 * The sensor reboots to the same state as before the reset but takes a few
 * seconds to resume measurements.
 *
 * The caller should wait at least SPS30_RESET_DELAY_USEC microseconds before
 * interacting with the sensor again in order for the sensor to restart.
 * Interactions with the sensor before this delay might fail.
 *
 * Note that the interface-select configuration is reinterpreted, thus Pin 4
 * must be pulled to ground during the reset period for the sensor to remain in
 * i2c mode.
 *
 * Return:          0 on success, an error code otherwise
 */
esp_err_t sps30_reset(sps30_t *sensor);

/**
 * sps30_sleep() - Send the (idle) sensor to sleep
 *
 * The sensor will reduce its power consumption to a minimum, but must be woken
 * up again with sps30_wake_up() prior to resuming operations. It will only
 * suceed if the sensor is idle, i.e. not currently measuring.
 * Note that this command only works on firmware 2.0 or more recent.
 *
 * Return:          0 on success, an error code otherwise (e.g. if the firmware
 *                  does not support the command)
 */
esp_err_t sps30_sleep(sps30_t *sensor);

/**
 * sps30_wake_up() - Wake up the sensor from sleep
 *
 * Use this command to wake up the sensor from sleep mode into idle mode.
 * Note that this command only works on firmware 2.0 or more recent.
 *
 * Return:          0 on success, an error code otherwise (e.g. if the firmware
 *                  does not support the command)
 */
esp_err_t sps30_wake_up(sps30_t *sensor);

/**
 * sps30_read_device_status_register() - Read the Device Status Register
 *
 * Reads the Device Status Register which reveals info, warnings and errors
 * about the sensor's current operational state. Note that the flags are
 * self-clearing, i.e. they reset to 0 when the condition is resolved.
 * Note that this command only works on firmware 2.2 or more recent.
 *
 * @device_status_flags:    Memory where the device status flags are written
 *                          into
 *
 * Return:          0 on success, an error code otherwise (e.g. if the firmware
 *                  does not support the command)
 */
esp_err_t sps30_read_device_status_register(sps30_t *sensor);

esp_err_t sps30_clear_device_status_register(sps30_t *sensor);

#ifdef __cplusplus
}
#endif
