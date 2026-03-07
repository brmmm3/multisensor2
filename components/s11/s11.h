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

#pragma once


// C++
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <esp_err.h>
#include "driver/i2c_master.h"

#include "s11_defs.h"


typedef struct {
	/// 06-07: CO2 value filtered and pressure compensated [ppm] (R)
	uint16_t co2_fp;
	/// 10-11: CO2 value unfiltered & pressure compensated [ppm] (R)
	uint16_t co2_p;
	/// 12-13: CO2 value filtered [ppm] (R)
	uint16_t co2_f;
	/// 14-15: CO2 value unfiltered [ppm] (R)
	uint16_t co2;
	/// 08-09: Chip temperature [0.01 deg C] (R)
  	int16_t temp;
	/// 00-01: Error status (R)
	uint16_t error_status;
} s11_values_t;

typedef struct {	
	/// 3A-3D: Sensor ID (R)
	uint32_t sensor_id;
	/// 38-39: Firmware revision, MSB=main, LSB=sub (R)
	uint16_t firmware_rev;
	/// 2F: Firmware type (R)
	uint8_t firmware_type;
	/// 70-7F: Product code (R)
	uint8_t product_code[8];
} s11_dev_info_t;

typedef struct {	
	uint8_t old_measurement_count;
	/// 0D: Measurement count (0-255). Increases after each measurement. (R)
	uint8_t measurement_count;
	/// 0E-0F: Measurement cycle time. 0 when new measurement starts. [s] (R)
	uint16_t measurement_cycle_time;
} s11_dev_status_t;

typedef struct {
	/// Calibration command (W)
	uint16_t cal_cmd;
	/// Calibration target [ppm] (W)
	uint16_t cal_target;
	/// Calibration status (R)
	uint8_t cal_status;
} s11_cal_data_t;

typedef struct {
	uint16_t concentration_scale_factor_nominator;
	uint16_t concentration_scale_factor_denominator;
	uint16_t scaled_calibration_target;
	uint16_t scaled_measured_concentration_override;
	uint16_t air_pressure_value;
	/// Measurement Period [seconds] (EE) (R/W) 
	uint16_t measurement_period;
	/// Number of samples (EE) (R/W)
	uint16_t meas_nb_samples;
	/// Scaled ABC Target
	uint16_t scaled_abc_target;
	/// ABC pressure value
	uint16_t abc_pressure_value;
	/// ABC period [hours] (EE) (R/W)
	uint16_t abc_period;	
	/// ABC target [ppm co2] (EE) (R/W)
	uint16_t abc_target;	
	/// ABC Time [h] (W)
	uint16_t abc_time;
	uint16_t abc_par[4];
	uint16_t filter_par[7];
	/// Measurement mode (EE) (R/W)
	uint8_t meas_mode;
	/// Static IIR filter parameter (EE) (R/W)
	uint8_t iir_filter_par;	
	/// Meter control (EE) (R/W)
	uint8_t dev_meter_ctl;
	/// MB/I2c address (EE) (R/W)
	uint8_t address;
	uint8_t calibration_status;
} s11_dev_settings_t;

typedef struct s11_s {
	// I2C master handle via port with configuration
	i2c_master_dev_handle_t dev_handle;
	// I2C master configuration
	i2c_device_config_t dev_config;
	// I2C master handle via port
	i2c_master_bus_handle_t bus_handle;

	s11_values_t values;
	/// Sensor information
	s11_dev_info_t dev_info;
	/// Sensor status
	s11_dev_status_t dev_status;
	/// Calibration settings
	s11_cal_data_t cal_data;
	/// Device settings
	s11_dev_settings_t dev_settings;
	/// Last communication error
	esp_err_t last_error;
	/// Mask for debugging output
	uint8_t debug;
} s11_t;

esp_err_t s11_init(s11_t **sensor_ptr, i2c_master_bus_handle_t bus_handle);

void s11_close(s11_t *sensor);

esp_err_t s11_probe(s11_t *sensor);

void s11_wakeup(s11_t *sensor);

esp_err_t s11_reset(s11_t *sensor);

esp_err_t s11_get_error_status(s11_t *sensor);

esp_err_t s11_get_firmware_rev(s11_t *sensor);

esp_err_t s11_get_firmware_type(s11_t *sensor);

esp_err_t s11_get_sensor_id(s11_t *sensor);

esp_err_t s11_get_product_code(s11_t *sensor);

esp_err_t s11_get_dev_info(s11_t *sensor);

esp_err_t s11_get_measurement_mode(s11_t *sensor);

esp_err_t s11_set_measurement_mode(s11_t *sensor, uint8_t meas_mode);

esp_err_t s11_get_measurement_period(s11_t *sensor);

esp_err_t s11_set_measurement_period(s11_t *sensor, uint16_t meas_period);

esp_err_t s11_get_number_samples(s11_t *sensor);

esp_err_t s11_set_number_samples(s11_t *sensor, uint16_t meas_nb_samples);

esp_err_t s11_get_abc_period(s11_t *sensor);

esp_err_t s11_set_abc_period(s11_t *sensor, uint16_t abc_period);

esp_err_t s11_get_abc_target(s11_t *sensor);

esp_err_t s11_set_abc_target(s11_t *sensor, uint16_t abc_target);

esp_err_t s11_get_iir_filter_par(s11_t *sensor);

esp_err_t s11_set_iir_filter_par(s11_t *sensor, uint8_t iir_filter_par);

esp_err_t s11_get_dev_meter_ctl(s11_t *sensor);

esp_err_t s11_set_dev_meter_ctl(s11_t *sensor, uint8_t dev_meter_ctl);

esp_err_t s11_get_address(s11_t *sensor);

esp_err_t s11_set_address(s11_t *sensor, uint8_t address);

esp_err_t s11_get_concentration_scale_factor_nominator(s11_t *sensor);

esp_err_t s11_set_concentration_scale_factor_nominator(s11_t *sensor, uint16_t concentration_scale_factor_nominator);

esp_err_t s11_get_concentration_scale_factor_denominator(s11_t *sensor);

esp_err_t s11_set_concentration_scale_factor_denominator(s11_t *sensor, uint16_t concentration_scale_factor_denominator);

esp_err_t s11_get_scaled_calibration_target(s11_t *sensor);

esp_err_t s11_set_scaled_calibration_target(s11_t *sensor, uint16_t scaled_calibration_target);

esp_err_t s11_get_scaled_measured_concentration_override(s11_t *sensor);

esp_err_t s11_set_scaled_measured_concentration_override(s11_t *sensor, uint16_t scaled_measured_concentration_override);

esp_err_t s11_get_scaled_abc_target(s11_t *sensor);

esp_err_t s11_set_scaled_abc_target(s11_t *sensor, uint16_t scaled_abc_target);

esp_err_t s11_get_calibration_status(s11_t *sensor);

esp_err_t s11_set_calibration_status(s11_t *sensor, uint8_t calibration_status);

esp_err_t s11_get_abc_time(s11_t *sensor);

esp_err_t s11_get_abc_par(s11_t *sensor);

esp_err_t s11_set_abc_par(s11_t *sensor);

esp_err_t s11_get_filter_par(s11_t *sensor);

esp_err_t s11_set_filter_par(s11_t *sensor);

esp_err_t s11_get_air_pressure_value(s11_t *sensor);

esp_err_t s11_set_air_pressure_value(s11_t *sensor, uint16_t air_pressure_value);

esp_err_t s11_get_abc_pressure_value(s11_t *sensor);

esp_err_t s11_set_abc_pressure_value(s11_t *sensor, uint16_t abc_pressure_value);

esp_err_t s11_get_sensor_temperature(s11_t *sensor);

esp_err_t s11_get_measurement_count(s11_t *sensor);

esp_err_t s11_get_measurement_cycle(s11_t *sensor);

esp_err_t s11_get_co2(s11_t *sensor);

esp_err_t s11_read_measurement(s11_t *sensor);

esp_err_t s11_get_cal_data(s11_t *sensor);

esp_err_t s11_clear_error_status(s11_t *sensor);

esp_err_t s11_start_single_measurement(s11_t *sensor);

void s11_dump_dev_info(s11_t *sensor);

void s11_dump_error_status(s11_t *sensor);

void s11_dump_values(s11_t *sensor, bool force);

#ifdef __cplusplus
};
#endif
