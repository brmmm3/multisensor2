/**
 * S11
 *
 * MIT License
 *
 * Copyright (C) 2026 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Default
#define S11_DEF_I2C_ADDR           (0x68) ///< Default I2C address

// Error Status register (ERRSTAT)
#define S11_ADDR_ERRSTAT_MSB       (0x00) ///< Register address
#define S11_ADDR_ERRSTAT_LSB       (0x01) ///< Register address

// Measurement data register range (MD)
// Reg addr - measurement data array (R)
#define S11_ADDR_MD_BUF             (0x06) ///< First register address
#define S11_ADDR_MD_BUF_LEN         (16)   ///< Length [bytes]
// Offsets
#define S11_ADDR_MD_CO2_FP_MSB      (0)  ///< Offset - CO2 value filtered & pressure compensated MSB
#define S11_ADDR_MD_CO2_FP_LSB      (1)  ///< Offset - CO2 value filtered & pressure compensated LSB
#define S11_ADDR_MD_TEMP_MSB        (2)  ///< Offset - PCB temperature MSB
#define S11_ADDR_MD_TEMP_LSB        (3)  ///< Offset - PCB temperature LSB
#define S11_ADDR_MD_COUNT           (7)  ///< Offset - Measurement counts
#define S11_ADDR_MD_CYCLE_TIME_MSB  (8)  ///< Offset - Measurement cycle time MSB
#define S11_ADDR_MD_CYCLE_TIME_LSB  (9)  ///< Offset - Measurememnt cycle time LSB
#define S11_ADDR_MD_CO2_P_MSB       (10) ///< Offset - CO2 value - pressure compensated
#define S11_ADDR_MD_CO2_P_LSB       (11) ///< Offset - CO2 value - pressure compensated
#define S11_ADDR_MD_CO2_F_MSB       (12) ///< Offset - CO2 value - filtered MSB
#define S11_ADDR_MD_CO2_F_LSB       (13) ///< Offset - CO2 value - filtered LSB
#define S11_ADDR_MD_CO2_MSB         (14) ///< Offset - CO2 value MSB
#define S11_ADDR_MD_CO2_LSB         (15) ///< Offset - CO2 value LSB

#define S11_ADDR_CO2_FP             (0x06) ///< Measured concentration Filtered & pressure compensated
#define S11_ADDR_CO2_P              (0x10) ///< Measured concentration Unfiltered & pressure compensated
#define S11_ADDR_CO2_F              (0x12) ///< Measured concentration Filtered
#define S11_ADDR_CO2                (0x14) ///< Measured concentration Unfiltered

#define S11_ADDR_TEMPERATURE        (0x08) ///< Sensor temperature

#define S11_ADDR_MEAS_CNT           (0x0D) ///< Measurement count

#define S11_ADDR_MEAS_CYCLE_TIME    (0x0E) ///< Measurement cycle time

// Elapsed time counter: counts while the sensor is
// powered up. Unsigned 32 bit value, unit hours.
#define S11_ADDR_ETC_MMSB           (0x2A) ///< First register address
#define S11_ADDR_ETC_MLSB           (0x2B)
#define S11_ADDR_ETC_LMSB           (0x2C)
#define S11_ADDR_ETC_LLSB           (0x2D)

// Firmware type
#define S11_ADDR_FW_TYPE            (0x2F)

// Info register range (INF)
#define S11_ADDR_INF_BUF            (0x38) ///< First register address
#define S11_ADDR_INF_BUF_LEN        (6)    ///< Length [bytes]
// Offsets
#define S11_ADDR_INF_FW_REV_MSB     (0) ///< Offset - Firmware revision MSB
#define S11_ADDR_INF_FW_REV_LSB     (1) ///< Offset - Firmware revision LSB
#define S11_ADDR_INF_ID_MMSB        (2) ///< Offset - Device ID MMSB
#define S11_ADDR_INF_ID_MLSB        (3) ///< Offset - Device ID MLSB
#define S11_ADDR_INF_ID_LMSB        (4) ///< Offset - Device ID LMSB
#define S11_ADDR_INF_ID_LLSB        (5) ///< Offset - Device ID LLSB

// Firmware revision
#define S11_ADDR_FW_REV_MSB         (0x38)
#define S11_ADDR_FW_REV_LSB         (0x39)

// Sensor ID
#define S11_ADDR_SID_MMSB           (0x3A)
#define S11_ADDR_SID_MLSB           (0x3B)
#define S11_ADDR_SID_LMSB           (0x3C)
#define S11_ADDR_SID_LLSB           (0x3D)

// Product Code
#define S11_ADDR_PRDCOD_BUF         (0x70) ///< First register address
#define S11_ADDR_PRDCOD_BUF_LEN     (16)   ///< Length [bytes]

// Calibration data register range (CAL)
#define S11_ADDR_CAL_BUF            (0x81)
#define S11_ADDR_CAL_BUF_LEN        (5)
// Offsets
#define S11_ADDR_CAL_STATUS         (0)
#define S11_ADDR_CAL_CMD_MSB        (1)
#define S11_ADDR_CAL_CMD_LSB        (2)
#define S11_ADDR_CAL_TGT_MSB        (3)
#define S11_ADDR_CAL_TGT_LSB        (4)

// CO2 override register (CO2)
#define S11_ADDR_CO2_OVERRIDE       (0x86) ///< CO2 Override value [ppm]

#define S11_ADDR_ABC_TIME           (0x88) ///< Time passed since last ABC calibration in hours.

#define S11_ADDR_ABC_PAR0           (0x8A)
#define S11_ADDR_ABC_PAR_LEN        (8)

// CO2 override disable command
#define S11_CO2_CMD_OVERRIDE_DIS    (32762) ///< Write this value to CO2 override register to disable override

// Device configuration register range
// EE = EEPROM register, limited no of write cycles
#define S11_ADDR_DEV_MEAS_MODE      (0x95) ///< Measurement mode register (EE)
#define S11_ADDR_DEV_MEAS_PER_MSB   (0x96) ///< Measurement period register MSB (EE)
#define S11_ADDR_DEV_MEAS_PER_LSB   (0x97) ///< Measurement period register LSB (EE)
#define S11_ADDR_DEV_NB_SAMP_MSB    (0x98) ///< Number of samples register MSB (EE)
#define S11_ADDR_DEV_NB_SAMP_LSB    (0x99) ///< Number of samples register LSB (EE)
#define S11_ADDR_ABC_PER_MSB        (0x9A) ///< ABC period register MSB (EE)
#define S11_ADDR_ABC_PER_LSB        (0x9B) ///< ABC period register LSB (EE)

// Clear error status
#define S11_ADDR_CLR_ERR_STATUS     (0x9D) ///< Register address
#define S11_CLEAR_ERRSTAT_CMD       (0x01) ///< Command

#define S11_ADDR_ABC_TARGET_MSB     (0x9E) ///< ABC target register MSB (EE)
#define S11_ADDR_ABC_TARGET_LSB     (0x9F) ///< ABC target register LSB (EE)
#define S11_ADDR_IIR_FILTER_PAR     (0xA1) ///< Static IIR filter parameter register (EE)

// Device reset
#define S11_ADDR_RESET              (0xA3) ///< Register address
#define S11_RESET                   (0xFF) ///< Command

#define S11_ADDR_DEV_METER_CTL      (0xA5) ///< Meter control register (EE)

#define S11_ADDR_DEV_ADDR           (0xA7) ///< Device address register, I2C/Modbus (EE)

#define S11_ADDR_CONC_SCL_FAC_NOM   (0xA8) ///< Concentration Scale Factor Nominator

#define S11_ADDR_CONC_SCL_FAC_DEN   (0xAA) ///< Concentration Scale Factor Denominator

#define S11_ADDR_SCLD_CAL_TARGET    (0xAC) ///< Scaled Calibration Target

#define S11_ADDR_SCLD_MEAS_CONC_OVR (0xAE) ///< Scaled Measurement Concentration Override

#define S11_ADDR_SCLD_ABC_TARGET    (0xB0) ///< Scaled ABC Target

#define S11_ADDR_CALIBRATION_STATUS (0xC1)

// Single Measurement command register
#define S11_ADDR_SINGLE_MEAS        (0xC3)
#define S11_MC_CMD_START_MEAS       (0x01) ///< Start measurement command

#define S11_ADDR_FILTER_PAR0        (0xCE) ///< Filter Par0
#define S11_ADDR_FILTER_PAR_LEN     (14)   ///< Filter Par0-6

#define S11_ADDR_AIR_PRESS_VALUE    (0xDC) ///< Barometric Air Pressure Value

#define S11_ADDR_ABC_PRESS_VALUE    (0xDE) ///< ABC Barometric Pressure Value

// Error Status Register (ERRSTAT) - Bitmask
#define S11_ERRSTAT_MSK_FATAL       (0x0001)  ///< Fatal error
#define S11_ERRSTAT_MSK_I2C         (0x0002)  ///< I2C error
#define S11_ERRSTAT_MSK_ALG         (0x0004)  ///< Algorithm error
#define S11_ERRSTAT_MSK_CAL         (0x0008)  ///< Calibration error
#define S11_ERRSTAT_MSK_SLF_DIA     (0x0010)  ///< Self diagnostics error
#define S11_ERRSTAT_MSK_OUT_RNG     (0x0020)  ///< Out of range
#define S11_ERRSTAT_MSK_MEM         (0x0040)  ///< Memory error
#define S11_ERRSTAT_MSK_NO_MEAS     (0x0080)  ///< No measurement completed
#define S11_ERRSTAT_MSK_LO_V        (0x0100)  ///< Low internal regulated voltage
#define S11_ERRSTAT_MSK_MEAS_TO     (0x0200)  ///< Measurement timeout
#define S11_ERRSTAT_MSK_SIG_LVL     (0x0400)  ///< Abnormal signal level
#define S11_ERRSTAT_MSK_SCL_FAC     (0x8000)  ///< Scale factor error

// Calibration Status Register (CAL_STAT) - Bitmask
#define S11_CAL_STAT_MSK_FACT_CAL   (0x02) ///< Factory calibration completed
#define S11_CAL_STAT_MSK_ABC_CAL    (0x04) ///< ABC calibration completed
#define S11_CAL_STAT_MSK_TARGET_CAL (0x08) ///< Target calibration completed
#define S11_CAL_STAT_MSK_BKG_CAL    (0x10) ///< Background calibration completed
#define S11_CAL_STAT_MSK_ZERO_CAL   (0x20) ///< Calibration status - Zero calibration completed

// Calibration Command Register (CAL_CMD) - Calibration types
#define S11_CAL_CMD_FACT_CAL       (0x7C02) ///< Restore factory calibration
#define S11_CAL_CMD_ABC_CAL        (0x7C03) ///< Forced ABC calibration
#define S11_CAL_CMD_TARGET_CAL     (0x7C05) ///< Target value calibration
#define S11_CAL_CMD_BKG_CAL        (0x7C06) ///< Background calibration [400ppm]
#define S11_CAL_CMD_ZERO_CAL       (0x7C07) ///< Restore factory calibration [0ppm]

// Meter control Register (MET_CTL)(EE) - Bitmask
#define S11_MET_CTL_MSK_NRDY_EN    (0x01) ///< Meter controls - Enable nReady pin
#define S11_MET_CTL_MSK_ABC_EN     (0x02) ///< Meter controls - Enable ABC calibration
#define S11_MET_CTL_MSK_SIIR_EN    (0x04) ///< Meter controls - Enable static IIR filter
#define S11_MET_CTL_MSK_DIIR_EN    (0x08) ///< Meter controls - Enable dynamic IIR filter
#define S11_MET_CTL_MSK_PCOMP_EN   (0x10) ///< Meter controls - Enable pressure compensation

// Measurement Mode Register (MM_STATE) - Values
#define S11_MM_STATE_CONT_MEAS     (0) ///< Continuous measurements
#define S11_MM_STATE_SGL_MEAS      (1) ///< Single measurement

// Driver Error Codes
#define S11_E_OK                    (0x00) ///< Function executed with no error
#define S11_E_NULL_PTR              (0x01) ///< One or more function pointers are invalid
#define S11_E_I2C                   (0x02) ///< I2C communication error
#define S11_E_VAL_OUT_OF_LIMIT      (0x03) ///< One or more input parameters are outside valid range
#define S11_E_CALIBRATION           (0x04) ///< Calibration error

// Shared I2C buffer length
#define S11_BUF_LEN                 (32)

// Time constants
#define S11_EE_WR_DELAY_MS           (25) ///< Wait time between EEPROM writes
#define S11_RESTART_DELAY_MS         (35) ///< Wait time for sensor to boot

#ifdef __cplusplus
};
#endif
