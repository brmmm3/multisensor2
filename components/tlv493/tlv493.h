/**
 * RTC
 *
 * MIT License
 *
 * Copyright (C) 2024 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#pragma once

#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "driver/i2c_master.h"

#define I2C_TLV493D_CONFIG_DEFAULT {                          \
    .dev_config.device_address = I2C_TLV493D_DEV_ADDR_LO,    \
    .dev_config.scl_speed_hz   = I2C_TLV493D_SCL_SPEED_HZ,   \
    .parity_test_enabled       = true,                       \
    .power_mode                = I2C_TLV493D_LOW_POWER_MODE, \
    .irq_pin_enabled           = true }


typedef enum tlv_493d_power_modes_tag {
    TLV493D_POWER_DOWN_MODE,        /*!< 0, 0, 0, 1000 */
    TLV493D_FAST_MODE,              /*!< 1, 0, 0, 0 */
    TLV493D_LOW_POWER_MODE,         /*!< 0, 1, 1, 10 */
    TLV493D_ULTRA_LOW_POWER_MODE,   /*!< 0, 1, 0, 100 */
    TLV493D_MASTER_CONTROLLED_MODE  /*!< 1, 1, 1, 10 */
} tlv_493d_power_modes_t;

typedef enum tlv493d_channel_conversions_tag {
    TLV493D_CHANNEL_CONV_COMPLETED    = (0b00),
    TLV493D_CHANNEL_Y_CONV_ONGOING    = (0b01),
    TLV493D_CHANNEL_Z_CONV_ONGOING    = (0b10),
    TLV493D_CHANNEL_TEMP_CONV_ONGOING = (0b11),
} tlv493d_channel_conversions_t;

typedef enum tlv493d_i2c_addresses_tag {
    TLV493D_I2C_ADDRESS_00 = (0b00),
    TLV493D_I2C_ADDRESS_01 = (0b01),
    TLV493D_I2C_ADDRESS_10 = (0b10),
    TLV493D_I2C_ADDRESS_11 = (0b11),
} tlv493d_i2c_addresses_t;

typedef enum tlv493d_low_power_periods_tag {
    TLV493D_LOW_POWER_PERIOD_100MS = (0b0),
    TLV493D_LOW_POWER_PERIOD_12MS  = (0b1),
} tlv493d_low_power_periods_t;


typedef union __attribute__((packed)) {
    struct {
        tlv493d_channel_conversions_t channel:2;    /*!< channel conversion status           (bit:0-1)  */
        uint8_t frame_counter:2;                    /*!< frame counter     (bit:2-3) */
        uint8_t temperature_msb:4;                  /*!< temperature msb   (bit:4-7) */
    } bits;
    uint8_t reg;
} tlv493d_temperature_msb_register_t;

typedef union __attribute__((packed)) {
    struct {
        uint8_t by_lsb:4;  /*!< by lsb         (bit:0-3)  */
        uint8_t bx_lsb:4;  /*!< bx lsb         (bit:4-7) */
    } bits;
    uint8_t reg;
} tlv493d_bx_by_lsb_register_t;

typedef union __attribute__((packed)) {
    struct {
        uint8_t bz_lsb:4;           /*!< bz lsb           (bit:0-3)  */
        bool power_down_flag:1;     /*!< power-down flag, bx, by, bz and temperature conversion completed when true     (bit:4) */
        bool parity_fuse_flag:1;    /*!< parity fuse flag, fuse setup is okay with true (PT bit must be enabled MOD2)     (bit:5) */
        bool test_mode_flag:1;      /*!< test-mode flag, when false the data is valid   (bit:6) */
        uint8_t reserved:1;         /*!< reserved, do not modify   (bit:7) */
    } bits;
    uint8_t reg;
} tlv493d_bz_lsb_register_t;

typedef union __attribute__((packed)) {
    struct {
        uint8_t factory_setting:8;    /*!< factory setting - device specific  0x00 (reset)         (bit:0-7)  */
    } bits;
    uint8_t reg;
} tlv493d_reserved1_register_t;

typedef union __attribute__((packed)) {
    struct {
        bool low_power_mode_enabled:1;  /*!< low power mode enabled when true           (bit:0)  */
        bool fast_mode_enabled:1;       /*!< fast mode enabled when true                (bit:1) */
        bool irq_pin_enabled:1;         /*!<  interrupt pin assertion when true         (bit:2) */
        uint8_t factory_setting:2;      /*!< factory setting - device specific 0x07     (bit:3-4) */
        tlv493d_i2c_addresses_t i2c_slave_address:2;  /*!< defines slave address in bus configuration   (bit:5-6) */
        uint8_t parity:1;               /*!< parity of configuration map   (bit:7) */
    } bits;
    uint8_t reg;
} tlv493d_mode1_register_t;

typedef union __attribute__((packed)) {
    struct {
        uint8_t factory_setting:8;    /*!< factory setting - device specific 0x08          (bit:0-7)  */
    } bits;
    uint8_t reg;
} tlv493d_reserved2_register_t;

typedef union __attribute__((packed)) {
    struct {
        uint8_t factory_setting:5;      /*!< factory setting - device specific 0x09           (bit:0-4)  */
        bool parity_test_enabled:1;     /*!< parity test enabled when true     (bit:5) */
        tlv493d_low_power_periods_t low_power_period:1; /*!<  low power period, 12ms or 100ms   (bit:6) */
        bool temperature_disabled:1;    /*!<  temperature measurement is disabled when true  (bit:7) */
    } bits;
    uint8_t reg;
} tlv493d_mode2_register_t;

typedef union __attribute__((packed)) {
    struct {
        uint8_t reserved1:3;          /*!< reserved           (bit:0-2)  */
        uint8_t factory_setting:2; /*!< factory setting - device spefici (bit:3-4)  */
        uint8_t reserved2:3;  /*!< reserved   (bit:5-7) */
    } bits;
    uint8_t reg;
} tlv493d_factorysetting1_register_t;

typedef union __attribute__((packed)) {
    struct {
        uint8_t factory_setting:8; /*!< factory setting - device spefici (bit:0-7)  */
    } bits;
    uint8_t reg;
} tlv493d_factorysetting2_register_t;

typedef union __attribute__((packed)) {
    struct {
        uint8_t factory_setting:5; /*!< factory setting - device spefici (bit:0-4)  */
        uint8_t reserved:3;  /*!< reserved   (bit:5-7) */
    } bits;
    uint8_t reg;
} tlv493d_factorysetting3_register_t;

typedef struct {
    int16_t x_axis; /*!< 12-bit resolution x-axis read out */
    int16_t y_axis; /*!< 12-bit resolution y-axis read out */
    int16_t z_axis; /*!< 12-bit resolution z-axis read out */
    int16_t temperature; /*!< 12-bit resolution temperature read out */
    bool temperature_enabled;
} tlv493d_raw_data_t;

typedef struct {
    float x_axis;       /*!< x-axis magnetic in mT (+/-130 mT) */
    float y_axis;       /*!< y-axis magnetic in mT (+/-130 mT) */
    float z_axis;       /*!< z-axis magnetic in mT (+/-130 mT) */
    float temperature;  /*!< temperature in degrees celsius */
    bool temperature_enabled;
} tlv493d_data_t;

typedef struct tlv493d_s {
    // I2C master handle via port with configuration
    i2c_master_dev_handle_t i2c_dev;
    // I2C master configuration
    i2c_device_config_t dev_cfg;
    // I2C master handle via port
    i2c_master_bus_handle_t bus_handle;
    tlv493d_data_t data;
    tlv493d_factorysetting1_register_t factset1;
    tlv493d_factorysetting2_register_t factset2;
    tlv493d_factorysetting3_register_t factset3;
    bool parity_test_enabled;           /*!< tlv493d parity test enabled when true */
    bool temperature_disabled;          /*!< tlv493d temperature sensor disabled when true */
    tlv_493d_power_modes_t power_mode;  /*!< tlv493d power mode */
    bool irq_pin_enabled;               /*!< tlv493d interrupt pin enabled when true */
    uint8_t device_id;
} tlv493d_t;

esp_err_t tlv493_read_data(tlv493d_t *sensor);

esp_err_t tlv493_init(tlv493d_t **sensor, i2c_master_bus_handle_t bus_handle);

#ifdef __cplusplus
};
#endif
