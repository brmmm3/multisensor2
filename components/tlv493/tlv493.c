#include <stdint.h>
#include <string.h>
#include <math.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "tlv493.h"

static const char *TAG = "TLV493";

#define TLV493_LO_I2C_ADDR  0x1F    // TLV493 I2C address
#define TLV493_HI_I2C_ADDR  0x5E    // TLV493 I2C address

#define TLV493D_REG_BX_MSB_R          UINT8_C(0x00)
#define TLV493D_REG_BY_MSB_R          UINT8_C(0x01)
#define TLV493D_REG_BZ_MSB_R          UINT8_C(0x02)
#define TLV493D_REG_TEMP_MSB_R        UINT8_C(0x03)
#define TLV493D_REG_BX_BY_LSB_R       UINT8_C(0x04)
#define TLV493D_REG_BZ_LSB_R          UINT8_C(0x05)
#define TLV493D_REG_TEMP_LSB_R        UINT8_C(0x06)
#define TLV493D_REG_FACTSET1_R        UINT8_C(0x07)  // factory setting for write register 0x01 (bits: 4-3 device spefific)
#define TLV493D_REG_FACTSET2_R        UINT8_C(0x08)  // factory setting for write register 0x02
#define TLV493D_REG_FACTSET3_R        UINT8_C(0x09)  // factory setting for write register 0x03 (bits: 4-0 device specific)
#define TLV493D_REG_RESERVED1_W       UINT8_C(0x00)
#define TLV493D_REG_MOD1_W            UINT8_C(0x01)  // set bits 4-3 (device spefific) from read register 0x07
#define TLV493D_REG_RESERVED2_W       UINT8_C(0x02)  // set bits from read register 0x08
#define TLV493D_REG_MOD2_W            UINT8_C(0x03)  // set bits 4-0 (device spefific) from read register 0x09

#define TLV493D_DATA_POLL_TIMEOUT_MS  UINT16_C(1000)
#define TLV493D_DATA_READY_DELAY_MS   UINT16_C(2)
#define TLV493D_POWERUP_DELAY_MS      UINT16_C(120)
#define TLV493D_RESET_DELAY_MS        UINT16_C(25)
#define TLV493D_SETUP_DELAY_MS        UINT16_C(15)
#define TLV493D_APPSTART_DELAY_MS     UINT16_C(25)    /*!< delay after initialization before application start-up */
#define TLV493D_CMD_DELAY_MS          UINT16_C(5)     /*!< delay before attempting I2C transactions after a command is issued */
#define TLV493D_TX_RX_DELAY_MS        UINT16_C(10)    /*!< delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */


void tlv493_close(tlv493d_t *sensor);


float tlv493_atan2_remapped(float x, float y)
{
    if ((x == 0.0) && (y == 0.0)) return 0.0;
    else if ((x > 0.0) && (y == 0.0)) return 0.0;
    else if ((x > 0.0) && (y > 0.0)) return atan(y / x) * 57.29;
    else if ((x == 0.0) && (y > 0.0)) return 90.0;
    else if ((x < 0.0) && (y > 0.0)) return (M_PI + atan(y / x)) * 57.29;
    else if ((x < 0.0) && (y == 0.0)) return 180.0;
    else if ((x < 0.0) && (y < 0.0)) return (M_PI + atan(y / x)) * 57.29;
    else if ((x == 0.0) && (y < 0.0)) return 270.0;
    else if ((x > 0.0) && (y < 0.0)) return (2.0 * M_PI + atan(y / x)) * 57.29;
    return 0.0;
}

esp_err_t tlv493_read(tlv493d_t *sensor, uint8_t addr, uint8_t *dout, size_t size)
{
    esp_err_t err;

    err = i2c_master_transmit(sensor->i2c_dev, &addr, 1, CONFIG_TLV493_TIMEOUT);
    if (err != ESP_OK) return err;
    return i2c_master_receive(sensor->i2c_dev, dout, size, CONFIG_TLV493_TIMEOUT);
    //return i2c_master_transmit_receive(sensor->i2c_dev, &addr, 1, dout, size, CONFIG_TLV493_TIMEOUT);
}

esp_err_t tlv493_write(tlv493d_t *sensor, uint8_t addr, uint8_t *din, size_t size)
{
    if (din == NULL || size == 0) {
        return i2c_master_transmit(sensor->i2c_dev, &addr, 1, CONFIG_TLV493_TIMEOUT);
    }

    i2c_master_transmit_multi_buffer_info_t buffer[2] = {
        {.write_buffer = &addr, .buffer_size = 1},
        {.write_buffer = din, .buffer_size = size},
    };

    return i2c_master_multi_buffer_transmit(sensor->i2c_dev, buffer, 2, CONFIG_TLV493_TIMEOUT);
}

esp_err_t tlv493_write_byte(tlv493d_t *sensor, uint8_t addr, uint8_t data)
{
    return tlv493_write(sensor, addr, &data, 1);
}

static inline esp_err_t tlv493_get_bz_lsb_register(tlv493d_t *sensor, tlv493d_bz_lsb_register_t *bz_lsb_register)
{
    esp_err_t err;

    if ((err = tlv493_read(sensor, TLV493D_REG_BZ_LSB_R, &bz_lsb_register->reg, sizeof(bz_lsb_register->reg)))) return err;
    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));
    return ESP_OK;
}

esp_err_t tlv493_reset(tlv493d_t *sensor)
{
    esp_err_t err;

    /* attempt i2c recovery transaction */
    //if ((err = tlv493_write(sensor, 0xFF, NULL, 0)) != ESP_OK) return err;
    /* delay before next i2c transaction */
    //vTaskDelay(pdMS_TO_TICKS(25));
    /* attempt i2c recovery transaction */
    if ((err = tlv493_write(sensor, 0x00, NULL, 0)) != ESP_OK) return err;
    //if ((err = tlv493_write(sensor, TLV493D_REG_RESERVED1_W, NULL, 0)) != ESP_OK) return err;
    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(250));
    return ESP_OK;
}

static inline int16_t tlv493d_concat_12bit_data(const uint8_t msb, const uint8_t lsb)
{
    int16_t value = 0x0000;	//16-bit signed integer for 12-bit values of sensor

    value = (msb & 0x0F) << 12;
	value |= lsb <<4 ;
    value >>= 4;  //shift left so that value is a signed 12 bit integer
	return value;
}

esp_err_t tlv493_read_data(tlv493d_t *sensor)
{
    tlv493d_raw_data_t                 out_data;
    tlv493d_temperature_msb_register_t temperature_msb_reg;
    tlv493d_bx_by_lsb_register_t       bx_by_lsb_reg;
    tlv493d_bz_lsb_register_t          bz_lsb_reg;
    uint8_t data[7];
    esp_err_t err;

    if ((err = tlv493_read(sensor, TLV493D_REG_BX_MSB_R, data, sizeof(data))) != ESP_OK) return err;

    temperature_msb_reg.reg = data[3];
    bx_by_lsb_reg.reg = data[4];
    bz_lsb_reg.reg = data[5];
    out_data.x_axis = tlv493d_concat_12bit_data(data[0], bx_by_lsb_reg.bits.bx_lsb);
    out_data.y_axis = tlv493d_concat_12bit_data(data[1], bx_by_lsb_reg.bits.by_lsb);
    out_data.z_axis = tlv493d_concat_12bit_data(data[2], bz_lsb_reg.bits.bz_lsb);
    out_data.temperature = tlv493d_concat_12bit_data(temperature_msb_reg.bits.temperature_msb, data[6]);

    ESP_LOGI(TAG, "x-axis: %d", out_data.x_axis);
    ESP_LOGI(TAG, "y-axis: %d", out_data.y_axis);
    ESP_LOGI(TAG, "z-axis: %d", out_data.z_axis);
    ESP_LOGI(TAG, "temp:   %d", out_data.temperature);

    /*uint16_t bx = (uint16_t)data[0] << 4 | (uint16_t)data[4] >> 4;
    uint16_t by = (uint16_t)data[1] << 4 | (uint16_t)(data[4] & 0x0f);
    uint16_t bz = (uint16_t)data[2] << 4 | (uint16_t)(data[5] & 0x0f);
    uint16_t temp = (uint16_t)data[3] << 4 | (uint16_t)(data[5] & 0xc0) >> 4;
    ESP_LOGI(TAG, "bx=%d by=%d bz=%d temp=%d", bx, by, bz, temp);*/
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, sizeof(data), ESP_LOG_INFO);
    return ESP_OK;
}

esp_err_t tlv493d_get_data_status(tlv493d_t *sensor, bool *ready)
{
    tlv493d_bz_lsb_register_t bz_reg;
    esp_err_t err;

    if ((err = tlv493_get_bz_lsb_register(sensor, &bz_reg)) != ESP_OK) return err;
    *ready = bz_reg.bits.power_down_flag;
    return ESP_OK;
}

/**
 * Create sensor device.
 * @param tlv493 Driver Sturcture.
 * @param dev_addr Chip address.
 */
static esp_err_t tlv493_device_create(tlv493d_t *sensor, const uint16_t dev_addr)
{
    esp_err_t err;

    ESP_LOGI(TAG, "device_create for TLV493 sensors on ADDR %02X", dev_addr);
    sensor->dev_cfg.device_address = dev_addr;
    // Add device to the I2C bus
    if ((err = i2c_master_bus_add_device(sensor->bus_handle, &sensor->dev_cfg, &sensor->i2c_dev)) != ESP_OK) {
        ESP_LOGE(TAG, "device_create error on 0x%02X", dev_addr);
        return err;
    }
    ESP_LOGI(TAG, "device_create success on 0x%02X", dev_addr);
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));
    return ESP_OK;
}

tlv493d_t *tlv493_create_master(i2c_master_bus_handle_t bus_handle)
{
    tlv493d_t *sensor = malloc(sizeof(tlv493d_t));

    if (sensor) {
        memset(sensor, 0, sizeof(tlv493d_t));
        sensor->bus_handle = bus_handle;
        sensor->dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        sensor->dev_cfg.device_address = TLV493_LO_I2C_ADDR;
        sensor->dev_cfg.scl_speed_hz = CONFIG_TLV493_I2C_CLK_SPEED_HZ;
        sensor->i2c_dev = NULL;
        sensor->parity_test_enabled = true;
        sensor->power_mode = TLV493D_LOW_POWER_MODE;
        sensor->irq_pin_enabled = true;
        sensor->device_id = 0x00;
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for TLV493.");
        tlv493_close(sensor);
        return NULL;
    }
    return sensor;
}

void tlv493_close(tlv493d_t *sensor)
{
    if (sensor != NULL && sensor->i2c_dev != NULL) {
        i2c_master_bus_rm_device(sensor->i2c_dev);
    }
    free(sensor);
}

static inline uint8_t tlv493d_calculate_parity(uint8_t data)
{
    uint8_t out = data;

	out ^= out >> 4;
	out ^= out >> 2;
	out ^= out >> 1;
	return out & 1U;
}

static inline esp_err_t tlv493d_configure_power_mode1_register(tlv493d_t *sensor, tlv493d_mode1_register_t *mode1_reg)
{
    switch(sensor->power_mode) {
        case TLV493D_POWER_DOWN_MODE:
            mode1_reg->bits.fast_mode_enabled      = false;
            mode1_reg->bits.low_power_mode_enabled = false;
            break;
        case TLV493D_FAST_MODE:
            mode1_reg->bits.fast_mode_enabled      = true;
            mode1_reg->bits.low_power_mode_enabled = false;
            break;
        case TLV493D_LOW_POWER_MODE:
            mode1_reg->bits.fast_mode_enabled      = false;
            mode1_reg->bits.low_power_mode_enabled = true;
            break;
        case TLV493D_ULTRA_LOW_POWER_MODE:
            mode1_reg->bits.fast_mode_enabled      = false;
            mode1_reg->bits.low_power_mode_enabled = true;
            break;
        case TLV493D_MASTER_CONTROLLED_MODE:
            mode1_reg->bits.fast_mode_enabled      = true;
            mode1_reg->bits.low_power_mode_enabled = true;
            break;
    }
    return ESP_OK;
}

static inline esp_err_t tlv493d_configure_power_mode2_register(tlv493d_t *sensor, tlv493d_mode2_register_t *mode2_reg) {
    switch(sensor->power_mode) {
        case TLV493D_POWER_DOWN_MODE:
            mode2_reg->bits.low_power_period = TLV493D_LOW_POWER_PERIOD_100MS;
            break;
        case TLV493D_FAST_MODE:
            mode2_reg->bits.low_power_period = TLV493D_LOW_POWER_PERIOD_100MS;
            break;
        case TLV493D_LOW_POWER_MODE:
            mode2_reg->bits.low_power_period = TLV493D_LOW_POWER_PERIOD_12MS;
            break;
        case TLV493D_ULTRA_LOW_POWER_MODE:
            mode2_reg->bits.low_power_period = TLV493D_LOW_POWER_PERIOD_100MS;
            break;
        case TLV493D_MASTER_CONTROLLED_MODE:
            mode2_reg->bits.low_power_period = TLV493D_LOW_POWER_PERIOD_12MS;
            break;
    }
    return ESP_OK;
}

static inline esp_err_t tlv493d_set_mode1_register(tlv493d_t *sensor, tlv493d_mode1_register_t mode1_register)
{
    tlv493d_mode1_register_t mode1_reg;
    esp_err_t err;

    mode1_reg.reg = mode1_register.reg;
    /* set factory bits */
    mode1_reg.bits.factory_setting = sensor->factset1.bits.factory_setting;
    ESP_LOGI(TAG, "tlv493d_set_mode1_register %02X", mode1_reg.reg);
    /* attempt i2c write transaction */
    if ((err = tlv493_write_byte(sensor, TLV493D_REG_MOD1_W, mode1_reg.reg))) return err;
    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));
    return ESP_OK;
}

static inline esp_err_t tlv493d_set_mode2_register(tlv493d_t *sensor, const tlv493d_mode2_register_t mode2_register)
{
    tlv493d_mode2_register_t mode2_reg;
    esp_err_t err;

    mode2_reg.reg = mode2_register.reg;
    mode2_reg.bits.factory_setting = sensor->factset3.bits.factory_setting;
    if ((err = tlv493_write_byte(sensor, TLV493D_REG_MOD2_W, mode2_reg.reg))) return err;
    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));
    return ESP_OK;
}

esp_err_t tlv493_device_init(tlv493d_t *sensor)
{
    tlv493d_reserved1_register_t reserved1_reg;
    tlv493d_mode1_register_t mode1_reg;
    tlv493d_reserved2_register_t reserved2_reg;
    tlv493d_mode2_register_t mode2_reg;
    uint8_t result = 0x00;
    esp_err_t err;

    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if ((err = tlv493_reset(sensor)) != ESP_OK) return err;
    // Get factory settings registers
    if ((err = tlv493_read(sensor, TLV493D_REG_FACTSET1_R, &sensor->factset1.reg, 1)) != ESP_OK) return err;
    if ((err = tlv493_read(sensor, TLV493D_REG_FACTSET2_R, &sensor->factset2.reg, 1)) != ESP_OK) return err;
    if ((err = tlv493_read(sensor, TLV493D_REG_FACTSET3_R, &sensor->factset3.reg, 1)) != ESP_OK) return err;
    reserved1_reg.reg = 0;
    mode1_reg.bits.factory_setting = sensor->factset1.bits.factory_setting;
    mode1_reg.bits.irq_pin_enabled = sensor->irq_pin_enabled;
    mode1_reg.bits.i2c_slave_address = TLV493D_I2C_ADDRESS_00;
    if ((err = tlv493d_configure_power_mode1_register(sensor, &mode1_reg)) != ESP_OK) return err;
    reserved2_reg.bits.factory_setting  = sensor->factset2.bits.factory_setting;
    mode2_reg.bits.factory_setting      = sensor->factset3.bits.factory_setting;
    mode2_reg.bits.parity_test_enabled  = sensor->parity_test_enabled;
    mode2_reg.bits.temperature_disabled = sensor->temperature_disabled;
    if ((err = tlv493d_configure_power_mode2_register(sensor, &mode2_reg)) != ESP_OK) return err;
    if ((err = tlv493d_set_mode2_register(sensor, mode2_reg)) != ESP_OK) return err;
    ESP_LOGI(TAG, "factset1=%02X", sensor->factset1.reg);
    ESP_LOGI(TAG, "factset2=%02X", sensor->factset2.reg);
    ESP_LOGI(TAG, "factset3=%02X", sensor->factset3.reg);
    ESP_LOGI(TAG, "mode1=%02X", mode1_reg.reg);
    ESP_LOGI(TAG, "mode2=%02X", mode2_reg.reg);
    /* Calc checksum */
    result ^= reserved1_reg.reg;
    result ^= mode1_reg.reg;
    result ^= reserved2_reg.reg;
    result ^= mode2_reg.reg;
    result = tlv493d_calculate_parity(result);
    ESP_LOGI(TAG, "parity 0x%02x", result);
    mode1_reg.bits.parity = result;
    if ((err = tlv493d_set_mode1_register(sensor, mode1_reg)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_LOGI(TAG, "Device initialized");
    return ESP_OK;
}

esp_err_t tlv493_init(tlv493d_t **sensor, i2c_master_bus_handle_t bus_handle)
{
    uint8_t addr = TLV493_LO_I2C_ADDR;
    esp_err_t err;

    ESP_LOGI(TAG, "Initialize TLV493");
    *sensor = tlv493_create_master(bus_handle);
    if (!*sensor) {
        ESP_LOGE(TAG, "Could not create TLV493 driver.");
        return ESP_FAIL;
    }
    // Probe and create device
    ESP_LOGI(TAG, "Probing for TLV493");
    if ((err = i2c_master_probe(bus_handle, addr, CONFIG_TLV493_PROBE_TIMEOUT)) != ESP_OK) {
        addr = TLV493_HI_I2C_ADDR;
        if ((err = i2c_master_probe(bus_handle, addr, CONFIG_TLV493_PROBE_TIMEOUT)) != ESP_OK) return err;
    }
    ESP_LOGI(TAG, "Found TLV493 on I2C address 0x%02X", addr);
    if ((err = tlv493_device_create(*sensor, addr)) != ESP_OK) return err;
    // Initialize device
    if ((err = tlv493_device_init(*sensor)) != ESP_OK) return err;
    ESP_LOGI(TAG, "TLV493 initialized");
    return ESP_OK;
}
