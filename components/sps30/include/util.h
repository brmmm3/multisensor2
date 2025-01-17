#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CRC8_POLYNOMIAL             (0x31)
#define CRC8_INIT                   (0xFF)

uint8_t sps30_calc_cksum(const uint8_t* data, uint16_t count);

uint8_t sps30_is_data_valid(const uint8_t *data, uint8_t size);

uint8_t sps30_bytes_to_data(const uint8_t *bytes, uint8_t size, uint8_t *data);

uint8_t *sps30_uint32_to_bytes(const uint32_t value);

uint32_t sps30_bytes_to_uint32(const uint8_t* bytes);

/*
 * Convert an array of bytes in big-endian format to a float IEEE754.
 */
float sps30_bytes_to_float(const uint8_t* bytes);

#ifdef __cplusplus
}
#endif
