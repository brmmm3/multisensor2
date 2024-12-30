
#include "util.h"

uint8_t uint32_data[6];

/*
* The 8-bit CRC checksum transmitted after each data word is generated by a CRC algorithm.
* The CRC covers the contents of the two previously transmitted data bytes.
* To calculate the checksum only these two previously transmitted data bytes are used.
* Note that command words are not followed by CRC.
*/
uint8_t calc_cksum(const uint8_t* data, uint16_t count)
{
    uint16_t current_byte;
    uint8_t crc_bit = 0;
    uint8_t crc = CRC8_INIT;

    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

uint8_t *uint32_to_bytes(const uint32_t value)
{
    uint32_data[0] = (uint8_t)(value >> 24);
    uint32_data[1] = (uint8_t)(value >> 16);
    uint32_data[2] = calc_cksum((uint8_t *)&uint32_data[0], 2);
    uint32_data[3] = (uint8_t)(value >> 8);
    uint32_data[4] = (uint8_t)value;
    uint32_data[5] = calc_cksum((uint8_t *)&uint32_data[3], 2);
    return (uint8_t *)&uint32_data[0];
}

uint32_t bytes_to_uint32(const uint8_t* bytes)
{
    return (uint32_t)bytes[0] << 24
           | (uint32_t)bytes[1] << 16
           | (uint32_t)bytes[3] << 8
           | (uint32_t)bytes[4];
}

float bytes_to_float(const uint8_t* bytes)
{
    union {
        uint32_t u32_value;
        float float32;
    } tmp;
    tmp.u32_value = bytes_to_uint32(bytes);
    return tmp.float32;
}