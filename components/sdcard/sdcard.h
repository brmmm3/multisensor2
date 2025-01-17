/**
 * SD-Card
 *
 * MIT License
 *
 * Copyright (C) 2024 Martin Bammer
 * Please contact at <mrbm74@gmail.com>
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int sd_card_init(uint8_t cs_pin, uint8_t sclk_pin, uint8_t mosi_pin, uint8_t miso_pin);

#ifdef __cplusplus
};
#endif
