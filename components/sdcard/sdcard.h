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
#include "esp_err.h"

#define MOUNT_POINT "/sdcard"

int sd_card_init(uint8_t cs_pin, uint8_t sclk_pin, uint8_t mosi_pin, uint8_t miso_pin);

esp_err_t write_text_file(const char *path, char *data);

esp_err_t read_text_file(const char *path, char *buf, uint16_t size);

esp_err_t write_bin_file(const char *path, void *data, uint16_t size);

esp_err_t read_bin_file(const char *path, void *buf, uint16_t size);

#ifdef __cplusplus
};
#endif
