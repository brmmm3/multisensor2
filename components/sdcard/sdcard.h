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
#include <stdbool.h>
#include "esp_err.h"
#include <dirent.h>

#define MOUNT_POINT "/sdcard"

typedef struct sd_fat_info_s {
    uint64_t bytes_total;
    uint64_t  bytes_free;
} sd_fat_info_t;


sd_fat_info_t *sd_get_fat_info();

int sd_card_init(uint8_t cs_pin, uint8_t sclk_pin, uint8_t mosi_pin, uint8_t miso_pin);

esp_err_t sd_card_get_info(char *buf, uint64_t *bytes_total, uint64_t *bytes_free);

esp_err_t sd_card_mount_fs();

int sd_card_mounted(bool check);

int sd_card_get_file_count(const char *path);

esp_err_t ensure_dir(const char *path);

esp_err_t write_text_file(const char *path, char *data);

esp_err_t read_text_file(const char *path, char *buf, uint32_t size);

esp_err_t write_bin_file(const char *path, void *data, uint32_t size);

uint32_t read_bin_file(const char *path, void *buf, uint32_t size);

FILE *open_bin_file(const char *path);

uint32_t read_bin_file_part(FILE *f, void *buf, uint32_t size);

int close_bin_file(FILE *f);

char *get_data_file_path(const char *path);

FILE *open_data_file(const char *path);

uint32_t read_data_file_part(FILE *f, void *buf, uint32_t size);

int close_data_file(FILE *f);

uint32_t read_data_file(const char *path, void *buf, uint32_t size);

esp_err_t remove_data_file(const char *path);

DIR *sd_open_data_dir();

int sd_read_dir(DIR *dir, char *buf, int maxlen);

void sd_closedir(DIR *dir);

#ifdef __cplusplus
};
#endif
