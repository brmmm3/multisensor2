/**
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
#include <stdbool.h>

extern bool tcp_server_running;

extern uint8_t tcp_client_cnt;

esp_err_t tcp_server_start();

esp_err_t tcp_server_stop();

void tcp_server_publish_values();

#ifdef __cplusplus
};
#endif
