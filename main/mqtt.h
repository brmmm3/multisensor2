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

extern const char *MQTT_BROKER;

extern bool mqtt_connected;

void mqtt_start();

void mqtt_stop();

void mqtt_publish_values();

#ifdef __cplusplus
};
#endif
