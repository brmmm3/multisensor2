/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "config.h"

#include "wifi/include/wifi.h"
#include "include/wifi_cmd.h"
#include "ui/include/ui_config.h"

const static char *TAG = "WiFi";

typedef struct {
    struct arg_str *cmd;
    struct arg_str *ssid;
    struct arg_str *password;
    struct arg_int *mode;          // Powersave mode.
    struct arg_int *index;         // Index for array of known networks. Will be saved in config.
    struct arg_int *auto_connect;  // Connect to this network automatically.
    struct arg_end *end;
} sta_connect_args_t;
static sta_connect_args_t wifi_cmd_args;


static esp_err_t cmd_do_wifi_connect(const char *ssid, const char *password)
{
    esp_err_t err;

    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WIFI_MODE_STA. err(%d)=%s", err, strerror(err));
        return err;
    }
    err = ui_wifi_set_pwr_mode(true);
    ESP_LOGI(TAG, "WIFI_CONNECT_START, ret: 0x%x", err);
    return err;
}

static void cmd_do_wifi_show()
{
    for (uint8_t i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "%d: <%s>", i, config_nvs->wifi.ssid[i]);
    }
    ESP_LOGI(TAG, "Auto connect: %d", config->auto_connect);
}

int process_wifi_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&wifi_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, wifi_cmd_args.end, argv[0]);
        return 1;
    }
    if (wifi_cmd_args.cmd->count == 1) {
        const char *cmd = wifi_cmd_args.cmd->sval[0];
        if (strcmp(cmd, "show") == 0) {
            cmd_do_wifi_show();
        } else if (strcmp(cmd, "init") == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(ui_wifi_set_pwr_mode(true));
        } else if (strcmp(cmd, "down") == 0) {
            config->auto_connect = 4;
            ESP_ERROR_CHECK_WITHOUT_ABORT(ui_wifi_set_pwr_mode(false));
        } else if (strcmp(cmd, "scan") == 0) {
            wifi_scan();
        } else if (strcmp(cmd, "pwr") == 0) {
            if (wifi_cmd_args.mode->count < 1) {
                ESP_LOGE(TAG, "mode missing");
                return 1;
            }
            int mode = wifi_cmd_args.mode->ival[0];
            esp_err_t err = wifi_set_pwr_mode(mode);  // Or WIFI_PS_MAX_MODEM / WIFI_PS_NONE
            ESP_LOGI(TAG, "WiFi PowerSave mode=%d err=%d", mode, err);
        } else if (strcmp(cmd, "auto") == 0) {
            if (wifi_cmd_args.auto_connect->count < 1) {
                ESP_LOGE(TAG, "auto index missing");
                return 1;
            }
            uint8_t index = (uint8_t)wifi_cmd_args.auto_connect->ival[0];
            config->auto_connect = index;
            ESP_ERROR_CHECK_WITHOUT_ABORT(ui_wifi_set_pwr_mode(true));
        } else if (strcmp(cmd, "connect") == 0) {
            if (wifi_cmd_args.ssid->count < 1) {
                ESP_LOGE(TAG, "SSID missing");
                return 1;
            }
            if (wifi_cmd_args.password->count < 1) {
                ESP_LOGE(TAG, "Password missing");
                return 1;
            }
            const char *ssid = wifi_cmd_args.ssid->sval[0];
            const char *password = wifi_cmd_args.password->sval[0];
            if (wifi_cmd_args.index->count == 1) {
                uint8_t index = (uint8_t)wifi_cmd_args.index->ival[0];
                ESP_LOGI(TAG, "Save to index=%d", index);
                if (index < 4) {
                    strcpy(config_nvs->wifi.ssid[index], ssid);
                    strcpy(config_nvs->wifi.password[index], password);
                }
            }
            if (wifi_cmd_args.auto_connect->count == 1) {
                uint8_t index = (uint8_t)wifi_cmd_args.auto_connect->ival[0];
                ESP_LOGI(TAG, "Auto connect index=%d", index);
                config->auto_connect = index;
            }
            cmd_do_wifi_connect(ssid, password);
        } else {
            ESP_LOGE(TAG, "no valid arguments");
            return 1;
        }
    } else {
        ESP_LOGE(TAG, "no valid arguments");
        return 1;
    }
    return 0;
}

void register_wifi_cmd(void)
{
    wifi_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    wifi_cmd_args.mode = arg_int0("m", NULL, "<mode>", "powersave mode: 0|1|2");
    wifi_cmd_args.auto_connect = arg_int0("a", NULL, "<0-3>", "Auto connect");
    wifi_cmd_args.ssid = arg_str0("s", NULL, "<ssid>", "SSID of AP");
    wifi_cmd_args.password = arg_str0("p", NULL, "<pass>", "password of AP");
    wifi_cmd_args.index = arg_int0("i", NULL, "<0-3>", "Known Networks list index");
    wifi_cmd_args.end = arg_end(6);

    const esp_console_cmd_t cmd = {
        .command = "wifi",
        .help = "WiFi. Command: show, init, down, scan, pwr -m mode, auto -i index, connect ssid pwd [-i index] [-a index]",
        .hint = NULL,
        .func = &process_wifi_cmd,
        .argtable = &wifi_cmd_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
