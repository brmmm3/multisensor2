/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "config.h"

#include "include/wifi.h"
#include "include/wifi_cmd.h"

const static char *TAG = "WIFI";


static int initialize_wifi(int argc, char **argv)
{
    wifi_init(true);
    return 0;
}

static void register_wifi_init(void)
{
    const esp_console_cmd_t cmd = {
        .command = "wifi_init",
        .help = "Initialize WiFi",
        .hint = NULL,
        .func = &initialize_wifi,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}


static int uninitialize_wifi(int argc, char **argv)
{
    wifi_uninit();
    return 0;
}

static void register_wifi_uninit(void)
{
    const esp_console_cmd_t cmd = {
        .command = "wifi_uninit",
        .help = "Uninitialize WiFi",
        .hint = NULL,
        .func = &uninitialize_wifi,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

typedef struct {
    struct arg_str *ssid;
    struct arg_str *password;
    struct arg_int *authmode;
    struct arg_int *channel;
    struct arg_int *max_conn;
    struct arg_end *end;
} ap_set_args_t;
static ap_set_args_t ap_set_args;

static int cmd_do_ap_set(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &ap_set_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, ap_set_args.end, argv[0]);
        return 1;
    }

    wifi_config_t wifi_config = {};

    const char *ssid = ap_set_args.ssid->sval[0];
    strncpy((char *) wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    const char *pass = ap_set_args.password->sval[0];
    if (ap_set_args.password->count > 0) {
        strncpy((char *) wifi_config.ap.password, pass, sizeof(wifi_config.ap.password));
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK; // set default auth mode
    }
    if (ap_set_args.channel->count > 0) {
        wifi_config.sta.channel = (uint8_t)(ap_set_args.channel->ival[0]);
    }
    if (ap_set_args.authmode->count > 0) {
        wifi_config.ap.authmode = ap_set_args.authmode->ival[0];
    }
    if (ap_set_args.max_conn->count > 0) {
        wifi_config.ap.max_connection = ap_set_args.max_conn->ival[0];
    } else {
        wifi_config.ap.max_connection = 2;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP) );
    esp_err_t err = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "set ap config OK.");
    } else {
        ESP_LOGE(TAG, "set ap config Failed! (%s)", esp_err_to_name(err));
    }
    return 0;
}


static void register_wifi_set_ap(void)
{
    ap_set_args.ssid = arg_str1(NULL, NULL, "<ssid>", "SSID of AP");
    ap_set_args.password = arg_str0(NULL, NULL, "<pass>", "password of AP");
    ap_set_args.authmode = arg_int0("a", "authmode", "<authmode>", "wifi auth type (ie. open | wep| wpa2 | wpa2_enterprise)");
    ap_set_args.channel = arg_int0("n", "channel", "<channel>", "channel of AP");
    ap_set_args.max_conn = arg_int0("m", "max_conn", "<max_conn>", "Max station number, default: 2");
    ap_set_args.end = arg_end(5);
    const esp_console_cmd_t cmd = {
        .command = "wifi_set_ap",
        .help = "WiFi is ap mode, set ap config.",
        .hint = NULL,
        .func = &cmd_do_ap_set,
        .argtable = &ap_set_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));

}

typedef struct {
    struct arg_str *ssid;
    struct arg_str *password;
    struct arg_int *index;         // Index for array of known networks. Will be saved in config.
    struct arg_int *auto_connect;  // Connect to this network automatically.
    struct arg_end *end;
} sta_connect_args_t;
static sta_connect_args_t connect_args;


static int cmd_do_sta_connect(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &connect_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, connect_args.end, argv[0]);
        return 1;
    }

    wifi_config_t wifi_config = {
        .sta = {
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
        },
    };

    const char *ssid = connect_args.ssid->sval[0];
    memcpy((char *) wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    const char *pass = connect_args.password->sval[0];
    if (connect_args.password->count > 0) {
        memcpy((char *) wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WEP;
    }
    if (connect_args.index->count > 0) {
        uint8_t index = (uint8_t)connect_args.index->ival[0];
        ESP_LOGI(TAG, "index=%d", index);
        if (index < 4) {
            strcpy(config->nvs.wifi.ssid[index], ssid);
            strcpy(config->nvs.wifi.password[index], pass);
        }
        if (connect_args.auto_connect->count > 0) {
            uint8_t auto_connect = (uint8_t)connect_args.auto_connect->ival[0];
            ESP_LOGI(TAG, "auto_connect=%d", auto_connect);
            if (auto_connect) {
                config->auto_connect = index;
                wifi_init(false);
                return 0;
            }
        }
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_LOGI(TAG, "Connecting to %s...", ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_err_t err = esp_wifi_connect();
    ESP_LOGI(TAG, "WIFI_CONNECT_START, ret: 0x%x", err);
    return 0;
}

static void register_wifi_connect(void)
{
    connect_args.ssid = arg_str1(NULL, NULL, "<ssid>", "SSID of AP");
    connect_args.password = arg_str0(NULL, NULL, "<pass>", "password of AP");
    connect_args.index = arg_int0("i", "index", "<int>", "Array index 0-3");
    connect_args.auto_connect = arg_int0("a", "auto", "<int>", "Autoconnect");
    connect_args.end = arg_end(4);
    const esp_console_cmd_t cmd = {
        .command = "wifi_connect",
        .help = "WiFi is station mode, join specified soft-AP",
        .hint = NULL,
        .func = &cmd_do_sta_connect,
        .argtable = &connect_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}


typedef struct {
    struct arg_int *index;         // Index for array of known networks. Will be saved in config.
    struct arg_end *end;
} sta_auto_connect_args_t;
static sta_auto_connect_args_t auto_connect_args;


static int cmd_do_sta_auto_connect(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &auto_connect_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, auto_connect_args.end, argv[0]);
        return 1;
    }

    if (auto_connect_args.index->count > 0) {
        uint8_t index = (uint8_t)auto_connect_args.index->ival[0];
        config->auto_connect = index;
        wifi_init(false);
    }
    return 0;
}

static void register_wifi_auto_connect(void)
{
    auto_connect_args.index = arg_int0(NULL, NULL, "<int>", "Array index 0-3");
    auto_connect_args.end = arg_end(1);
    const esp_console_cmd_t cmd = {
        .command = "wifi_autoconnect",
        .help = "Set autoconnect index",
        .hint = NULL,
        .func = &cmd_do_sta_auto_connect,
        .argtable = &auto_connect_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

typedef struct {
    struct arg_int *type;
    struct arg_end *end;
} light_sleep_args_t;
static light_sleep_args_t sleep_args;

static int cmd_do_wifi_light_sleep(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &sleep_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, sleep_args.end, argv[0]);
        return 1;
    }
    if (sleep_args.type->count > 0) {
        int enabled = sleep_args.type->ival[0] != 0;
        esp_pm_config_t pm_config = {
            .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
            .min_freq_mhz = esp_clk_xtal_freq() / 1000000,
            .light_sleep_enable = enabled,
        };
        ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
        ESP_LOGI(TAG, "WIFI_LIGHT_SLEEP %d", enabled);
    }
    return 0;
}


static void register_wifi_light_sleep(void)
{
    sleep_args.type = arg_int1(NULL, NULL, "<type>", "light sleep mode: 0|1");
    sleep_args.end = arg_end(1);
    const esp_console_cmd_t cmd = {
        .command = "wifi_light_sleep",
        .help = "Config light sleep mode",
        .hint = NULL,
        .func = &cmd_do_wifi_light_sleep,
        .argtable = &sleep_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}


typedef struct {
    struct arg_int *type;
    struct arg_end *end;
} powersave_args_t;
static powersave_args_t powersave_args;

static int cmd_do_wifi_powersave(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &sleep_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, sleep_args.end, argv[0]);
        return 1;
    }
    if (powersave_args.type->count > 0) {
        int mode = powersave_args.type->ival[0];
        esp_err_t err = esp_wifi_set_ps(mode);  // Or WIFI_PS_MAX_MODEM / WIFI_PS_NONE
        ESP_LOGI(TAG, "WIFI_POWERSAVE mode=%d err=%d", mode, err);
    }
    return 0;
}


static void register_wifi_powersave(void)
{
    powersave_args.type = arg_int1(NULL, NULL, "<type>", "powersave mode: 0|1|2");
    powersave_args.end = arg_end(1);
    const esp_console_cmd_t cmd = {
        .command = "wifi_powersave",
        .help = "Config powersave mode",
        .hint = NULL,
        .func = &cmd_do_wifi_powersave,
        .argtable = &powersave_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}


static int cmd_do_wifi_scan(int argc, char **argv)
{
    wifi_scan();
    return 0;
}


static void register_wifi_scan(void)
{
    const esp_console_cmd_t cmd = {
        .command = "wifi_scan",
        .help = "Scan WiFi",
        .hint = NULL,
        .func = &cmd_do_wifi_scan,
        .argtable = NULL,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static int cmd_do_wifi_show(int argc, char **argv)
{
    for (uint8_t i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "%d: <%s>", i, config->nvs.wifi.ssid[i]);
    }
    ESP_LOGI(TAG, "Auto connect: %d", config->auto_connect);
    return 0;
}


static void register_wifi_show(void)
{
    const esp_console_cmd_t cmd = {
        .command = "wifi_show",
        .help = "Show configures WiFi networks",
        .hint = NULL,
        .func = &cmd_do_wifi_show,
        .argtable = NULL,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void register_wifi_cmd(void)
{
    register_wifi_init();
    register_wifi_uninit();
    register_wifi_set_ap();
    register_wifi_connect();
    register_wifi_auto_connect();
    register_wifi_light_sleep();
    register_wifi_powersave();
    register_wifi_scan();
    register_wifi_show();
}
