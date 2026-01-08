#include "main.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // Config command
    struct arg_end *end;
} config_cmd_args;


int process_config_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&config_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, config_cmd_args.end, argv[0]);
        return 1;
    }
    if (config_cmd_args.cmd->count == 1) {
        const char *cmd = config_cmd_args.cmd->sval[0];
        if (strcmp(cmd, "read") == 0) {
            config_read();
        } else if (strcmp(cmd, "write") == 0) {
            config_write();
        } else if (strcmp(cmd, "default") == 0) {
            create_default_config_sd();
        } else if (strcmp(cmd, "show") == 0) {
            ESP_LOGI(TAG, "SD-Card:");
            ESP_LOGI(TAG, "cfg_version=%d", config->cfg_version);
            ESP_LOGI(TAG, "auto_connect=%d", config->auto_connect);
            ESP_LOGI(TAG, "auto_record=%d", config->auto_record);
            ESP_LOGI(TAG, "lcd_pwr=%d", config->lcd_pwr);
            ESP_LOGI(TAG, "gps_pwr=%d", config->gps_pwr);
            ESP_LOGI(TAG, "scd4x_pwr=%d", config->scd4x_pwr);
            ESP_LOGI(TAG, "wifi_pwr=%d", config->wifi_pwr);
            ESP_LOGI(TAG, "mode_pwr=%d", config->mode_pwr);
            ESP_LOGI(TAG, "mqtt_broker=%s", config->mqtt_broker);
            ESP_LOGI(TAG, "mqtt_auto_connect=%d", config->mqtt_auto_connect);
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

void register_config_cmd()
{
    config_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    config_cmd_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "config",
        .help = "Config. Command: show, read, write, default",
        .hint = NULL,
        .func = &process_config_cmd,
        .argtable = &config_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
