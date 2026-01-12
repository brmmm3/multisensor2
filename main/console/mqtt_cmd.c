#include "main.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // MQTT command
    struct arg_int *auto_connect;
    struct arg_end *end;
} mqtt_cmd_args;


int process_mqtt_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&mqtt_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, mqtt_cmd_args.end, argv[0]);
        return 1;
    }
    if (mqtt_cmd_args.cmd->count == 1) {
        const char *cmd = mqtt_cmd_args.cmd->sval[0];
        if (strcmp(cmd, "start") == 0) {
            mqtt_start();
        } else if (strcmp(cmd, "stop") == 0) {
            mqtt_stop();
        } else if (strcmp(cmd, "auto") == 0) {
            if (mqtt_cmd_args.auto_connect->count == 1) {
                config->mqtt_auto_connect = mqtt_cmd_args.auto_connect->ival[0] != 0;
            }
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

void register_mqtt_cmd()
{
    mqtt_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    mqtt_cmd_args.auto_connect = arg_int0("a", NULL, "<0-1>", "Auto connect");
    mqtt_cmd_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "mqtt",
        .help = "MQTT. Command: start, stop, auto",
        .hint = NULL,
        .func = &process_mqtt_cmd,
        .argtable = &mqtt_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
