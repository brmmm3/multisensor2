#include "main.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // Config command
    struct arg_int *value;
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
        } else if (strcmp(cmd, "write") == 0 || strcmp(cmd, "save") == 0) {
            config_write();
        } else if (strcmp(cmd, "default") == 0) {
            create_default_config_sd();
        } else if (strcmp(cmd, "show") == 0) {
            config_show();
        } else if (strcmp(cmd, "lock") == 0) {
            ui_config_lock(true);
        } else if (strcmp(cmd, "unlock") == 0) {
            ui_config_lock(true);
        } else if (strcmp(cmd, "tab") == 0) {
            if (config_cmd_args.value->count == 1) {
                uint8_t tab = config_cmd_args.value->ival[0];
                if (tab < 6) {
                    ui_set_current_tab(tab);
                }
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

void register_config_cmd()
{
    config_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    config_cmd_args.value = arg_int0("v", NULL, "<0-1>", "Value");
    config_cmd_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "config",
        .help = "Config. Command: show, read, write|save, default, tab",
        .hint = NULL,
        .func = &process_config_cmd,
        .argtable = &config_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
