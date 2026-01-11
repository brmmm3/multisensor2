#include "main.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // TCP command
    struct arg_int *value;
    struct arg_end *end;
} sd_cmd_args;


int process_sd_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&sd_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, sd_cmd_args.end, argv[0]);
        return 1;
    }
    if (sd_cmd_args.cmd->count == 1) {
        const char *cmd = sd_cmd_args.cmd->sval[0];
        if (strcmp(cmd, "mount") == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(sd_card_mount_fs());
            return 0;
        }
        if (sd_cmd_args.value->count == 0) {
            ESP_LOGE(TAG, "no valid arguments");
            return 1;
        }
        if (strcmp(cmd, "rec") == 0) {
            bool recording = sd_cmd_args.value->ival[0] != 0;
            ui_set_switch_state(ui->sw_record, recording);
            ui_sd_record_set_value(recording);
        } else if (strcmp(cmd, "auto") == 0) {
            config->auto_record = sd_cmd_args.value->ival[0] != 0;
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

void register_sd_cmd()
{
    sd_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    sd_cmd_args.value = arg_int0("v", NULL, "<0-1>", "Value");
    sd_cmd_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "sd",
        .help = "SD-Card. Command: mount, rec, auto",
        .hint = NULL,
        .func = &process_sd_cmd,
        .argtable = &sd_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
