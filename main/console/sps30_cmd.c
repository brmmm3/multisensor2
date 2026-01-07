#include "main.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // SCD4x command
    struct arg_end *end;
} sps30_cmd_args;


int process_sps30_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&sps30_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, sps30_cmd_args.end, argv[0]);
        return 1;
    }
    if (sps30_cmd_args.cmd->count == 1) {
        const char *cmd = sps30_cmd_args.cmd->sval[0];
        if (strcmp(cmd, "st") == 0) {
            // Get sensor info and status
            sps30_dump_info(sps30);
            sps30_dump_values(sps30, true);
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

void register_sps30_cmd()
{
    sps30_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    sps30_cmd_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "sps",
        .help = "SPS30. Command: st",
        .hint = NULL,
        .func = &process_sps30_cmd,
        .argtable = &sps30_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
