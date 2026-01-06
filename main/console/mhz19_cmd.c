#include "main.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // SCD4x command
    struct arg_end *end;
} mhz19_cmd_args;


int process_mhz19_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&mhz19_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, mhz19_cmd_args.end, argv[0]);
        return 1;
    }
    if (mhz19_cmd_args.cmd->count == 1) {
        const char *cmd = mhz19_cmd_args.cmd->sval[0];
        if (strcmp(cmd, "st") == 0) {
            mhz19_values_t *values = &mhz19->values;

            // Get sensor info and status
            ESP_LOGI(TAG, "MHZ19 (fw=%s status=%d  err=%d  range=%d):", mhz19->fw_version, values->status, mhz19->error_cnt, mhz19->range);
            ESP_LOGI(TAG, "co2=%d ppm  temp=%d °C", values->co2, values->temp);
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

void register_mhz19_cmd()
{
    mhz19_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    mhz19_cmd_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "mhz",
        .help = "MHZ19. Command: st",
        .hint = NULL,
        .func = &process_mhz19_cmd,
        .argtable = &mhz19_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
