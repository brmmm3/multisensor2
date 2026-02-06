#include "argtable3/argtable3.h"
#include "main.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // BMX280 command
    struct arg_int *value;
    struct arg_end *end;
} gps_cmd_args;

static int process_gps_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&gps_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, gps_cmd_args.end, argv[0]);
        return 1;
    }
    if (gps_cmd_args.cmd->count == 1) {
        const char *cmd = gps_cmd_args.cmd->sval[0];
        if (strcmp(cmd, "off") == 0) {
            gps_power_off(gps);
        } else if (strcmp(cmd, "reset") == 0) {
            if (gps_cmd_args.value->count == 1) {
                uint8_t value = gps_cmd_args.value->ival[0];
                if (value == 0) {
                    if (gps_soft_reset(gps) == -1) {
                        ESP_LOGE(TAG, "GPS soft reset failed");
                        return 1;
                    }
                } else if (value == 1) {
                    if (gps_partial_reset(gps) == -1) {
                        ESP_LOGE(TAG, "GPS partial reset failed");
                        return 1;
                    }
                } else if (value == 2) {
                    if (gps_full_reset(gps) == -1) {
                        ESP_LOGE(TAG, "GPS full reset failed");
                        return 1;
                    }
                }
            } else {
                ESP_LOGE(TAG, "no valid arguments");
                return 1;
            }
        } else if (strcmp(cmd, "pwr") == 0) {
            if (gps_cmd_args.value->count == 1) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(ui_gps_set_pwr_mode(gps_cmd_args.value->ival[0]));
            } else {
                ESP_LOGE(TAG, "no valid arguments");
                return 1;
            }
        } else if (strcmp(cmd, "debug") == 0) {
            if (gps_cmd_args.value->count == 1) {
                gps->debug = gps_cmd_args.value->ival[0];
            } else {
                ESP_LOGE(TAG, "no valid arguments");
                return 1;
            }
        }
    } else {
        ESP_LOGE(TAG, "no valid arguments");
        return 1;
    }
    return 0;
}

void register_gps_cmd()
{
    gps_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    gps_cmd_args.value = arg_int0("v", NULL, "<int>", "Value");
    gps_cmd_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "gps",
        .help = "GPS. Command: off, reset, pwr, debug",
        .hint = NULL,
        .func = &process_gps_cmd,
        .argtable = &gps_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
