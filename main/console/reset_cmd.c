#include "gps.h"
#include "main.h"
#include "ui/include/ui_config.h"

#include "include/reset_cmd.h"

static const char *TAG = "CMD";

struct {
    struct arg_int *gps;    // 0=soft reset, 1=full reset
    struct arg_end *end;
} reset_cmd_args;


int process_reset_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&reset_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, reset_cmd_args.end, argv[0]);
        return 1;
    }
    if (reset_cmd_args.gps->count == 1) {
        uint8_t value = reset_cmd_args.gps->ival[0];
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
    return 0;
}

void register_reset_cmd()
{
    reset_cmd_args.gps = arg_int0("g", NULL, "<0-2>", "GPS: 0=soft, 1=partial, 2=full");
    reset_cmd_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "reset",
        .help = "Reset component",
        .hint = NULL,
        .func = &process_reset_cmd,
        .argtable = &reset_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
