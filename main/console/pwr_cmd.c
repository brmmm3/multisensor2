#include "gps.h"
#include "main.h"
#include "ui/include/ui_config.h"

#include "include/pwr_cmd.h"

static const char *TAG = "CMD";

struct {
    struct arg_int *lcd;    // Power mode (0=full speed, 1=low power, 2=sleep) LCD
    struct arg_int *gps;    // Power mode (0=full speed, 1=low power, 2=sleep) GPS
    struct arg_int *scd;    // Power mode (0=full speed, 1=low power, 2=sleep) SCD4x
    struct arg_int *wifi;   // Power mode (0=full speed, 1=low power, 2=sleep, 3=deep sleep) WiFi
    struct arg_int *mode;   // Power mode (0=full speed, 1=low power, 2=sleep) Mode
    struct arg_end *end;
} pwr_cmd_args;


int process_pwr_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&pwr_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, pwr_cmd_args.end, argv[0]);
        return 1;
    }
    if (pwr_cmd_args.lcd->count == 1) {
        ui_lcd_set_pwr_mode(pwr_cmd_args.lcd->ival[0]);
        config->mode_pwr = 4;
    } else if (pwr_cmd_args.gps->count == 1) {
        ui_gps_set_pwr_mode(pwr_cmd_args.gps->ival[0]);
        config->mode_pwr = 4;
    } else if (pwr_cmd_args.scd->count == 1) {
        ui_scd4x_set_pwr_mode(pwr_cmd_args.scd->ival[0]);
        config->mode_pwr = 4;
    } else if (pwr_cmd_args.wifi->count == 1) {
        ui_wifi_set_pwr_mode(pwr_cmd_args.wifi->ival[0]);
        config->mode_pwr = 4;
    } else if (pwr_cmd_args.mode->count == 1) {
        uint8_t mode = pwr_cmd_args.mode->ival[0];

        if (mode < 4) {
            ui_lcd_set_pwr_mode(pwr_cmd_args.lcd->ival[0]);
            ui_gps_set_pwr_mode(pwr_cmd_args.gps->ival[0]);
            ui_scd4x_set_pwr_mode(pwr_cmd_args.scd->ival[0]);
            ui_wifi_set_pwr_mode(pwr_cmd_args.wifi->ival[0]);
        }
        config->mode_pwr = mode;
    } else {
        ESP_LOGE(TAG, "no valid arguments");
        return 1;
    }
    return 0;
}

void register_pwr_cmd()
{
    pwr_cmd_args.lcd = arg_int0("l", NULL, "<0-2>", "Set power mode (0=full speed, 1=low power, 2=sleep) LCD");
    pwr_cmd_args.gps = arg_int0("g", NULL, "<0-3>", "Set power mode (0=full speed, 1=low power, 2=sleep) GPS");
    pwr_cmd_args.scd = arg_int0("s", NULL, "<0-2>", "Set power mode (0=full speed, 1=low power, 2=sleep) SCD4x");
    pwr_cmd_args.wifi = arg_int0("w", NULL, "<0-3>", "Set power mode (0=full speed, 1=low power, 2=sleep) WiFi");
    pwr_cmd_args.mode = arg_int0("m", NULL, "<0-3>", "Set power mode (0=full speed, 1=low power, 2=sleep) Mode");
    pwr_cmd_args.end = arg_end(5);

    const esp_console_cmd_t cmd = {
        .command = "pwr",
        .help = "Power mode",
        .hint = NULL,
        .func = &process_pwr_cmd,
        .argtable = &pwr_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
