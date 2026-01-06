#include "gps.h"
#include "main.h"

#include "include/pwr_cmd.h"

static const char *TAG = "CMD";

struct {
    struct arg_int *lcd;    // Power mode (0=full speed, 1=low power, 2=sleep) LCD
    struct arg_int *gps;    // Power mode (0=full speed, 1=low power, 2=sleep) GPS
    struct arg_int *bme;    // Power mode (0=full speed, 1=low power, 2=sleep) BME280
    struct arg_int *scd;    // Power mode (0=full speed, 1=low power, 2=sleep) SCD41
    struct arg_int *mhz;    // Power mode (0=full speed, 1=low power, 2=sleep) MHZ19
    struct arg_int *yys;    // Power mode (0=full speed, 1=low power, 2=sleep) YYS
    struct arg_int *sps;    // Power mode (0=full speed, 1=low power, 2=sleep) SPS30
    struct arg_int *qmc;    // Power mode (0=full speed, 1=low power, 2=sleep) QMC5883L
    struct arg_int *adx;    // Power mode (0=full speed, 1=low power, 2=sleep) ADXL345
    struct arg_int *mode;   // Power mode (0=full speed, 1=low power, 2=sleep) ADXL345
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
        lcd_set_bg_pwr(pwr_cmd_args.lcd->ival[0]);
    } else if (pwr_cmd_args.gps->count == 1) {
        uint8_t mode = pwr_cmd_args.gps->ival[0];

        if (mode == 0) {
            gps_power_off(gps);
        } else {
            gps_set_power_mode(gps, mode);
        }
    } else if (pwr_cmd_args.scd->count == 1) {
        if (pwr_cmd_args.scd->ival[0] > 0) {
            scd4x_device_init_do(scd4x);
        } else {
            scd4x_power_down(scd4x);
        }
    } else if (pwr_cmd_args.mode->count == 1) {
        uint8_t mode = pwr_cmd_args.mode->ival[0];

        lcd_set_bg_pwr(mode);
        gps_set_power_mode(gps, mode);
        if (mode > 0) {
            scd4x_device_init_do(scd4x);
        } else {
            scd4x_power_down(scd4x);
        }
    } else {
        ESP_LOGE(TAG, "no valid arguments");
        return 1;
    }
    return 0;
}

void register_pwr_cmd()
{
    pwr_cmd_args.lcd = arg_int0("l", "lcd", "<0-2>", "Set power mode (0=full speed, 1=low power, 2=sleep) LCD");
    pwr_cmd_args.gps = arg_int0("g", "gps", "<0-3>", "Set power mode (0=full speed, 1=low power, 2=sleep) GPS");
    pwr_cmd_args.scd = arg_int0("s", "scd", "<0-2>", "Set power mode (0=full speed, 1=low power, 2=sleep) SCD41");
    pwr_cmd_args.mhz = arg_int0("m", "mhz", "<0-2>", "Set power mode (0=full speed, 1=low power, 2=sleep) MHZ19");
    pwr_cmd_args.sps = arg_int0("p", "sps", "<0-2>", "Set power mode (0=full speed, 1=low power, 2=sleep) SPS30");
    pwr_cmd_args.qmc = arg_int0("q", "qmc", "<0-2>", "Set power mode (0=full speed, 1=low power, 2=sleep) QMC5883L");
    pwr_cmd_args.adx = arg_int0("a", "adx", "<0-2>", "Set power mode (0=full speed, 1=low power, 2=sleep) ADXL345");
    pwr_cmd_args.mode = arg_int0("M", "mode", "<0-2>", "Set power mode (0=full speed, 1=low power, 2=sleep) ADXL345");
    pwr_cmd_args.end = arg_end(9);

    const esp_console_cmd_t cmd = {
        .command = "pwr",
        .help = "Power mode",
        .hint = NULL,
        .func = &process_pwr_cmd,
        .argtable = &pwr_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
