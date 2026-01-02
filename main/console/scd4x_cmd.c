#include "main.h"

#include "include/scd4x_cmd.h"

static const char *TAG = "CMD";

struct {
    struct arg_int *temp;   // Configure Temperature offset
    struct arg_int *alt;    // Configure altitude
    struct arg_int *press;  // Configure pressure
    struct arg_end *end;
} scd4x_cmd_args;


int process_scd4x_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&scd4x_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, scd4x_cmd_args.end, argv[0]);
        return 1;
    }
    if (scd4x_cmd_args.temp->count == 1) {
        scd4x_set_temperature_offset(scd4x, (float)scd4x_cmd_args.temp->ival[0]);
    } else if (scd4x_cmd_args.alt->count == 1) {
        scd4x_set_sensor_altitude(scd4x, scd4x_cmd_args.alt->ival[0]);
    } else if (scd4x_cmd_args.press->count == 1) {
        scd4x_set_ambient_pressure(scd4x, scd4x_cmd_args.press->ival[0]);
    } else {
        ESP_LOGE(TAG, "no valid arguments");
        return 1;
    }
    return 0;
}

void register_scd4x_cmd()
{
    scd4x_cmd_args.temp = arg_int0("t", "temp", "<temp>", "Temperature offset");
    scd4x_cmd_args.alt = arg_int0("a", "alt", "<alt>", "Altitude");
    scd4x_cmd_args.press = arg_int0("p", "press", "<press>", "Pressure");
    scd4x_cmd_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "scd",
        .help = "SCD4x",
        .hint = NULL,
        .func = &process_scd4x_cmd,
        .argtable = &scd4x_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
