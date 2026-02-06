#include "argtable3/argtable3.h"
#include "main.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // BMX280 command
    struct arg_int *temperature;  // Temperature oversampling
    struct arg_int *pressure;     // Pressure oversampling
    struct arg_int *humidity;     // Humidity oversampling
    struct arg_int *filter;       // Filter coefficient
    struct arg_int *stdby;        // Standby time
    struct arg_int *value;
    struct arg_end *end;
} bme_cmd_args;


static void show_bmx280_status(int num, bmx280_t *sensor)
{
    if (sensor == NULL) return;
    bmx280_getMode(sensor);
    ESP_LOGI(TAG, "BME280 %u:", num);
    bmx280_dump_info(sensor);
    bmx280_dump_values(sensor, true);
}

static void cmd_bmx280_configure(bmx280_t *sensor)
{
    if (bme_cmd_args.temperature->count == 1) {
        sensor->config.t_sampling = bme_cmd_args.temperature->ival[0];
    }
    if (bme_cmd_args.pressure->count == 1) {
        sensor->config.p_sampling = bme_cmd_args.pressure->ival[0];
    }
    if (bme_cmd_args.humidity->count == 1) {
        sensor->config.h_sampling = bme_cmd_args.humidity->ival[0];
    }
    if (bme_cmd_args.filter->count == 1) {
        sensor->config.iir_filter = bme_cmd_args.filter->ival[0];
    }
    if (bme_cmd_args.stdby->count == 1) {
        sensor->config.t_standby = bme_cmd_args.stdby->ival[0];
    }
    ESP_ERROR_CHECK(bmx280_setMode(sensor, BMX280_MODE_SLEEP));
    bmx280_configure(sensor, &sensor->config);
    ESP_ERROR_CHECK(bmx280_setMode(sensor, BMX280_MODE_CYCLE));
}

static int process_bme_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&bme_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, bme_cmd_args.end, argv[0]);
        return 1;
    }
    if (bme_cmd_args.cmd->count == 1) {
        const char *cmd = bme_cmd_args.cmd->sval[0];
        if (strcmp(cmd, "st") == 0 || strcmp(cmd, "status") == 0) {
            // Get sensor info and status
            show_bmx280_status(1, bmx280lo);
            show_bmx280_status(2, bmx280hi);
        } else if (strcmp(cmd, "cfg") == 0) {
            cmd_bmx280_configure(bmx280lo);
            cmd_bmx280_configure(bmx280hi);
        } else if (strcmp(cmd, "debug") == 0) {
            if (bme_cmd_args.value->count == 1) {
                bmx280lo->debug = bme_cmd_args.value->ival[0];
                bmx280hi->debug = bme_cmd_args.value->ival[0];
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

void register_bme_cmd()
{
    bme_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    bme_cmd_args.temperature = arg_int0("t", NULL, "<0-5>", "Temperature oversampling");
    bme_cmd_args.pressure = arg_int0("p", NULL, "<0-5>", "Pressure oversamling");
    bme_cmd_args.humidity = arg_int0("h", NULL, "<0-5>", "Humidity oversampling");
    bme_cmd_args.filter = arg_int0("f", NULL, "<0-5>", "IIR filter");
    bme_cmd_args.stdby = arg_int0("s", NULL, "<0-9>", "Standby time");
    bme_cmd_args.value = arg_int0("v", NULL, "<int>", "Value");
    bme_cmd_args.end = arg_end(7);

    const esp_console_cmd_t cmd = {
        .command = "bme",
        .help = "BMX280. Command: st[atus], cfg, debug",
        .hint = NULL,
        .func = &process_bme_cmd,
        .argtable = &bme_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
