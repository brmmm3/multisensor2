#include "main.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // SCD4x command
    struct arg_int *temperature;  // Configure temperature offset
    struct arg_int *altitude;     // Configure altitude
    struct arg_int *pressure;     // Configure pressure
    struct arg_int *co2;          // Target CO2 concentration for recalibration
    struct arg_end *end;
} scd4x_cmd_args;


int process_scd4x_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&scd4x_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, scd4x_cmd_args.end, argv[0]);
        return 1;
    }
    if (scd4x_cmd_args.cmd->count == 1) {
        const char *cmd = scd4x_cmd_args.cmd->sval[0];
        if (strcmp(cmd, "st") == 0) {
            scd4x_values_t *values = &scd4x->values;

            // Get sensor info and status
            ESP_LOGI(TAG, "SCD4x (serial=0x%012llX ready=%d):", scd4x->serial_number, scd4x_get_data_ready_status(scd4x));
            ESP_LOGI(TAG, "temp_offs=%f °C  altitude=%d m  pressure=%d hPa * co2=%d ppm  temp=%.1f °C  hum=%.1f %%  st=%d",
                    scd4x->temperature_offset, scd4x->altitude, scd4x->pressure,
                    values->co2, values->temperature, values->humidity, scd4x_st_machine_status);
        } else if (strcmp(cmd, "cfg") == 0) {
            if (scd4x_cmd_args.temperature->count == 1) {
                scd4x_set_temperature_offset(scd4x, (float)scd4x_cmd_args.temperature->ival[0]);
            }
            if (scd4x_cmd_args.altitude->count == 1) {
                scd4x_set_sensor_altitude(scd4x, scd4x_cmd_args.altitude->ival[0]);
            }
            if (scd4x_cmd_args.pressure->count == 1) {
                scd4x_set_ambient_pressure(scd4x, scd4x_cmd_args.pressure->ival[0]);
            }
        } else if (strcmp(cmd, "cal") == 0) {
            if (scd4x_cmd_args.co2->count == 1) {
                scd4x_state_machine_cmd(SCD4X_CMD_FRC, scd4x_cmd_args.co2->ival[0]);
            } else {
                scd4x_state_machine_cmd(SCD4X_CMD_FRC, mhz19->values.co2);
            }
        } else if (strcmp(cmd, "reset") == 0) {
            esp_err_t err = scd4x_stop_periodic_measurement(scd4x);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to stop periodic measurement: %d", err);
                return 1;
            }
            err = scd4x_perfom_factory_reset(scd4x);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to perform factory reset: %d", err);
                return 1;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            err = scd4x_start_periodic_measurement(scd4x);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed start periodic measurement: %d", err);
                return 1;
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

void register_scd4x_cmd()
{
    scd4x_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    scd4x_cmd_args.temperature = arg_int0("t", NULL, "<temp>", "Temperature offset");
    scd4x_cmd_args.altitude = arg_int0("a", NULL, "<alt>", "Altitude");
    scd4x_cmd_args.pressure = arg_int0("p", NULL, "<press>", "Pressure");
    scd4x_cmd_args.co2 = arg_int0("c", NULL, "<co2>", "Target CO2 concentration");
    scd4x_cmd_args.end = arg_end(5);

    const esp_console_cmd_t cmd = {
        .command = "scd",
        .help = "SCD4x. Command: st, cfg, cal, reset, save",
        .hint = NULL,
        .func = &process_scd4x_cmd,
        .argtable = &scd4x_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
