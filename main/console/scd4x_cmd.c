#include "main.h"
#include "scd4x.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // SCD4x command
    struct arg_int *temperature;  // Configure temperature offset
    struct arg_int *altitude;     // Configure altitude
    struct arg_int *pressure;     // Configure pressure
    struct arg_int *co2;          // Target CO2 concentration for recalibration
    struct arg_int *value;        // Value
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
        if (strcmp(cmd, "status") == 0) {
            scd4x_values_t *values = &scd4x->values;
            ESP_LOGI(TAG, "SCD4x (serial=0x%012llX ready=%d, enabled=%d):",
                scd4x->serial_number, scd4x_get_data_ready_status(scd4x), scd4x->enabled);
            ESP_LOGI(TAG, "temp_offs=%f °C  altitude=%d m  pressure=%d hPa * co2=%d ppm  temp=%.1f °C  hum=%.1f %%  st=%d",
                    scd4x->temperature_offset, scd4x->altitude, scd4x->pressure,
                    values->co2, values->temperature, values->humidity, scd4x_st_machine_status);
        } else if (strcmp(cmd, "readcfg") == 0) {
            esp_err_t err = scd4x_stop_periodic_measurement(scd4x);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to stop periodic measurement: %d", err);
                return 1;
            }
            // Get sensor info and status
            scd4x->temperature_offset = scd4x_get_temperature_offset(scd4x);
            scd4x->altitude = scd4x_get_sensor_altitude(scd4x);
            err = scd4x_start_periodic_measurement(scd4x);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed start periodic measurement: %d", err);
                return 1;
            }
        } else if (strcmp(cmd, "cfg") == 0) {
            esp_err_t err = scd4x_stop_periodic_measurement(scd4x);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to stop periodic measurement: %d", err);
                return 1;
            }
            if (scd4x_cmd_args.temperature->count == 1) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_set_temperature_offset(scd4x, (float)scd4x_cmd_args.temperature->ival[0]));
            }
            if (scd4x_cmd_args.altitude->count == 1) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_set_sensor_altitude(scd4x, scd4x_cmd_args.altitude->ival[0]));
            }
            if (scd4x_cmd_args.pressure->count == 1) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_set_ambient_pressure(scd4x, scd4x_cmd_args.pressure->ival[0]));
            }
            err = scd4x_start_periodic_measurement(scd4x);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed start periodic measurement: %d", err);
                return 1;
            }
        } else if (strcmp(cmd, "cal") == 0) {
            uint32_t co2;

            if (scd4x_cmd_args.co2->count == 1) {
                co2 = scd4x_cmd_args.co2->ival[0];
            } else {
                co2 = mhz19->values.co2;
            }
            ESP_LOGI(TAG, "Start SCD41 calibration with CO2 value %d", co2);
            scd4x_state_machine_cmd(SCD4X_CMD_FRC, co2);
        } else if (strcmp(cmd, "autocal") == 0) {
            if (scd4x_cmd_args.value->count == 1) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_set_automatic_self_calibration_enabled(scd4x, scd4x_cmd_args.value->ival[0] != 0));
            } else {
                ESP_LOGE(TAG, "no valid arguments");
                return 1;
            }
        } else if (strcmp(cmd, "autoadj") == 0) {
            if (scd4x_cmd_args.value->count == 1) {
                config->scd4x_auto_adjust = scd4x_cmd_args.value->ival[0] != 0;
            } else {
                ESP_LOGE(TAG, "no valid arguments");
                return 1;
            }
        } else if (strcmp(cmd, "start") == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_start_periodic_measurement(scd4x));
        } else if (strcmp(cmd, "startlp") == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_start_low_power_periodic_measurement(scd4x));
        } else if (strcmp(cmd, "stop") == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_stop_periodic_measurement(scd4x));
        } else if (strcmp(cmd, "reset") == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_perfom_factory_reset(scd4x));
        } else if (strcmp(cmd, "test") == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_perform_self_test(scd4x));
        } else if (strcmp(cmd, "save") == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_persist_settings(scd4x));
        } else if (strcmp(cmd, "wakeup") == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_wake_up(scd4x));
        } else if (strcmp(cmd, "down") == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_power_down(scd4x));
        } else if (strcmp(cmd, "reinit") == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_reinit(scd4x));
        } else if (strcmp(cmd, "measure") == 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_measure_single_shot(scd4x));
        } else if (strcmp(cmd, "read") == 0) {
            scd4x_values_t *values = &scd4x->values;
            esp_err_t err = scd4x_read_measurement(scd4x);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read measurement: %d", err);
                return 1;
            }
            ESP_LOGI(TAG, "co2=%d ppm  temp=%.1f °C  hum=%.1f %%  st=%d",
                values->co2, values->temperature, values->humidity, scd4x_st_machine_status);
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
    scd4x_cmd_args.value = arg_int0("v", NULL, "<int>", "Value");
    scd4x_cmd_args.end = arg_end(6);

    const esp_console_cmd_t cmd = {
        .command = "scd",
        .help = "SCD4x. Command: st, cfg, cal, reset, save",
        .hint = NULL,
        .func = &process_scd4x_cmd,
        .argtable = &scd4x_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
