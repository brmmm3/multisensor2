#include "main.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // RTC command
    struct arg_int *year;
    struct arg_int *month;
    struct arg_int *day;
    struct arg_int *hour;
    struct arg_int *minute;
    struct arg_int *second;
    struct arg_end *end;
} rtc_cmd_args;


int process_rtc_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&rtc_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, rtc_cmd_args.end, argv[0]);
        return 1;
    }
    if (rtc_cmd_args.cmd->count == 1) {
        const char *cmd = rtc_cmd_args.cmd->sval[0];
        if (strcmp(cmd, "get") == 0) {
            struct tm timeinfo;

            ESP_ERROR_CHECK_WITHOUT_ABORT(rtc_get_datetime(rtc->rtc, &timeinfo));
            ESP_LOGI(TAG, "%d.%02d.%02d %02d:%02d:%02d",
                timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        } else if (strcmp(cmd, "set") == 0) {
            struct tm timeinfo;

            ESP_ERROR_CHECK_WITHOUT_ABORT(rtc_get_datetime(rtc->rtc, &timeinfo));
            if (rtc_cmd_args.year->count == 1) {
                timeinfo.tm_year = rtc_cmd_args.year->ival[0];
            }
            if (rtc_cmd_args.month->count == 1) {
                timeinfo.tm_mon = rtc_cmd_args.month->ival[0];
            }
            if (rtc_cmd_args.day->count == 1) {
                timeinfo.tm_mday = rtc_cmd_args.day->ival[0];
            }
            if (rtc_cmd_args.hour->count == 1) {
                timeinfo.tm_hour = rtc_cmd_args.hour->ival[0];
            }
            if (rtc_cmd_args.minute->count == 1) {
                timeinfo.tm_min = rtc_cmd_args.minute->ival[0];
            }
            if (rtc_cmd_args.second->count == 1) {
                timeinfo.tm_sec = rtc_cmd_args.second->ival[0];
            }
            ESP_ERROR_CHECK_WITHOUT_ABORT(rtc_set_datetime(rtc->rtc, &timeinfo));
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

void register_rtc_cmd()
{
    rtc_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    rtc_cmd_args.year = arg_int0("y", NULL, "<int>", "Year");
    rtc_cmd_args.month = arg_int0("m", NULL, "<int>", "Month");
    rtc_cmd_args.day = arg_int0("d", NULL, "<int>", "Day");
    rtc_cmd_args.hour = arg_int0("H", NULL, "<int>", "Hour");
    rtc_cmd_args.minute = arg_int0("M", NULL, "<int>", "Minute");
    rtc_cmd_args.second = arg_int0("S", NULL, "<int>", "Second");
    rtc_cmd_args.end = arg_end(7);

    const esp_console_cmd_t cmd = {
        .command = "rtc",
        .help = "RTC. Command: get, set",
        .hint = NULL,
        .func = &process_rtc_cmd,
        .argtable = &rtc_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
