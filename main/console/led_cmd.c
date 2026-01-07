#include "main.h"

#include "include/led_cmd.h"

static const char *TAG = "CMD";

struct {
    struct arg_int *red;    // LED Red value
    struct arg_int *green;  // LED Green value
    struct arg_int *blue;   // LED Blue value
    struct arg_end *end;
} led_cmd_args;


int process_led_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&led_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, led_cmd_args.end, argv[0]);
        return 1;
    }
    if (led_cmd_args.red->count == 1 && led_cmd_args.green->count == 1 && led_cmd_args.blue->count == 1) {
        led_strip_set_pixel(led_strip, 0, led_cmd_args.red->ival[0],
                            led_cmd_args.green->ival[1], led_cmd_args.blue->ival[2]);
        led_strip_refresh(led_strip);
    } else {
        ESP_LOGE(TAG, "no valid arguments");
        return 1;
    }
    return 0;
}

void register_led_cmd()
{
    led_cmd_args.red = arg_int1("r", NULL, "<int>", "Red value for LED");
    led_cmd_args.green = arg_int1("g", NULL, "<int>", "Green value for LED");
    led_cmd_args.blue = arg_int1("b", NULL, "<int>", "Blue value for LED");
    led_cmd_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "led",
        .help = "LED",
        .hint = NULL,
        .func = &process_led_cmd,
        .argtable = &led_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
