#include "main.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // TCP command
    struct arg_int *tcp_auto_start;
    struct arg_end *end;
} tcp_cmd_args;


int process_tcp_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tcp_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, tcp_cmd_args.end, argv[0]);
        return 1;
    }
    if (tcp_cmd_args.cmd->count == 1) {
        const char *cmd = tcp_cmd_args.cmd->sval[0];
        if (strcmp(cmd, "start") == 0) {
            tcp_server_start();
        } else if (strcmp(cmd, "stop") == 0) {
            tcp_server_stop();
        } else if (strcmp(cmd, "auto") == 0) {
            if (tcp_cmd_args.tcp_auto_start->count == 1) {
                config->tcp_auto_start = tcp_cmd_args.tcp_auto_start->ival[0] != 0;
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

void register_tcp_cmd()
{
    tcp_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    tcp_cmd_args.tcp_auto_start = arg_int0("a", NULL, "<0-1>", "Auto connect");
    tcp_cmd_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "tcp",
        .help = "TCP. Command: start, stop, auto",
        .hint = NULL,
        .func = &process_tcp_cmd,
        .argtable = &tcp_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
