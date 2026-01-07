#include "main.h"

static const char *TAG = "CMD";

struct {
    struct arg_str *cmd;          // FTP command
    struct arg_end *end;
} ftp_cmd_args;


int process_ftp_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&ftp_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, ftp_cmd_args.end, argv[0]);
        return 1;
    }
    if (ftp_cmd_args.cmd->count == 1) {
        const char *cmd = ftp_cmd_args.cmd->sval[0];
        if (strcmp(cmd, "start") == 0) {
            ftp_start();
            ui_set_label_text(ui->lbl_ftp_status, "Started");
        } else if (strcmp(cmd, "stop") == 0) {
            ftp_stop();
            ui_set_label_text(ui->lbl_ftp_status, "Stopped");
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

void register_ftp_cmd()
{
    ftp_cmd_args.cmd = arg_str1(NULL, NULL, "<cmd>", "Command");
    ftp_cmd_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "ftp",
        .help = "FTP. Command: start, stop",
        .hint = NULL,
        .func = &process_ftp_cmd,
        .argtable = &ftp_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
