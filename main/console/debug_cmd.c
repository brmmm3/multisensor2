
#include "main.h"

#include "debug_cmd.h"

static const char *TAG = "CMD";

static struct {
    struct arg_int *gps;    // Debug GPS
    struct arg_int *bme;    // Debug BME280
    struct arg_int *scd;    // Debug SCD41
    struct arg_int *mhz;    // Debug MHZ19
    struct arg_int *yys;    // Debug YYS
    struct arg_int *sps;    // Debug SPS30
    struct arg_int *qmc;    // Debug QMC5883L
    struct arg_int *adx;    // Debug ADXL345
    struct arg_int *sws;    // Debug SW Serial
    struct arg_end *end;
} debug_cmd_args;


int process_debug_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&debug_cmd_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, debug_cmd_args.end, argv[0]);
        return 1;
    }
    if (debug_cmd_args.gps->count == 1) {
        gps->debug = debug_cmd_args.gps->ival[0];
    } else if (debug_cmd_args.bme->count == 1) {
        bmx280lo->debug = debug_cmd_args.bme->ival[0];
        bmx280hi->debug = debug_cmd_args.bme->ival[0];
    } else if (debug_cmd_args.scd->count == 1) {
        scd4x->debug = debug_cmd_args.scd->ival[0];
    } else if (debug_cmd_args.mhz->count == 1) {
        mhz19->debug = debug_cmd_args.mhz->ival[0];
    } else if (debug_cmd_args.yys->count == 1) {
        yys_sensor->debug = debug_cmd_args.yys->ival[0];
    } else if (debug_cmd_args.sps->count == 1) {
        sps30->debug = debug_cmd_args.sps->ival[0];
    } else if (debug_cmd_args.qmc->count == 1) {
        qmc5883l->debug = debug_cmd_args.qmc->ival[0];
    } else if (debug_cmd_args.adx->count == 1) {
        adxl345->debug = debug_cmd_args.adx->ival[0];
    } else if (debug_cmd_args.sws->count == 1) {
        yys_sensor->sw_serial->debug = debug_cmd_args.sws->ival[0];
    } else {
        ESP_LOGE(TAG, "no valid arguments");
        return 1;
    }
    return 0;
}

void register_debug_cmd()
{
    debug_cmd_args.gps = arg_int0("g", "gps", "<0-31>", "Configure GPS debugging");
    debug_cmd_args.bme = arg_int0("b", "bme", "<0-15>", "Configure BME280 debugging");
    debug_cmd_args.scd = arg_int0("s", "scd", "<0-15>", "Configure SCD41 debugging");
    debug_cmd_args.mhz = arg_int0("m", "mhz", "<0-15>", "Configure MHZ19 debugging");
    debug_cmd_args.yys = arg_int0("y", "yys", "<0-15>", "Configure YYS debugging");
    debug_cmd_args.sps = arg_int0("p", "sps", "<0-15>", "Configure SPS30 debugging");
    debug_cmd_args.qmc = arg_int0("q", "qmc", "<0-15>", "Configure QMC5883L debugging");
    debug_cmd_args.adx = arg_int0("a", "adx", "<0-15>", "Configure ADXL345 debugging");
    debug_cmd_args.sws = arg_int0("w", "sws", "<0-15>", "Configure SW Serial debugging");
    debug_cmd_args.end = arg_end(9);

    const esp_console_cmd_t cmd = {
        .command = "debug",
        .help = "configure debugging",
        .hint = NULL,
        .func = &process_debug_cmd,
        .argtable = &debug_cmd_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
