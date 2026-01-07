#include "main.h"

#include "include/console.h"
#include "include/debug_cmd.h"
#include "include/led_cmd.h"
#include "include/pwr_cmd.h"
#include "include/bme_cmd.h"
#include "include/mhz19_cmd.h"
#include "include/scd4x_cmd.h"
#include "include/sps30_cmd.h"
#include "include/wifi_cmd.h"
#include "include/rtc_cmd.h"
#include "include/ftp_cmd.h"
#include "include/tcp_cmd.h"
//#include "include/mqtt_cmd.h"
#include "include/config_cmd.h"

static const char *TAG = "CMD";

esp_err_t console_init()
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    ESP_LOGI(TAG, "console_init");
    esp_console_register_help_command();
    register_debug_cmd();
    register_pwr_cmd();
    register_led_cmd();
    register_bme_cmd();
    register_mhz19_cmd();
    register_scd4x_cmd();
    register_sps30_cmd();
    register_wifi_cmd();
    register_rtc_cmd();
    register_ftp_cmd();
    register_tcp_cmd();
    //register_mqtt_cmd();
    register_config_cmd();

    repl_config.prompt = "MS>";
    repl_config.max_cmdline_length = 256;

    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    ESP_LOGI(TAG, "init_console done.");
    return ESP_OK;
}
