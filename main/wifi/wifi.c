#include <string.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "include/wifi_sntp.h"
#include "config.h"
#include "ui/include/ui.h"
#include "ui/include/ui_config.h"
#include "include/wifi.h"

#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif

#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_UNSPECIFIED
#define EXAMPLE_H2E_IDENTIFIER ""
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "WiFi";

static esp_event_handler_instance_t instance_any_id;
static esp_event_handler_instance_t instance_got_ip;
static esp_netif_t *netif = NULL;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool is_scanning = false;


static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (is_scanning) return;
    ESP_LOGI(TAG, "event_handler: %d %d %s", s_retry_num, event_id, event_base);
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGE(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    ESP_LOGI(TAG,"wifi_init_sta");
    if (netif != NULL) return;
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());
    netif = esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    ESP_LOGI(TAG,"wifi_init_sta DONE");
}

void wifi_deinit_sta(void)
{
    ESP_LOGI(TAG,"wifi_deinit_sta");
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &instance_any_id);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &instance_got_ip);
    esp_wifi_disconnect();             // break connection to AP
    esp_wifi_stop();                   // shut down the wifi radio
    esp_wifi_deinit();                 // release wifi resources
    esp_netif_destroy_default_wifi(netif);
    netif = NULL;
    esp_event_loop_delete_default();
    esp_netif_deinit();
    ESP_LOGI(TAG,"wifi_deinit_sta DONE");
}

esp_err_t wifi_connect(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "wifi_connect %s", ssid);
    ui_set_label_text(ui->lbl_wifi_status1, "Connecting...");
    ui_set_label_text(ui->lbl_wifi_status2, "?");
    wifi_config_t wifi_config = {
        .sta = {
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    strncpy((char *)wifi_config.sta.ssid, ssid, 32);
    strncpy((char *)wifi_config.sta.password, password, 64);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually happened
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        esp_err_t err;
        esp_netif_ip_info_t ip_info;

        ui_set_label_text(ui->lbl_wifi_status1, ssid);
        ESP_LOGI(TAG, "Connected to SSID:%s", ssid);
        err = esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);
        if (err == ESP_OK && ip_info.ip.addr != 0) {
            /* Print the local IP address */
            char buf[32];
            sprintf(buf, IPSTR, IP2STR(&ip_info.ip));

            ui_set_label_text(ui->lbl_wifi_status2, buf);
            ESP_LOGI(TAG, "IP Address : %s", buf);
            ESP_LOGI(TAG, "Subnet mask: " IPSTR, IP2STR(&ip_info.netmask));
            ESP_LOGI(TAG, "Gateway    : " IPSTR, IP2STR(&ip_info.gw));
            // Austria/Vienna: CET-1CEST,M3.5.0,M10.5.0/3
            setenv("TZ","CET-1CEST,M3.5.0,M10.5.0/3",1);
            tzset();
            sntp_obtain_time();
        }
        return ESP_OK;
    }
    if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", ssid);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT %x", bits);
    }
    return ESP_FAIL;
}

esp_err_t wifi_init()
{
    ESP_LOGI(TAG, "Initialize WiFi");
    wifi_init_sta();
    ui_set_switch_state(ui->sw_wifi_enable, true);
    wifi_scan();

    uint8_t auto_connect = config->auto_connect;
    if (auto_connect < 4) {
        char *ssid = config->nvs.wifi.ssid[auto_connect];
        char *password = config->nvs.wifi.password[auto_connect];
        if (strlen(ssid) > 0 && strlen(password) > 0) {
            return wifi_connect(ssid, password);
        }
        return ESP_FAIL;
    }
    return ESP_OK;
}

void wifi_uninit()
{
    ESP_LOGI(TAG, "Uninitialize WiFi");
    wifi_deinit_sta();
    ui_set_switch_state(ui->sw_wifi_enable, false);
    ui_set_label_text(ui->lbl_wifi_status1, "-");
    ui_set_label_text(ui->lbl_wifi_status2, "-");
}

bool wifi_initialized()
{
    return netif != NULL;
}

void set_scanning(bool scanning)
{
    is_scanning = scanning;
}
