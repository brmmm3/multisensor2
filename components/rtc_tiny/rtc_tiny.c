#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hal/wdt_hal.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"

#include "lwip/dns.h"

#include "rtci2c/rtci2c.h"

#include "rtc_tiny.h"

static const char *TAG = "RTC";

// RTC (DS1307)
#define DS1307_ADDRESS      0 /* let the library figure it out */

rtci2c_context *rtc = NULL;


void time_sync_notification_cb(struct timeval *tv)
{
	ESP_LOGI(TAG, "Notification of a time synchronization event");
    ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %jd sec: %li ms: %li us",
                        (intmax_t)tv->tv_sec,
                        tv->tv_usec / 1000,
                        tv->tv_usec % 1000);
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    now = tv->tv_sec;
    //time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "#The current date/time in Vienna is: %s", strftime_buf);
    if (rtc != NULL) {
        struct tm t;

        rtci2c_set_datetime(rtc, &timeinfo);
        if (!rtci2c_get_datetime(rtc, &t)) {
            ESP_LOGE(TAG, "Date/tate query failed");
        } else {
            ESP_LOGI(TAG, "RTC Current: %02u/%02u/%u %u:%02u:%02u",
                     t.tm_mday, t.tm_mon, t.tm_year, t.tm_hour, t.tm_min, t.tm_sec);
        }
    }
}

static void initialize_sntp(void)
{
	ESP_LOGI(TAG, "Initializing SNTP");
	esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
	//sntp_setservername(0, "pool.ntp.org");
	ESP_LOGI(TAG, "Your NTP Server is %s", CONFIG_NTP_SERVER);
	esp_sntp_setservername(0, CONFIG_NTP_SERVER);
	sntp_set_time_sync_notification_cb(time_sync_notification_cb);
	esp_sntp_init();
}

static esp_err_t obtain_time(void)
{
	initialize_sntp();
	// wait for time to be set
	int retry = 0;
	const int retry_count = 10;
	while (sntp_get_sync_status() != SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
		ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	if (retry == retry_count) return ESP_FAIL;
	return ESP_OK;
}

void rtc_init(i2c_master_bus_handle_t *bus_handle)
{
    i2c_lowlevel_config config = {
        .bus = bus_handle
    };

    rtc = rtci2c_init(RTCI2C_DEVICE_DS1307, DS1307_ADDRESS, &config);
    if (rtc == NULL) {
        ESP_LOGE(TAG, "RTC Initialization failed");
    } else {
        struct tm t;

        if (!rtci2c_get_datetime(rtc, &t)) {
            ESP_LOGE(TAG, "Date/tate query failed");
        } else {
            ESP_LOGI(TAG, "RTC Current: %02u/%02u/%u %u:%02u:%02u",
                     t.tm_mday, t.tm_mon, t.tm_year, t.tm_hour, t.tm_min, t.tm_sec);
        }
    }
}
