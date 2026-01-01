#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "include/wifi_sntp.h"

static const char *TAG = "SNTP";

sntp_t sntp;


void time_sync_notification_cb(struct timeval *tv)
{
	ESP_LOGI(TAG, "Notification of a time synchronization event");
    ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %jd sec: %li ms: %li us",
		(intmax_t)tv->tv_sec, tv->tv_usec / 1000, tv->tv_usec % 1000);
    time_t now;
    char strftime_buf[64];

    now = tv->tv_sec;
    //time(&now);
    localtime_r(&now, &sntp.timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &sntp.timeinfo);
    ESP_LOGI(TAG, "#The current date/time in Vienna is: %s", strftime_buf);
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


struct tm *sntp_get_timeinfo()
{
    return &sntp.timeinfo;
}

esp_err_t sntp_obtain_time(void)
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
