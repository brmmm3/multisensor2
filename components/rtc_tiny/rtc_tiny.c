#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "rtc_tiny.h"

static const char *TAG = "RTC";

// RTC (DS1307)
#define DS1307_ADDRESS      0 /* let the library figure it out */


esp_err_t rtc_set_datetime(rtci2c_context *rtc, struct tm *timeinfo)
{
    struct tm t;

    rtci2c_set_datetime(rtc, timeinfo);
    if (!rtci2c_get_datetime(rtc, &t)) {
        ESP_LOGE(TAG, "Date/tate query failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "RTC Current: %02u/%02u/%u %u:%02u:%02u",
                t.tm_mday, t.tm_mon, t.tm_year, t.tm_hour, t.tm_min, t.tm_sec);
    return ESP_OK;
}


esp_err_t rtc_get_datetime(rtci2c_context *rtc, struct tm *timeinfo)
{
    if (!rtci2c_get_datetime(rtc, timeinfo)) {
        ESP_LOGE(TAG, "Date/tate query failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}


esp_err_t rtc_init(rtc_t **rtc_ptr, i2c_master_bus_handle_t *bus_handle)
{
    ESP_LOGI(TAG, "Initialize RTC");

    rtc_t *rtc = pvPortMalloc(sizeof(rtc_t));
    i2c_lowlevel_config config = {
        .bus = bus_handle
    };
    struct tm t;

    rtc->rtc = rtci2c_init(RTCI2C_DEVICE_DS1307, DS1307_ADDRESS, &config);
    if (rtc->rtc == NULL) {
        ESP_LOGE(TAG, "RTC Initialization failed");
        return ESP_FAIL;
    }
    if (!rtci2c_get_datetime(rtc->rtc, &t)) {
        ESP_LOGE(TAG, "Date/tate query failed");
        return ESP_FAIL;
    }
    *rtc_ptr = rtc;
    ESP_LOGI(TAG, "RTC Current: %02u/%02u/%u %u:%02u:%02u",
                t.tm_mday, t.tm_mon, t.tm_year, t.tm_hour, t.tm_min, t.tm_sec);
    return ESP_OK;
}
