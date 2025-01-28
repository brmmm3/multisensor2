#include <nvs.h>
#include "esp_log.h"

#include "config.h"

static const char *TAG = "CFG";


esp_err_t config_read(config_t *config)
{
    esp_err_t err;
    nvs_handle_t handle;
    size_t size = sizeof(config_t);

    if ((err = nvs_open("config", NVS_READWRITE, &handle)) != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }
    if ((err = nvs_get_blob(handle, "config", NULL, &size)) == ESP_OK) {
        err = nvs_get_blob(handle, "config", config, &size);
    }
    if (err == ESP_OK && size != sizeof(config_t)) {
        ESP_LOGW(TAG, "Ignoring invalid configuration in Flash!");
        err = ESP_FAIL;
    }
    nvs_close(handle);
    return err;
}

esp_err_t config_write(config_t *config)
{
    esp_err_t err;
    nvs_handle_t handle;

    if ((err = nvs_open("config", NVS_READWRITE, &handle)) != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }
    if ((err = nvs_set_blob(handle, "config", NULL, sizeof(config_t))) == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}
