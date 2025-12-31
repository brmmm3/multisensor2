#include <nvs.h>
#include "esp_log.h"
#include "sdcard.h"
#include "config.h"

static const char *TAG = "CFG";

config_t *config;

esp_err_t config_read(void)
{
    esp_err_t err;
    nvs_handle_t handle;

    config = calloc(1, sizeof(config_t));
    // First read config file from SD-Card
    err = read_bin_file(MOUNT_POINT"/config.dat", config, sizeof(config_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read config.dat: %s", esp_err_to_name(err));
        config_write();
    }
    // Then read sensitive data from NVS
    if ((err = nvs_open("config", NVS_READWRITE, &handle)) != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }
    size_t size = sizeof(nvs_config_t);
    if ((err = nvs_get_blob(handle, "config", NULL, &size)) == ESP_OK) {
        err = nvs_get_blob(handle, "config", &config->nvs, &size);
    }
    if (err == ESP_OK && size != sizeof(nvs_config_t)) {
        ESP_LOGW(TAG, "Ignoring invalid configuration in Flash!");
        err = ESP_FAIL;
    }
    nvs_close(handle);
    return err;
}

esp_err_t config_write(void)
{
    esp_err_t err;
    nvs_handle_t handle;
    nvs_config_t nvs = {0};

    memcpy(&config->nvs, &nvs, sizeof(nvs_config_t));
    memset(&config->nvs, 0, sizeof(nvs_config_t));
    // First write sensitive data to NVS
    if ((err = nvs_open("config", NVS_READWRITE, &handle)) != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }
    if ((err = nvs_set_blob(handle, "config", &nvs, sizeof(nvs_config_t))) == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    // Then write config file to SD-Card
    err = write_bin_file(MOUNT_POINT"/config.dat", config, sizeof(config_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read config.dat: %s", esp_err_to_name(err));
    }
    memcpy(&nvs, &config->nvs, sizeof(nvs_config_t));
    return err;
}
