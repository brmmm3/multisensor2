#include <nvs.h>
#include "esp_log.h"
#include "esp_console.h"
#include "sdcard.h"
#include "main.h"
#include "config.h"

static const char *TAG = "CFG";

config_t *config = NULL;
config_nvs_t *config_nvs = NULL;

// Configuration in NVS

esp_err_t config_nvs_read()
{
    nvs_handle_t handle;
    esp_err_t err;

    ESP_LOGI(TAG, "Read NVS config.");
    if (config_nvs == NULL) {
        config_nvs = calloc(1, sizeof(config_nvs_t));
    }
    // Then read sensitive data from NVS
    if ((err = nvs_open("config", NVS_READWRITE, &handle)) != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }
    size_t size = sizeof(config_nvs_t);
    if ((err = nvs_get_blob(handle, "config", NULL, &size)) == ESP_OK) {
        err = nvs_get_blob(handle, "config", config_nvs, &size);
    }
    if (err == ESP_OK && size != sizeof(config_nvs_t)) {
        ESP_LOGW(TAG, "Ignoring invalid configuration in Flash!");
        return ESP_FAIL;
    }
    nvs_close(handle);
    return err;
}

esp_err_t config_nvs_write()
{
    nvs_handle_t handle;
    esp_err_t err;

    ESP_LOGI(TAG, "Write NVS config.");
    if ((err = nvs_open("config", NVS_READWRITE, &handle)) != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }
    if ((err = nvs_set_blob(handle, "config", config_nvs, sizeof(config_nvs_t))) == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

// configuration on SD-Card

esp_err_t create_default_config_sd()
{
    ESP_LOGI(TAG, "create_default_config_sd");
    memset(config, 0, sizeof(config_t));
    //strcpy(config->mqtt_broker, MQTT_BROKER);
    return config_sd_write();
}

esp_err_t config_sd_read()
{
    esp_err_t err;

    ESP_LOGI(TAG, "Read SD-Card config.");
    if (config == NULL) {
        config = calloc(1, sizeof(config_t));
    }
    err = read_bin_file(MOUNT_POINT"/config.dat", config, sizeof(config_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read config.dat: %s", esp_err_to_name(err));
        err = create_default_config_sd();
    }
    if (config->cfg_version != CONFIG_VERSION) {
        ESP_LOGW(TAG, "Reset config -> Ignore old config");
        err = create_default_config_sd();
    }
    return err;
}

esp_err_t config_sd_write()
{
    esp_err_t err;

    ESP_LOGI(TAG, "Write SD-Card config.");
    config->cfg_version = CONFIG_VERSION;
    err = write_bin_file(MOUNT_POINT"/config.dat", config, sizeof(config_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write config.dat: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t config_read()
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(config_nvs_read());
    return config_sd_read();
}

esp_err_t config_write(void)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(config_nvs_write());
    return config_sd_write();
}

esp_err_t lcd_set_pwr_mode(uint8_t mode)
{
    esp_err_t err = lcd_set_bg_pwr(mode);

    if (err == ESP_OK) {
        config->lcd_pwr = mode;
    }
    return err;
}

esp_err_t gps_set_pwr_mode(uint8_t mode)
{
    esp_err_t err = gps_set_power_mode(gps, mode) < 0 ? ESP_FAIL : ESP_OK;

    if (err == ESP_OK) {
        config->gps_pwr = mode;
    }
    return err;
}

esp_err_t scd4x_set_pwr_mode(uint8_t mode)
{
    esp_err_t err;

    if (mode > 1) {
        err = scd4x_power_down(scd4x);
    } else {
        err = scd4x_init_do(scd4x, mode == 1);
    }
    if (err == ESP_OK) {
        config->scd4x_pwr = mode;
    }
    return err;
}
