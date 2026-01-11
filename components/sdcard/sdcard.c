#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>
#include "esp_vfs.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_vfs_fat.h"
// Needed for LVGL locking to avoid race conditions when using shared SPI bus
#include "esp_lvgl_port.h"
#include "sdmmc_cmd.h"
#include "sd_protocol_defs.h"
#include "sdcard.h"

static const char *TAG = "SD";

#define SDCARD_MAX_FREQ_KHZ 4000

static sdmmc_card_t *sd_card = NULL;
// By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
// For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
// Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
static sdmmc_host_t sd_card_host = SDSPI_HOST_DEFAULT();
static uint8_t sd_cs_pin = 0;


esp_err_t write_text_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Write text file %s", path);
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        lvgl_port_unlock();
        ESP_LOGE(TAG, "Failed to open file for writing: %s (errno=%d)", strerror(errno), errno);
        return ESP_FAIL;
    }
    fwrite(data, strlen(data), 1, f);
    fclose(f);
    lvgl_port_unlock();
    return ESP_OK;
}

esp_err_t read_text_file(const char *path, char *buf, uint32_t size)
{
    ESP_LOGI(TAG, "Read text file %s", path);
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        lvgl_port_unlock();
        ESP_LOGE(TAG, "Failed to open file for reading: %s (errno=%d)", strerror(errno), errno);
        return ESP_FAIL;
    }
    fgets(buf, size, f);
    fclose(f);
    lvgl_port_unlock();
    return ESP_OK;
}

esp_err_t write_bin_file(const char *path, void *data, uint32_t size)
{
    ESP_LOGI(TAG, "Write binary file %s", path);
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        lvgl_port_unlock();
        ESP_LOGE(TAG, "Failed to open file for writing: err(%d)=%s", errno, strerror(errno));
        return ESP_FAIL;
    }
    fwrite(data, size, 1, f);
    fclose(f);
    lvgl_port_unlock();
    return ESP_OK;
}

uint32_t read_bin_file(const char *path, void *buf, uint32_t size)
{
    ESP_LOGI(TAG, "Read bin file %s", path);
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        lvgl_port_unlock();
        ESP_LOGE(TAG, "Failed to open file for reading: %s (errno=%d)", strerror(errno), errno);
        return 0;
    }
    if (buf == NULL) return 0;
    uint32_t len = fread(buf, size, 1, f);
    fclose(f);
    lvgl_port_unlock();
    return len;
}

FILE *open_bin_file(const char *path)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return NULL;
    FILE *fd = fopen(path, "rb");
    lvgl_port_unlock();
    return fd;
}

uint32_t read_bin_file_part(FILE *f, void *buf, uint32_t size)
{
    if (buf == NULL) return 0;
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return 0;
    uint32_t len = fread(buf, size, 1, f);
    lvgl_port_unlock();
   return len;
}

int close_bin_file(FILE *f)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return 0;
    int res = fclose(f);
    lvgl_port_unlock();
    return res;
}

char *get_data_file_path(const char *path)
{
    static char p[80];

    sprintf(p, "%s/data/%s", MOUNT_POINT, path);
    return p;
}

FILE *open_data_file(const char *path)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return NULL;
    FILE *f = fopen(get_data_file_path(path), "rb");
    lvgl_port_unlock();
    return f;
}

uint32_t read_data_file_part(FILE *f, void *buf, uint32_t size)
{
    uint32_t res = read_bin_file_part(f, buf, size);
    return res;
}

int close_data_file(FILE *f)
{
    return close_bin_file(f);
}

uint32_t read_data_file(const char *path, void *buf, uint32_t size)
{
    return read_bin_file(get_data_file_path(path), buf, size);
}

esp_err_t remove_data_file(const char *path)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    if (remove((get_data_file_path(path))) != 0) {
        lvgl_port_unlock();
        ESP_LOGE(TAG, "Failed to delete file: %s (error: %d)", path, errno);
        return ESP_FAIL;
    }
    lvgl_port_unlock();
    return ESP_OK;
}

DIR *sd_open_dir(char *path)
{
    ESP_LOGD(TAG, "sd_open_dir %s", path);
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return NULL;
    DIR *dp = opendir(path);
    lvgl_port_unlock();
    return dp;
}

DIR *sd_open_data_dir()
{
    char path[32];

    sprintf(path, "%s/data", MOUNT_POINT);
    return sd_open_dir(path);
}

int sd_read_dir(DIR *dir, char *buf, int maxlen, int maxcnt)
{
    struct dirent *dp;
    int pos = 0;

    maxlen -= 80;
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return 0;
    while ((dp = readdir(dir)) != NULL) {
        strcpy(&buf[pos], dp->d_name);
        pos += strlen(dp->d_name);
        buf[pos++] = '\n';
        if (pos >= maxlen || (maxcnt > 0 && --maxcnt == 0)) break;
    }
    lvgl_port_unlock();
    return pos;
}

void sd_closedir(DIR *dir)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return;
    closedir(dir);
    lvgl_port_unlock();
}

void list_dir(char *path)
{
    struct dirent *dp;

    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return;
    DIR *dir = opendir(path);
    char p[80];

    if (dir == NULL) {
        lvgl_port_unlock();
        ESP_LOGE(TAG, "Can't Open Dir.");
        return;
    }
    int pos = strlen(MOUNT_POINT) + 1;
    strcpy(p, MOUNT_POINT);
    p[pos - 1] = '/';

    while ((dp = readdir(dir)) != NULL) {
        struct stat st;
        strcpy(&p[pos], dp->d_name);
        if (stat(p, &st) == 0) {
            // TODO
        }
        ESP_LOGI(TAG, "%s: %s %lu",
            (dp->d_type == DT_DIR) ? "DIR" : "FILE",
            dp->d_name, st.st_size);
    }
    closedir(dir);
    lvgl_port_unlock();
}

sd_fat_info_t *sd_get_fat_info()
{
    static sd_fat_info_t fat_info;

    ESP_LOGI(TAG, "Get FAT Info");
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return NULL;
    esp_vfs_fat_info(MOUNT_POINT, &fat_info.bytes_total, &fat_info.bytes_free);
    lvgl_port_unlock();
    ESP_LOGI(TAG, "FAT FS: %" PRIu64 " MB total, %" PRIu64 " MB free",
        fat_info.bytes_total / (1024 * 1024), fat_info.bytes_free / (1024 * 1024));
    return &fat_info;
}

esp_err_t spi_bus_init(uint8_t cs_pin, uint8_t sclk_pin, uint8_t mosi_pin, uint8_t miso_pin)
{
    ESP_LOGI(TAG, "Initialize SPI bus");
    sd_card_host.max_freq_khz = SDCARD_MAX_FREQ_KHZ;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = mosi_pin,
        .miso_io_num = miso_pin,
        .sclk_io_num = sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 38400,
    };

    esp_err_t err = spi_bus_initialize(sd_card_host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
    }
    return err;
}

esp_err_t sd_card_mount_fs()
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();

    slot_config.gpio_cs = sd_cs_pin;
    slot_config.host_id = sd_card_host.slot;  // Card handle

    ESP_LOGI(TAG, "Mounting filesystem");
    // IMPORTANT WORKAROUND: Comment out "SDMMC_INIT_STEP(is_spi, sdmmc_init_spi_crc);" in sdmmc_init.c
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    esp_err_t err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &sd_card_host, &slot_config, &mount_config, &sd_card);
    lvgl_port_unlock();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Filesystem mounted");
        // Card has been initialized, print its properties
        if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
        sdmmc_card_print_info(stdout, sd_card);
        sd_get_fat_info();
        lvgl_port_unlock();
    } else {
        if (err == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(err));
        }
    }
    return err;
}

int sd_card_init(uint8_t cs_pin, uint8_t sclk_pin, uint8_t mosi_pin, uint8_t miso_pin)
{
    esp_err_t err;

    if ((err = spi_bus_init(cs_pin, sclk_pin, mosi_pin, miso_pin)) != ESP_OK) return -1;

    ESP_LOGI(TAG, "Initialize SD card");
    sd_cs_pin = cs_pin;

    if ((err = sd_card_mount_fs()) != ESP_OK) return -1;
    return sd_card_host.slot;
}

bool sd_card_mounted(bool check)
{
    if (!check) return sd_card != NULL;

    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return false;
    DIR* dir = opendir(MOUNT_POINT);
    if (dir != NULL) {
        closedir(dir);
        lvgl_port_unlock();
        return true;
    }
    lvgl_port_unlock();
    sd_card = NULL;
    return false;
}

esp_err_t sd_card_get_info(char *buf, uint64_t *bytes_total, uint64_t *bytes_free)
{
    const char* type;

    if (sd_card == NULL) return ESP_FAIL;
    if (sd_card->is_sdio) {
        type = "SDIO";
    } else if (sd_card->is_mmc) {
        type = "MMC";
    } else {
        if ((sd_card->ocr & SD_OCR_SDHC_CAP) == 0) {
            type = "SDSC";
        } else {
            if (sd_card->ocr & SD_OCR_S18_RA) {
                type = "SDHC/SDXC (UHS-I)";
            } else {
                type = "SDHC";
            }
        }
    }
    uint64_t size = ((uint64_t) sd_card->csd.capacity) * sd_card->csd.sector_size / (1024 * 1024 * 1024);
    sprintf(buf, "%s (%s) %llu GB", sd_card->cid.name, type, size);
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    esp_err_t err = esp_vfs_fat_info(MOUNT_POINT, bytes_total, bytes_free);
    lvgl_port_unlock();
    return err;
}

int sd_card_get_file_count(const char *path)
{
    int file_count = 0;

    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return -1;
    DIR *dir = opendir(path);
    if (dir == NULL) {
        lvgl_port_unlock();
        ESP_LOGE(TAG, "Failed to open directory %s", path);
        return -1;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        // Skip "." and ".." entries
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        if (entry->d_type != DT_DIR) {
            file_count++;
        }
    }
    closedir(dir);
    lvgl_port_unlock();
    return file_count;
}

esp_err_t ensure_dir(const char *path)
{
    struct stat st;

    ESP_LOGI(TAG, "ensure_dir %s", path);
    if (!lvgl_port_lock(pdMS_TO_TICKS(1000))) return ESP_FAIL;
    if (stat(path, &st) == 0) {
        lvgl_port_unlock();
        // Path exists
        if (S_ISDIR(st.st_mode)) return ESP_OK;  // Already a directory
        ESP_LOGE(TAG, "Path exists but is not a directory: %s", path);
        return ESP_ERR_INVALID_STATE;
    }
    // Directory does not exist → create it
    if (mkdir(path, 0755) == 0) {
        lvgl_port_unlock();
        return ESP_OK;  // 0755 = rwxr-xr-x permissions
    }
    lvgl_port_unlock();
    ESP_LOGE(TAG, "Failed to create directory %s (errno: %d)", path, errno);
    return ESP_FAIL;
}
