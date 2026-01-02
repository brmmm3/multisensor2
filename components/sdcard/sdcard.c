#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>
#include "esp_vfs.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sd_protocol_defs.h"
#include "sdcard.h"

static const char *TAG = "SD";

#define SDCARD_MAX_FREQ_KHZ 8000

static sdmmc_card_t *card = NULL;


esp_err_t write_text_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Write text file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s (errno=%d)", strerror(errno), errno);
        return ESP_FAIL;
    }
    fwrite(data, strlen(data), 1, f);
    fclose(f);
    return ESP_OK;
}

esp_err_t read_text_file(const char *path, char *buf, uint16_t size)
{
    ESP_LOGI(TAG, "Read text file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s (errno=%d)", strerror(errno), errno);
        return ESP_FAIL;
    }
    fgets(buf, size, f);
    fclose(f);
    return ESP_OK;
}

esp_err_t write_bin_file(const char *path, void *data, uint16_t size)
{
    ESP_LOGI(TAG, "Write binary file %s", path);
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s (errno=%d)", strerror(errno), errno);
        return ESP_FAIL;
    }
    fwrite(data, size, 1, f);
    fclose(f);
    return ESP_OK;
}

esp_err_t read_bin_file(const char *path, void *buf, uint16_t size)
{
    ESP_LOGI(TAG, "Read bin file %s", path);
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s (errno=%d)", strerror(errno), errno);
        return ESP_FAIL;
    }
    if (buf != NULL) {
        fread(buf, size, 1, f);
    }
    fclose(f);
    return ESP_OK;
}

void list_dir(char *path)
{
    struct dirent *dp;
    DIR *dir = opendir(path);
    char p[80];

    if (dir == NULL) {
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
        }
        ESP_LOGI(TAG, "%s: %s %lu",
            (dp->d_type == DT_DIR)
                ? "DIR"
                : "FILE",
            dp->d_name, st.st_size);
    }
    closedir(dir);
}

int sd_card_init(uint8_t cs_pin, uint8_t sclk_pin, uint8_t mosi_pin, uint8_t miso_pin)
{
    esp_err_t err;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = SDCARD_MAX_FREQ_KHZ;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = mosi_pin,
        .miso_io_num = miso_pin,
        .sclk_io_num = sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 38400,
    };

    err = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return -1;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();

    slot_config.gpio_cs = cs_pin;
    slot_config.host_id = host.slot;  // Card handle

    ESP_LOGI(TAG, "Mounting filesystem");
    // IMPORTANT WORKAROUND: Comment out "SDMMC_INIT_STEP(is_spi, sdmmc_init_spi_crc);" in sdmmc_init.c
    err = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Filesystem mounted");

        // Card has been initialized, print its properties
        sdmmc_card_print_info(stdout, card);

        ESP_LOGI(TAG, "Get FAT Info");
        uint64_t bytes_total, bytes_free;
        esp_vfs_fat_info(MOUNT_POINT, &bytes_total, &bytes_free);
        ESP_LOGI(TAG, "FAT FS: %" PRIu64 " kB total, %" PRIu64 " kB free", bytes_total / 1024, bytes_free / 1024);
    } else {
        if (err == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(err));
        }
    }
    return host.slot;
}

int sd_card_mounted()
{
    return card != NULL;
}

esp_err_t sd_get_info(char *buf, uint64_t *bytes_total, uint64_t *bytes_free)
{
    const char* type;

    if (card->is_sdio) {
        type = "SDIO";
    } else if (card->is_mmc) {
        type = "MMC";
    } else {
        if ((card->ocr & SD_OCR_SDHC_CAP) == 0) {
            type = "SDSC";
        } else {
            if (card->ocr & SD_OCR_S18_RA) {
                type = "SDHC/SDXC (UHS-I)";
            } else {
                type = "SDHC";
            }
        }
    }
    uint64_t size = ((uint64_t) card->csd.capacity) * card->csd.sector_size / (1024 * 1024 * 1024);
    sprintf(buf, "%s (%s) %llu GB", card->cid.name, type, size);
    return esp_vfs_fat_info(MOUNT_POINT, bytes_total, bytes_free);
}

int sd_get_file_count(const char *path)
{
    int file_count = 0;

    DIR *dir = opendir(path);
    if (dir == NULL) {
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
    return file_count;
}

esp_err_t ensure_dir(const char *path)
{
    struct stat st;

    ESP_LOGI(TAG, "ensure_dir %s", path);
    if (stat(path, &st) == 0) {
        // Path exists
        if (S_ISDIR(st.st_mode)) {
            return ESP_OK;  // Already a directory
        } else {
            ESP_LOGE(TAG, "Path exists but is not a directory: %s", path);
            return ESP_ERR_INVALID_STATE;
        }
    }
    // Directory does not exist → create it
    if (mkdir(path, 0755) == 0) {  // 0755 = rwxr-xr-x permissions
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Failed to create directory %s (errno: %d)", path, errno);
    return ESP_FAIL;
}

/*static esp_err_t example_file_actions()
{
    char data[64];

    const char *file_hello = MOUNT_POINT"/hello.txt";

    // First create a file.
    snprintf(data, 64, "%s %d %d %s!\n", "Hello", card->cid.mfg_id, card->cid.oem_id, card->cid.name);
    err = write_text_file(file_hello, data);
    if (err != ESP_OK) {
        return -1;
    }

    const char *file_foo = MOUNT_POINT"/foo.txt";

    // Check if destination file exists before renaming
    struct stat st;
    if (stat(file_foo, &st) == 0) {
        // Delete it if it exists
        unlink(file_foo);
    }

    // Rename original file
    ESP_LOGI(TAG, "Renaming file %s to %s", file_hello, file_foo);
    if (rename(file_hello, file_foo) != 0) {
        ESP_LOGE(TAG, "Rename failed");
        return -1;
    }

    esp_err_t err = read_text_file(file_foo, data, sizeof(data));
    if (err != ESP_OK) {
        return -1;
    }

    const char *file_hello_bin = MOUNT_POINT"/hello.bin";
    snprintf(data, 64, "%s %d %d %s!\n", "Hello", card->cid.mfg_id, card->cid.oem_id, card->cid.name);
    err = write_bin_file(file_hello_bin, (uint8_t*)data, 64);
    if (err != ESP_OK) {
        return -1;
    }

    list_dir(MOUNT_POINT);
    return ESP_OK;
}
*/