#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_vfs.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

static const char *TAG = "SD";

#define SDCARD_MAX_FREQ_KHZ 10000

#define MOUNT_POINT "/sdcard"


static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_write_file_bin(const char *path, uint8_t *data, uint32_t size)
{
    ESP_LOGI(TAG, "Write binary file %s", path);
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fwrite(data, size, 1, f);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[64];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

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
    sdmmc_card_t *card;
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

    /*spi_device_interface_config_t dev_cfg = {
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_TXBIT_LSBFIRST | SPI_DEVICE_3WIRE,
        .clock_speed_hz = 1000000,
        .mode = 0,
        .queue_size = 10,
    };

    ESP_LOGI(TAG, "Adding fake LCD SPI device");
    dev_cfg.spics_io_num = PIN_NUM_LCD_CS;
    spi_device_handle_t lcd_handle;
    lcd_handle = (spi_device_handle_t)malloc(sizeof(lcd_handle));
    ESP_ERROR_CHECK(spi_bus_add_device(host.slot, &dev_cfg, &lcd_handle));

    ESP_LOGI(TAG, "Adding fake LCD TOUCH device");
    dev_cfg.spics_io_num = PIN_NUM_TOUCH_CS;
    spi_device_handle_t touch_handle;
    touch_handle = (spi_device_handle_t)malloc(sizeof(touch_handle));
    ESP_ERROR_CHECK(spi_bus_add_device(host.slot, &dev_cfg, &touch_handle));*/

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();

    slot_config.gpio_cs = cs_pin;
    slot_config.host_id = host.slot;  // Card handle

    ESP_LOGI(TAG, "Mounting filesystem");
    // IMPORTANT WORKAROUND: Comment out "SDMMC_INIT_STEP(is_spi, sdmmc_init_spi_crc);" in sdmmc_init.c
    err = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (err != ESP_OK) {
        if (err == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(err));
        }
        return -1;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    ESP_LOGI(TAG, "Get FAT Info");
    uint64_t bytes_total, bytes_free;
    esp_vfs_fat_info(MOUNT_POINT, &bytes_total, &bytes_free);
    ESP_LOGI(TAG, "FAT FS: %" PRIu64 " kB total, %" PRIu64 " kB free", bytes_total / 1024, bytes_free / 1024);

    // First create a file.
    const char *file_hello = MOUNT_POINT"/hello.txt";
    char data[64];
    snprintf(data, 64, "%s %d %d %s!\n", "Hello", card->cid.mfg_id, card->cid.oem_id, card->cid.name);
    err = s_example_write_file(file_hello, data);
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

    err = s_example_read_file(file_foo);
    if (err != ESP_OK) {
        return -1;
    }

    const char *file_hello_bin = MOUNT_POINT"/hello.bin";
    snprintf(data, 64, "%s %d %d %s!\n", "Hello", card->cid.mfg_id, card->cid.oem_id, card->cid.name);
    err = s_example_write_file_bin(file_hello_bin, (uint8_t*)data, 64);
    if (err != ESP_OK) {
        return -1;
    }

    list_dir(MOUNT_POINT);
    return host.slot;
}
