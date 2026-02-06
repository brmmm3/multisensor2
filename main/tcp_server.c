#include <string.h>
#include <sys/errno.h>
#include <sys/socket.h>
#include <netdb.h>
#include "config.h"
#include "mbedtls/base64.h"
#include "adxl345.h"
#include "esp_log.h"
#include "mhz19.h"
#include "sdcard.h"
#include "wifi/include/wifi.h"
#include "main.h"

static const char *TAG = "TCP";

#define PORT                    3333
#define MAX_CLIENTS             4          // Max simultaneous clients
#define BUFFER_SIZE             1024

typedef struct {
    uint8_t *data;
    size_t len;
} msg_t;

static msg_t msg_array[16];
static uint8_t msg_num;

static TaskHandle_t tcp_server_task_handle = NULL;
static QueueHandle_t tx_queue = NULL;
static char rx_buffer[BUFFER_SIZE];
static uint8_t update_all_cnt = 0;

bool tcp_server_running = false;
bool tcp_send_values = false;
uint8_t tcp_client_cnt = 0;
static bool opt_format = false;

static const char *help =
    "val 0=off,1=send optimized,2=send optimized all,3=send json,4=send json all\n"
    "lsr  Show root files\n"
    "ls   Show files in data\n"
    "cp *|filename  Copy data (all) file(s)\n"
    "mv *|filename  Move data (all) file(s)\n"
    "rm *|filename  Remove data (all) file(s)\n"
    "rec 0|1\n"
    "free\n"
    "config\n"
    "lock\n"
    "unlock\n"
    "status [0|1]\n"
    "pwr lcd 0-3|gps 0-2|scd 0-2|wifi 0-2|mode 0-3\n"
    "reset gps soft|full\n"
    "gps\n"
    "bme280lo\n"
    "bme280hi\n"
    "mhz19\n"
    "scd4x {cal [co2]|autoadj int|status|val [0|1]|temp_offs|altitude|pressure]\n"
    "yys\n"
    "sps30\n"
    "auto con 0-4|rec 0-1|tcp 0-1|ftp 0-1\n"
    "save\n"
    "tab 0-5\n"
    "rtc [yy.mm.dd HH:MM:SS]\n";


static bool send_message(const char *buf, int len)
{
    msg_t *msg = &msg_array[msg_num++];
    if (msg_num > 15) msg_num = 0;
    msg->data = pvPortMalloc(len);
    memcpy(msg->data, buf, len);
    msg->len = len;
    return xQueueSend(tx_queue, msg, pdMS_TO_TICKS(100)) != pdPASS;
}

static bool send_data_to_client(int client_sock, uint8_t *data, int to_write)
{
    int written = 0;
    int retries = 50;
    while (to_write > 0) {
        int ret = send(client_sock, data + written, to_write, 0);
        if (ret < 0) {
            if (errno == EAGAIN) {
                if (--retries > 0) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
            }
            ESP_LOGE(TAG, "SEND failed: errno %u", errno);
            return false;
        }
        to_write -= ret;
        written += ret;
    }
    return true;
}

static int get_date_time(char *buf)
{
   uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    get_current_date_time(&year, &month, &day, &hour, &min, &sec);
    const char *fmt = opt_format ? "*%u,%u.%02d.%02d,%02d:%02d:%02d,%d\n"
                                 : "{id=%u,date=\"%u.%02d.%02d\",time=\"%02d:%02d:%02d\",rssi=%d}\n";
    return sprintf(buf, fmt, E_SENSOR_DATETIME,
        year, month, day, hour, min, sec, status.rssi);
}

static int get_config(char *buf)
{
    const char *fmt = opt_format ? "*%u,%u,%u,%u,"
                                 : "{id=%u,cfg_version=%u,auto_connect=%u,auto_record=%u,";
    int len = sprintf(rx_buffer, fmt, E_SENSOR_CONFIG,
        config->cfg_version, config->wifi_auto_connect_idx, config->auto_record);
    fmt = opt_format ? "%u,%u,%u,%u,%u,"
                     : "lcd_pwr=%u,gps_pwr=%u,scd4x_pwr=%u,wifi_pwr=%u,mode_pwr=%u,";
    len += sprintf(&rx_buffer[len], fmt,
        config->lcd_pwr, config->gps_pwr, config->scd4x_pwr, config->wifi_pwr, config->mode_pwr);
    fmt = opt_format ? "%u,%u"
                     : "tcp_auto_start=%u,ftp_auto_start=%u";
    len += sprintf(&rx_buffer[len], fmt, config->tcp_auto_start, config->ftp_auto_start);
    fmt = opt_format ? ",%u=%s"
                     : ",ssid%u=%s";
    for (int i = 0; i < 4; i++) {
        if (strlen(config_nvs->wifi.ssid[i]) > 0) {
            len += sprintf(&rx_buffer[len], fmt, i, config_nvs->wifi.ssid[i]);
        }
    }
    if (opt_format) {
        len += sprintf(&rx_buffer[len], "\n");
    } else {
        len += sprintf(&rx_buffer[len], "}\n");
    }
    return len;
}

static int get_status(char *buf)
{
    const char *fmt = opt_format ? "*%u,%u,%u,%u,%u,%s,%d\n"
                                 : "{id=%u,force_update=%u,recording=%u,record_pos=%u,file_cnt=%u,filename=\"%s\",rssi=%d}\n";
    return sprintf(buf, fmt, E_SENSOR_STATUS,
        status.force_update, status.recording, status.record_pos, status.file_cnt, status.filename, status.rssi);
}

static int get_gps_values(char *buf)
{
    const char *fmt = opt_format ? "*%u,%u,%lu,%lu,%f,%c,%f,%c,%f,%f,%c,%u,%f,%f,%f,%x,%u,%u\n"
                                 : "{id=%u,sat=%u,date=%lu,time=%lu,lat=%f,lat_unit=\"%c\",lng=%f,lng_unit=\"%c\",alt=%f,spd=%f,mode_3d=\"%c\",sats=%u,pdop=%f,hdop=%f,vdop=%f,status=0x%x,data_cnt=%u,error_cnt=%u}\n";
    return sprintf(buf, fmt, E_SENSOR_GPS,
            gps_values.sat, (unsigned long)gps_values.date, (unsigned long)gps_values.time, gps_values.lat, gps_values.ns,
            gps_values.lng, gps_values.ew, gps_values.altitude, gps_values.speed, gps_values.mode_3d, gps_values.sats,
            gps_values.pdop, gps_values.hdop, gps_values.vdop, gps_values.status, gps_values.data_cnt, gps_values.error_cnt);
}

static int get_bme280_values(char *buf, bmx280_values_t *values)
{
    const char *fmt = opt_format ? "*%u,%f,%f,%f,%f\n"
                                 : "{id=%u,temp=%f,hum=%f,press=%f,alt=%f}\n";
    return sprintf(buf, fmt, E_SENSOR_BMX280_LO,
        values->temperature, values->humidity, values->pressure, values->altitude);
}

static int get_mhz19_values(char *buf, mhz19_values_t *values)
{
    const char *fmt = opt_format ? "*%u,%u,%u,%u\n"
                                 : "{id=%u,co2=%u,temp=%u,st=%u}\n";
    return sprintf(buf, fmt, E_SENSOR_MHZ19,
        values->co2, values->temp, values->status);
}

static int get_scd4x_values(char *buf, scd4x_values_t *values)
{
    const char *fmt = opt_format ? "*%u,%u,%f,%f,%u\n"
                                 : "{id=%u,co2=%u,temp=%f,hum=%f,st=%u}\n";
    return sprintf(buf, fmt, E_SENSOR_SCD4X,
        values->co2, values->temperature, values->humidity, scd4x_st_machine_status);
}

static int get_yys_values(char *buf)
{
    const char *fmt = opt_format ? "*%u,%f,%f,%f,%f\n"
                                 : "{id=%u,o2=%f,co=%f,h2s=%f,ch4=%f}\n";
    return sprintf(buf, fmt, E_SENSOR_YYS,
        yys_get_o2(yys_sensor), yys_get_co(yys_sensor),
        yys_get_h2s(yys_sensor), yys_get_ch4(yys_sensor));
}

static int get_sps30_values(char *buf, sps30_values_t *values)
{
    const char *fmt = opt_format ? "*%u,%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n"
                                 : "{id=%u,status=%lu,pm0_5=%f,pm1_0=%f,p1_0=%f,pm2_5=%f,p2_5=%f,pm4_0=%f,p4_0=%f,pm10_0=%f,p10_0=%f,typ_part_sz=%f}\n";
    return sprintf(buf, fmt, E_SENSOR_SPS30,
        (unsigned long)values->status, values->nc_0p5, values->mc_1p0, values->nc_1p0, values->mc_2p5, values->nc_2p5,
        values->mc_4p0, values->nc_4p0, values->mc_10p0, values->nc_10p0, values->typical_particle_size);
}

static int get_adxl345_values(char *buf, adxl345_values_t *values)
{
    const char *fmt = opt_format ? "*%u,%f,%f,%f,%f,%f,%f,%f\n"
                                 : "{id=%u,x=%f,y=%f,z=%f,abs=%f,offs=%f %f %f}\n";
    return sprintf(buf, fmt, E_SENSOR_ADXL345,
        values->accel_x, values->accel_y, values->accel_z, values->accel_abs,
        values->accel_offset_x, values->accel_offset_y, values->accel_offset_z);
}

static int get_qmc5883l_values(char *buf, qmc5883l_values_t *values)
{
    const char *fmt = opt_format ? "*%u,%f,%f,%f,%f,%u\n"
                                 : "{id=%u,x=%f,y=%f,z=%f,range=%f,st=%u}\n";
    return sprintf(buf, fmt, E_SENSOR_QMC5883L,
        values->status, values->mag_x, values->mag_y, values->mag_z, values->range);
}

static esp_err_t client_cmd_filecnt(int client_sock)
{
    DIR *dir = sd_open_data_dir();

    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open data dir");
        return ESP_FAIL;
    }
    int file_cnt = sd_dir_file_cnt(dir);
    int len = sprintf(rx_buffer, "%d", file_cnt);
    send_data_to_client(client_sock, (uint8_t *)rx_buffer,len);
    sd_closedir(dir);
    return ESP_OK;
}

static esp_err_t client_cmd_ls(int client_sock)
{
    DIR *dir = sd_open_data_dir();

    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open data dir");
        return ESP_FAIL;
    }
    int file_cnt = sd_dir_file_cnt(dir);
    while (file_cnt > 0) {
        int len = sd_read_dir(dir, rx_buffer, BUFFER_SIZE, 0, 0);
        if (len == 0) break;
        send_data_to_client(client_sock, (uint8_t *)rx_buffer,len);
    }
    sd_closedir(dir);
    return ESP_OK;
}

static esp_err_t client_cmd_lsr(int client_sock)
{
    DIR *dir = sd_open_dir(MOUNT_POINT);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open data dir");
        return ESP_FAIL;
    }
    while (true) {
        int len = sd_read_dir(dir, rx_buffer, BUFFER_SIZE, 0, 0);
        if (len == 0) break;
        send_data_to_client(client_sock, (uint8_t *)rx_buffer,len);
    }
    sd_closedir(dir);
    return ESP_OK;
}

static esp_err_t send_data_file_part(int client_sock, char *path, char *buf, size_t size)
{
    uint8_t encoded_data[BUFFER_SIZE];
    size_t encoded_len = 0;

    int ret = mbedtls_base64_encode(encoded_data, BUFFER_SIZE, &encoded_len, (const uint8_t *)buf, size);
    if (ret != 0) {
        ESP_LOGE(TAG, "Base64 encode failed: %d", ret);
        return ESP_FAIL;
    }
    if (!send_data_to_client(client_sock, (uint8_t *)encoded_data,encoded_len)) {
        ESP_LOGE(TAG, "Failed to send file %s", path);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t send_data_file(int client_sock, char *path)
{
    size_t buffer_size2 = BUFFER_SIZE >> 1;
    esp_err_t err = ESP_OK;

    FILE *f = open_data_file(path);
    if (f == NULL) return ESP_FAIL;
    if (!send_data_to_client(client_sock, (uint8_t *)":",1)) {
        ESP_LOGE(TAG, "Failed to send data file %s", path);
        err = ESP_FAIL;
    } else {
        uint32_t cnt;
        while (true) {
            uint32_t len = read_data_file_part(f, rx_buffer, BUFFER_SIZE);
            if (len == 0) break;
            cnt = buffer_size2;
            if (len < cnt) cnt = len;
            if ((err = send_data_file_part(client_sock, path, rx_buffer, cnt)) != ESP_OK) break;
            if (len > buffer_size2) {
                len -= buffer_size2;
                if ((err = send_data_file_part(client_sock, path, &rx_buffer[buffer_size2], len)) != ESP_OK) break;
            }
        }
    }
    close_data_file(f);
    if (!send_data_to_client(client_sock, (uint8_t *)"\n",1)) {
        ESP_LOGE(TAG, "Failed to send data file %s", path);
        err = ESP_FAIL;
    }
    return err;
}

static esp_err_t send_data_file_start(int client_sock, char *path, bool remove_file)
{
    char buf[32];
    esp_err_t err;

    int len = sprintf(buf, ":%s", path);
    if (!send_data_to_client(client_sock, (uint8_t *)buf,len)) {
        ESP_LOGE(TAG, "Failed to send data file %s", path);
        return ESP_FAIL;
    }
    if ((err = send_data_file(client_sock, path)) != ESP_OK) {
        return err;
    }
    if (!remove_file) return ESP_OK;
    return remove_data_file(path);
}

static esp_err_t send_all_data_files(int client_sock, bool remove_file)
{
    char path[32];
    esp_err_t err;

    DIR *dir = sd_open_data_dir();
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open data dir");
        return ESP_FAIL;
    }
    while (true) {
        int len = sd_read_dir(dir, path, 32, 0, 1);
        if (len < 2) break;
        path[len - 1] = 0;
        ESP_LOGI(TAG, "Send file %s", path);
        if ((err = send_data_file_start(client_sock, path, remove_file)) != ESP_OK) {
            break;
        }
    }
    sd_closedir(dir);
    return ESP_OK;
}

static esp_err_t remove_all_data_files()
{
    char path[32];
    esp_err_t err;

    DIR *dir = sd_open_data_dir();
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open data dir");
        return ESP_FAIL;
    }
    while (true) {
        int len = sd_read_dir(dir, path, 32, 0, 1);
        if (len < 2) break;
        path[len - 1] = 0;
        if ((err = remove_data_file(path)) != ESP_OK) {
            break;
        }
    }
    sd_closedir(dir);
    return ESP_OK;
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int listen_sock;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    // Create TCP socket
    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %u", errno);
        tcp_server_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Bind to port
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);

    int err = bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %u", errno);
        close(listen_sock);
        tcp_server_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Listen
    err = listen(listen_sock, MAX_CLIENTS);
    if (err < 0) {
        ESP_LOGE(TAG, "Error listening: errno %u", errno);
        close(listen_sock);
        tcp_server_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on port %u", PORT);
    tcp_server_running = true;

    while (true) {
        // Accept new client
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &client_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %u", errno);
            continue;
        }
        tcp_client_cnt++;
        update_all_cnt = 2;
        force_update_all = true;

        // Convert IP to string
        inet_ntoa_r(client_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Client connected from %s", addr_str);

        // Set Non-blocking flag
        int flags = fcntl(client_sock, F_GETFL, 0);
        fcntl(client_sock, F_SETFL, flags | O_NONBLOCK);

        int len = sprintf(rx_buffer, "{id=%u,name=\"MultiSensor V2.0\"}\n", E_SENSOR_INFO);
        send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);

        len = sprintf(rx_buffer, "{id=%u,lat=\"%c\",lng=\"%c\",co2=\"ppm\",temp=\"°C\",hum=\"%%\",o2=\"%%\",co=\"ppm\",h2s=\"ppm\",ch4=\"ppm\",",
            E_SENSOR_UNITS,
            gps_values.ns, gps_values.ew);
        len += sprintf(&rx_buffer[len], "pm0_5=\"#/cm3\",typ_part_sz=\"um\",pm1_0=\"ug/cm3\",p1_0=\"#/cm3\",");
        len += sprintf(&rx_buffer[len], "pm2_5=\"ug/cm3\",p2_5=\"#/cm3\",pm4_0=\"ug/cm3\",p4_0=\"#/cm3\",pm10_0=\"ug/cm3\",p10_0=\"#/cm3\",");
        len += sprintf(&rx_buffer[len], "adxl345=\"g\",qmc5883l=\"gauss\"}\n");
        send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);

        // Echo loop for this client
        while (true) {
            len = recv(client_sock, rx_buffer, BUFFER_SIZE - 1, 0);
            if (len < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    // No data available right now - normal in non-blocking mode
                } else {
                    ESP_LOGE(TAG, "recv failed: errno %u", errno);
                    break;
                }
            }
            if (len == 0) {
                ESP_LOGI(TAG, "Client %s disconnected", addr_str);
                break;
            }
            rx_buffer[len] = 0;  // NULL terminate string
            while (--len > 0) {
                if (rx_buffer[len] > 31) break;
                rx_buffer[len] = 0;
            }
            if (len > 0) {
                char *response = "OK\n";

                ESP_LOGI(TAG, "CMD: %s", rx_buffer);
                if (strcmp(rx_buffer, "bye") == 0) break;
                if (strcmp(rx_buffer, "help") == 0) {
                    send_data_to_client(client_sock, (uint8_t *)help, strlen(help));
                } else if (strncmp(rx_buffer, "val ", 4) == 0) {
                    int value = atoi(&rx_buffer[4]);
                    if (value == 0) {
                        tcp_send_values = false;
                    } else {
                        tcp_send_values = true;
                        force_update_all = true;
                        opt_format = value < 3;
                        if (value == 1 || value == 3) update_all_cnt = 2;
                    }
                } else if (strcmp(rx_buffer, "lsr") == 0) {
                    // Show root files on SD-Card
                    if ((err = client_cmd_lsr(client_sock)) != ESP_OK) {
                        response = "ERR\n";
                    }
                } else if (strcmp(rx_buffer, "filecnt") == 0) {
                    // Show file count in data folder
                    if ((err = client_cmd_filecnt(client_sock)) != ESP_OK) {
                        response = "ERR\n";
                    }
                } else if (strcmp(rx_buffer, "ls") == 0) {
                    // Show data files on SD-Card
                    if ((err = client_cmd_ls(client_sock)) != ESP_OK) {
                        response = "ERR\n";
                    }
                } else if (strncmp(rx_buffer, "cp ", 3) == 0) {
                    // Download data file and keep it on SD-Card
                    if (strcmp(&rx_buffer[3], "*") == 0) {
                        if ((err = send_all_data_files(client_sock, false)) != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to send all data files");
                            response = "ERR\n";
                        }
                    } else if ((err = send_data_file_start(client_sock, &rx_buffer[3], false)) != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to send data file");
                        response = "ERR\n";
                    }
                } else if (strncmp(rx_buffer, "mv ", 3) == 0) {
                    // Download data file and remove it on SD-Card
                    if (strcmp(&rx_buffer[3], "*") == 0) {
                        if ((err = send_all_data_files(client_sock, true)) != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to send all data files");
                            response = "ERR\n";
                        } else {
                            status.file_cnt = 0;
                            show_sd_card_info(0);
                        }
                    } else if ((err = send_data_file_start(client_sock, &rx_buffer[3], true)) != ESP_OK) {
                        response = "ERR\n";
                    } else if (status.file_cnt > 0) {
                        status.file_cnt--;
                        show_sd_card_info(status.file_cnt);
                    }
                } else if (strncmp(rx_buffer, "rm ", 3) == 0) {
                    // Remove file on SD-Card. A path "*" removes all data files.
                    if (strcmp(&rx_buffer[3], "*") == 0) {
                        if ((err = remove_all_data_files()) != ESP_OK) {
                            response = "ERR\n";
                        } else {
                            status.file_cnt = 0;
                            show_sd_card_info(0);
                        }
                    } else if ((err = remove_data_file(&rx_buffer[3])) != ESP_OK) {
                        response = "ERR\n";
                    } else if (status.file_cnt > 0) {
                        status.file_cnt--;
                        show_sd_card_info(status.file_cnt);
                    }
                } else if (strcmp(rx_buffer, "rec 1") == 0) {
                    ui_set_switch_state(ui->sw_record, true);
                    ui_sd_record_set_value(true);
                } else if (strcmp(rx_buffer, "rec 0") == 0) {
                    ui_set_switch_state(ui->sw_record, false);
                    ui_sd_record_set_value(false);
                } else if (strcmp(rx_buffer, "free") == 0) {
                    sd_fat_info_t *fat_info = sd_get_fat_info();
                    size_t total_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
                    size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
                    const char *fmt = opt_format ? "*%u,%llu,%u,%u,%d\n"
                                                 : "{id=%u,fs=%llu,ram=%u,largest_block=%u,frag=%d}\n";
                    int len = sprintf(rx_buffer, fmt, E_SENSOR_FREE,
                        fat_info->bytes_free, total_free, largest_block, 100 - (largest_block * 100 / (total_free + 1)));
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                    ESP_LOGI(TAG, "%s", rx_buffer);
                    continue;
                } else if (strcmp(rx_buffer, "config") == 0) {
                    // Get config
                    int len = get_config(rx_buffer);
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                    continue;
                } else if (strcmp(rx_buffer, "lock") == 0) {
                    ui_config_lock(true);
                } else if (strcmp(rx_buffer, "unlock") == 0) {
                    ui_config_lock(true);
                } else if (strcmp(rx_buffer, "status 1") == 0) {
                    status.auto_status = true;
                } else if (strcmp(rx_buffer, "status 0") == 0) {
                    status.auto_status = false;
                } else if (strcmp(rx_buffer, "status") == 0) {
                    // Get status
                    int len = get_status(rx_buffer);
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                    continue;
                } else if (strncmp(rx_buffer, "pwr ", 4) == 0) {
                    // Set power status
                    if (strncmp(&rx_buffer[4], "lcd ", 4) == 0) {
                        config->lcd_pwr = rx_buffer[8] - '0';
                        ESP_ERROR_CHECK_WITHOUT_ABORT(ui_lcd_set_pwr_mode(config->lcd_pwr));
                    } else if (strncmp(&rx_buffer[4], "gps ", 4) == 0) {
                        config->gps_pwr = rx_buffer[8] - '0';
                        ESP_ERROR_CHECK_WITHOUT_ABORT(ui_gps_set_pwr_mode(config->gps_pwr));
                    } else if (strncmp(&rx_buffer[4], "scd ", 4) == 0) {
                        config->scd4x_pwr = rx_buffer[8] - '0';
                        ESP_ERROR_CHECK_WITHOUT_ABORT(ui_scd4x_set_pwr_mode(config->scd4x_pwr));
                    } else if (strncmp(&rx_buffer[4], "wifi ", 5) == 0) {
                        config->wifi_pwr = rx_buffer[9] - '0';
                        ESP_ERROR_CHECK_WITHOUT_ABORT(ui_wifi_set_pwr_mode(config->wifi_pwr));
                    } else if (strncmp(&rx_buffer[4], "mode ", 5) == 0) {
                        config->mode_pwr = rx_buffer[9] - '0';
                        ESP_ERROR_CHECK_WITHOUT_ABORT(ui_mode_set_pwr_mode(config->mode_pwr));
                    } else {
                        ESP_LOGE(TAG, "Unknown command <%s>", rx_buffer);
                        response = "ERR\n";
                    }
                } else if (strncmp(rx_buffer, "reset ", 6) == 0) {
                    // Set power status
                    if (strncmp(&rx_buffer[6], "gps ", 4) == 0) {
                        if (strcmp(&rx_buffer[10], "soft") == 0) {
                            if (gps_soft_reset(gps) == -1) {
                                ESP_LOGE(TAG, "GPS sensor soft reset failed");
                                response = "ERR\n";
                            }
                        } else if (strcmp(&rx_buffer[10], "partial") == 0) {
                            if (gps_partial_reset(gps) == -1) {
                                ESP_LOGE(TAG, "GPS sensor partial reset failed");
                                response = "ERR\n";
                            }
                        } else if (strcmp(&rx_buffer[10], "full") == 0) {
                            if (gps_full_reset(gps) == -1) {
                                ESP_LOGE(TAG, "GPS sensor full reset failed");
                                response = "ERR\n";
                            }
                        } else {
                            ESP_LOGE(TAG, "Unknown command <%s>", rx_buffer);
                            response = "ERR\n";
                        }
                    } else {
                        ESP_LOGE(TAG, "Unknown command <%s>", rx_buffer);
                        response = "ERR\n";
                    }
                } else if (strcmp(rx_buffer, "gps") == 0) {
                    int len = get_gps_values(rx_buffer);
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                } else if (strcmp(rx_buffer, "bme280lo") == 0) {
                    int len = get_bme280_values(rx_buffer, &bmx280lo->values);
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                } else if (strcmp(rx_buffer, "bme280hi") == 0) {
                    int len = get_bme280_values(rx_buffer, &bmx280hi->values);
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                } else if (strcmp(rx_buffer, "mhz19") == 0) {
                    int len = get_mhz19_values(rx_buffer, &mhz19->values);
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                } else if (strcmp(rx_buffer, "scd4x") == 0) {
                    int len = get_scd4x_values(rx_buffer, &scd4x->values);
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                } else if (strcmp(rx_buffer, "yys") == 0) {
                    int len = get_yys_values(rx_buffer);
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                } else if (strcmp(rx_buffer, "sps30") == 0) {
                    int len = get_sps30_values(rx_buffer, &sps30->values);
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                } else if (strncmp(rx_buffer, "scd4x ", 6) == 0) {
                    // Set power status
                    if (strncmp(&rx_buffer[6], "cal ", 4) == 0) {
                        scd4x_state_machine_cmd(SCD4X_CMD_FRC, atoi(&rx_buffer[10]));
                    } else if (strcmp(&rx_buffer[6], "cal") == 0) {
                        scd4x_state_machine_cmd(SCD4X_CMD_FRC, mhz19->values.co2);
                    } else if (strncmp(&rx_buffer[6], "autoadj ", 8) == 0) {
                        config->scd4x_auto_adjust = atoi(&rx_buffer[14]) != 0;
                    } else if (strcmp(&rx_buffer[6], "status") == 0) {
                        esp_err_t err = scd4x_stop_periodic_measurement(scd4x);
                        if (err == ESP_OK) {
                            scd4x->temperature_offset = scd4x_get_temperature_offset(scd4x);
                            scd4x->altitude = scd4x_get_sensor_altitude(scd4x);
                            err = scd4x_start_periodic_measurement(scd4x);
                            if (err != ESP_OK) {
                                ESP_LOGE(TAG, "Failed start periodic measurement: %u", err);
                                response = "ERR\n";
                            }
                        } else {
                            ESP_LOGE(TAG, "Failed to stop periodic measurement: %u", err);
                            response = "ERR\n";
                        }
                        const char *fmt = opt_format ? "*%u,%f,%u,%u\n"
                                                     : "{id=%u,temp_offs=%f,altitude=%u,pressure=%u}\n";
                        int len = sprintf(rx_buffer, fmt, E_SENSOR_SCD4XCAL,
                            scd4x->temperature_offset, scd4x->altitude, scd4x->pressure);
                        send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                    } else if (strcmp(&rx_buffer[4], "val 1") == 0) {
                        status.scd4x_auto_values = true;
                    } else if (strcmp(&rx_buffer[4], "val 0") == 0) {
                        status.scd4x_auto_values = false;
                    } else if (strcmp(&rx_buffer[4], "val") == 0) {
                        scd4x_values_t *values = &scd4x->values;
                        const char *fmt = opt_format ? "*%u,%u,%f,%f,%u\n"
                                                     : "{id=%u,co2=%u,temp=%f,hum=%f,st=%u}\n";
                        int len = sprintf(rx_buffer, fmt, E_SENSOR_SCD4X,
                            values->co2, values->temperature, values->humidity, scd4x_st_machine_status);
                        send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                    } else if (strncmp(&rx_buffer[4], "temp_offs ", 10) == 0) {
                        scd4x_set_temperature_offset(scd4x, atof(&rx_buffer[14]));
                    } else if (strncmp(&rx_buffer[4], "altitude ", 9) == 0) {
                        scd4x_set_sensor_altitude(scd4x, atof(&rx_buffer[13]));
                    } else if (strncmp(&rx_buffer[4], "pressure ", 9) == 0) {
                        scd4x_set_ambient_pressure(scd4x, atof(&rx_buffer[13]));
                    } else {
                        ESP_LOGE(TAG, "Unknown command <%s>", rx_buffer);
                        response = "ERR\n";
                    }
                } else if (strncmp(rx_buffer, "auto ", 5) == 0) {
                    // Set auto configs
                    if (strncmp(&rx_buffer[5], "con ", 4) == 0) {
                        config->wifi_auto_connect_idx = rx_buffer[9] - '0';
                    } else if (strncmp(&rx_buffer[5], "rec ", 4) == 0) {
                        config->auto_record = rx_buffer[5] - '0';
                    } else if (strncmp(&rx_buffer[5], "tcp ", 4) == 0) {
                        config->tcp_auto_start = rx_buffer[5] - '0';
                    } else if (strncmp(&rx_buffer[5], "ftp ", 4) == 0) {
                        config->ftp_auto_start = rx_buffer[5] - '0';
                    } else {
                        ESP_LOGE(TAG, "Unknown command <%s>", rx_buffer);
                        response = "ERR\n";
                    }
                } else if (strcmp(rx_buffer, "save") == 0) {
                    // Save config
                    if ((err = config_write()) != ESP_OK) {
                        ESP_LOGE(TAG, "Writing config failed err=%u", err);
                        response = "ERR\n";
                    }
                } else if (strncmp(rx_buffer, "tab ", 4) == 0) {
                    // Set current tab
                    uint32_t tab = rx_buffer[4] - '0';
                    if (tab < 6) {
                        ui_set_current_tab(tab);
                    } else {
                        ESP_LOGE(TAG, "Unknown command <%s>", rx_buffer);
                        response = "ERR\n";
                    }
                } else if (strcmp(rx_buffer, "rtc") == 0) {
                    // Get RTC date and time
                    struct tm timeinfo;
                    rtc_get_datetime(rtc->rtc, &timeinfo);
                    int len = sprintf(rx_buffer, "%u.%02d.%02d %02d:%02d:%02d",
                        1900 + timeinfo.tm_year, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                        timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                } else if (strncmp(rx_buffer, "rtc ", 4) == 0) {
                    // Set RTC date and time
                    if (strlen(rx_buffer) != 21 || rx_buffer[12] != ' ') {
                        ESP_LOGE(TAG, "Invalid argument");
                        response = "ERR\n";
                    } else {
                        struct tm timeinfo;

                        timeinfo.tm_year = atoi(&rx_buffer[4]) - 1900;
                        timeinfo.tm_mon = atoi(&rx_buffer[7]) - 1;
                        timeinfo.tm_mday = atoi(&rx_buffer[10]);
                        timeinfo.tm_hour = atoi(&rx_buffer[13]);
                        timeinfo.tm_min = atoi(&rx_buffer[16]);
                        timeinfo.tm_sec = atoi(&rx_buffer[19]);
                        if ((err = set_sys_time(&timeinfo, true)) != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to set system date/time: err=%u", err);
                            response = "ERR\n";
                        }
                    }
                } else {
                    ESP_LOGE(TAG, "Unknown command <%s>", rx_buffer);
                    response = "ERR\n";
                }
                send_data_to_client(client_sock, (uint8_t *)response, strlen(response));
            }

            msg_t tx_data;
            if (xQueueReceive(tx_queue, &tx_data, pdMS_TO_TICKS(10))) {
                send_data_to_client(client_sock, tx_data.data, tx_data.len);
                vPortFree(tx_data.data);
            }
        }
        tcp_send_values = false;
        tcp_client_cnt--;

        // Close client socket
        shutdown(client_sock, 0);
        close(client_sock);
    }
}

esp_err_t tcp_server_start()
{
    if (!wifi_connected) return ESP_FAIL;
    tcp_server_stop();
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, &tcp_server_task_handle);
    tx_queue = xQueueCreate(10, sizeof(msg_t));
    return ESP_OK;
}

esp_err_t tcp_server_stop()
{
    if (tcp_server_task_handle != NULL) {
        vTaskDelete(tcp_server_task_handle);
        tcp_server_task_handle = NULL;
    }
    if (tx_queue != NULL) {
        vQueueDelete(tx_queue);
        tx_queue = NULL;
    }
    tcp_send_values = false;
    tcp_server_running = false;
    return ESP_OK;
}

void tcp_server_publish_values()
{
    static char buf[256];

    if (tcp_server_task_handle == NULL) return;
    if (!wifi_connected || !tcp_server_running || tcp_client_cnt == 0) return;
    if (uxQueueMessagesWaiting(tx_queue) == 10) {
        ESP_LOGW(TAG, "TX queue full");
        return;
    }
    if (!tcp_send_values && !status.scd4x_auto_values) return;
    if (!force_update_all && !gps_update && !bmx280lo_update && !bmx280hi_update && !mhz19_update
        && !scd4x_calibrate && !scd4x_update && !yys_update && !sps30_update && !adxl345_update
        && !qmc5883l_update && !status.scd4x_auto_values) {
        return;
    }
    if (update_all_cnt > 0) {
        if (--update_all_cnt == 0) {
            force_update_all = false;
        }
    }
 
    int len = get_date_time(buf);
    ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    if (force_update_all || status.auto_status) {
        len = get_status(buf);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (force_update_all || (tcp_send_values && gps_update)) {
        len = get_gps_values(buf);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (force_update_all || (tcp_send_values && bmx280lo_update)) {
        len = get_bme280_values(buf, &bmx280lo->values);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (force_update_all || (tcp_send_values && bmx280hi_update)) {
        len = get_bme280_values(buf, &bmx280hi->values);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (force_update_all || (tcp_send_values && mhz19_update)) {
        len = get_mhz19_values(buf, &mhz19->values);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (force_update_all || (scd4x_update || status.scd4x_auto_values)) {
        len = get_scd4x_values(buf, &scd4x->values);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (force_update_all || (tcp_send_values && yys_update)) {
        len = get_yys_values(buf);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (force_update_all || (tcp_send_values && sps30_update)) {
        len = get_sps30_values(buf, &sps30->values);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (force_update_all || (tcp_send_values && adxl345_update)) {
        len = get_adxl345_values(buf, &adxl345->values);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (force_update_all || (tcp_send_values && qmc5883l_update)) {
        len = get_qmc5883l_values(buf, &qmc5883l->values);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
}
