#include <string.h>
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
    while (to_write > 0) {
        int ret = send(client_sock, data + written, to_write, 0);
        if (ret < 0) {
            ESP_LOGE(TAG, "SEND failed: errno %d", errno);
            return false;
        }
        to_write -= ret;
        written += ret;
    }
    return true;
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
        ESP_LOGE(TAG, "Failed to send file %s", path);
        err = ESP_FAIL;
    } else {
        while (true) {
            uint32_t len = read_data_file_part(f, rx_buffer, BUFFER_SIZE);
            if (len == 0) break;
            if ((err = send_data_file_part(client_sock, path, rx_buffer, buffer_size2)) != ESP_OK) break;
            if ((err = send_data_file_part(client_sock, path, &rx_buffer[buffer_size2], buffer_size2)) != ESP_OK) break;
        }
    }
    close_data_file(f);
    if (!send_data_to_client(client_sock, (uint8_t *)"\n",1)) {
        ESP_LOGE(TAG, "Failed to send file %s", path);
        err = ESP_FAIL;
    }
    return err;
}

static esp_err_t client_cmd_ls(int client_sock)
{
    DIR *dir = sd_open_data_dir();
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open data dir");
        return ESP_FAIL;
    }
    while (true) {
        int len = sd_read_dir(dir, rx_buffer, BUFFER_SIZE);
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
        int len = sd_read_dir(dir, rx_buffer, BUFFER_SIZE);
        if (len == 0) break;
        send_data_to_client(client_sock, (uint8_t *)rx_buffer,len);
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
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // Bind to port
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);

    int err = bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    // Listen
    err = listen(listen_sock, MAX_CLIENTS);
    if (err < 0) {
        ESP_LOGE(TAG, "Error listening: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", PORT);
    tcp_server_running = true;

    while (true) {
        // Accept new client
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &client_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
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

        int len = sprintf(rx_buffer, "{id=%d,name=\"MultiSensor V2.0\"}\n", E_SENSOR_INFO);
        send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);

        len = sprintf(rx_buffer, "{id=%d,lat=\"%c\",lng=\"%c\",co2=\"ppm\",temp=\"°C\",hum=\"%%\",o2=\"%%\",co=\"ppm\",h2s=\"ppm\",ch4=\"ppm\",",
            E_SENSOR_UNITS,
            gps_values.ns, gps_values.ew);
        len += sprintf(&rx_buffer[len], "pm0_5=\"#/cm3\",typ_part_sz=\"um\",pm1_0=\"ug/cm3\",p1_0=\"#/cm3\"}");
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
                    ESP_LOGE(TAG, "recv failed: errno %d", errno);
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

                if (strcmp(rx_buffer, "bye") == 0) break;
                if (strncmp(rx_buffer, "val ", 4) == 0) {
                    int value = atoi(&rx_buffer[4]);
                    if (value == 0) {
                        tcp_send_values = false;
                    } else {
                        tcp_send_values = true;
                        force_update_all = true;
                        if (value == 1) update_all_cnt = 2;
                    }
                } else if (strcmp(rx_buffer, "lsr") == 0) {
                    // Show root files on SD-Card
                    if ((err = client_cmd_lsr(client_sock)) != ESP_OK) {
                        response = "ERR\n";
                    }
                } else if (strcmp(rx_buffer, "ls") == 0) {
                    // Show data files on SD-Card
                    if ((err = client_cmd_ls(client_sock)) != ESP_OK) {
                        response = "ERR\n";
                    }
                } else if (strncmp(rx_buffer, "cp ", 3) == 0) {
                    // Download data file and keep it on SD-Card
                    char path[32];
                    strcpy(path, &rx_buffer[3]);
                    if ((err = send_data_file(client_sock, path)) != ESP_OK) {
                        response = "ERR\n";
                    }
                } else if (strncmp(rx_buffer, "mv ", 3) == 0) {
                    // Download data file and remove it on SD-Card
                    char path[32];
                    strcpy(path, &rx_buffer[3]);
                    if ((err = send_data_file(client_sock, path)) == ESP_OK) {
                        if ((err = remove_data_file(path)) != ESP_OK) {
                            response = "ERR\n";
                        }
                    } else {
                        ESP_LOGE(TAG, "Failed to send file");
                        response = "ERR\n";
                    }
                } else if (strncmp(rx_buffer, "rm ", 3) == 0) {
                    // Remove file on SD-Card
                    char path[32];
                    strcpy(path, &rx_buffer[3]);
                    if ((err = remove_data_file(path)) != ESP_OK) {
                        response = "ERR\n";
                    }
                } else if (strcmp(rx_buffer, "rec 1") == 0) {
                    ui_set_switch_state(ui->sw_record, true);
                    ui_sd_record_set_value(true);
                } else if (strcmp(rx_buffer, "rec 0") == 0) {
                    ui_set_switch_state(ui->sw_record, false);
                    ui_sd_record_set_value(false);
                } else if (strcmp(rx_buffer, "free") == 0) {
                    sd_fat_info_t *fat_info = sd_get_fat_info();
                    len = sprintf(rx_buffer, "fs: %" PRIu64 " MB\n", fat_info->bytes_free / (1024 * 1024));
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                    size_t total_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
                    size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
                    len = sprintf(rx_buffer, "heap: free=%u largest_block=%u frag=%d%%\n",
                        total_free, largest_block, 100 - (largest_block * 100 / (total_free + 1)));
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                    ESP_LOGI(TAG, "%s", rx_buffer);
                } else if (strcmp(rx_buffer, "config") == 0) {
                    // Get config
                    int len = sprintf(rx_buffer, "{id=%d,cfg_version=%d,auto_connect=%d,auto_record=%d,",
                        E_SENSOR_CONFIG,
                        config->cfg_version, config->auto_connect, config->auto_record);
                    len += sprintf(&rx_buffer[len], "lcd_pwr=%d,gps_pwr=%d,scd4x_pwr=%d,wifi_pwr=%d,mode_pwr=%d,",
                        config->lcd_pwr, config->gps_pwr, config->scd4x_pwr, config->wifi_pwr, config->mode_pwr);
                    len += sprintf(&rx_buffer[len], "tcp_auto_start=%d,ftp_auto_start=%d", config->tcp_auto_start, config->ftp_auto_start);
                    for (int i = 0; i < 4; i++) {
                        if (strlen(config_nvs->wifi.ssid[i]) > 0) {
                            len += sprintf(&rx_buffer[len], ",ssid%d=%s", i, config_nvs->wifi.ssid[i]);
                        }
                    }
                    len += sprintf(&rx_buffer[len], "}\n");
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
                } else if (strcmp(rx_buffer, "status") == 0) {
                    // Get status
                    int len = sprintf(rx_buffer, "{id=%d,force_update=%d,recording=%d,record_pos=%d,file_cnt=%d,filename=\"%s\"}\n",
                        E_SENSOR_STATUS,
                        status.force_update, status.recording, status.record_pos, status.file_cnt, status.filename);
                    send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);
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
                } else if (strncmp(rx_buffer, "auto ", 5) == 0) {
                    // Set auto configs
                    if (strncmp(&rx_buffer[5], "con ", 4) == 0) {
                        config->auto_connect = rx_buffer[9] - '0';
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
                        ESP_LOGE(TAG, "Writing config failed err=%d", err);
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
                    int len = sprintf(rx_buffer, "%d.%02d.%02d %02d:%02d:%02d",
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
                        if ((err = set_sys_time(&timeinfo)) != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to set system date/time: err=%d", err);
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

    if (!tcp_server_running && config->tcp_auto_start) {
        if (tcp_server_start() != ESP_OK) return;
    }
    if (tcp_server_task_handle == NULL) return;
    if (!wifi_connected || !tcp_server_running || tcp_client_cnt == 0) return;
    if (uxQueueMessagesWaiting(tx_queue) == 10) {
        ESP_LOGW(TAG, "TX queue full");
        return;
    }
    if (!tcp_send_values) return;
    if (!gps_update && !bmx280lo_update && !bmx280hi_update && !mhz19_update && !scd4x_calibrate &&
        !scd4x_update && !yys_update && !sps30_update && !adxl345_update && !qmc5883l_update) {
        return;
    }
    if (update_all_cnt > 0) {
        if (--update_all_cnt == 0) {
            force_update_all = false;
        }
    }
 
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    get_current_date_time(&year, &month, &day, &hour, &min, &sec);
    len = sprintf(buf, "{id=%d,date=\"%d.%d.%d\",time=\"%d:%d:%d\"}\n",
        E_SENSOR_TIME,
        year, month, day, hour, min, sec);
    ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    if (gps_update) {
        int len = sprintf(buf, "{id=%d,sat=%s,date=%lu,time=%lu,lat=%f,lng=%f,alt=%f,spd=%f,mode_3d=\"%c\",sats=%d,status=%d,pdop=%f,hdop=%f,vdop=%f}\n",
                E_SENSOR_GPS,
                gps_values.sat, (unsigned long)gps_values.date, (unsigned long)gps_values.time, gps_values.lat, gps_values.lng,
                gps_values.altitude, gps_values.speed, gps_values.mode_3d, gps_values.sats,
                gps_values.status, gps_values.pdop, gps_values.hdop, gps_values.vdop);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (bmx280lo_update) {
        bmx280_values_t *values = &bmx280lo->values;
        int len = sprintf(buf, "{id=%d,temp=%f,hum=%f,press=%f,alt=%f}\n",
            E_SENSOR_BMX280_LO,
            values->temperature, values->humidity, values->pressure, values->altitude);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (bmx280hi_update) {
        bmx280_values_t *values = &bmx280hi->values;
        int len = sprintf(buf, "{id=%d,temp=%f,hum=%f,press=%f,alt=%f}\n",
            E_SENSOR_BMX280_HI,
            values->temperature, values->humidity, values->pressure, values->altitude);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (mhz19_update) {
        mhz19_values_t *values = &mhz19->values;
        int len = sprintf(buf, "{id=%d,co2=%d,temp=%d,status=%d}\n",
            E_SENSOR_MHZ19,
            values->co2, values->temp, values->status);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (scd4x_update) {
        scd4x_values_t *values = &scd4x->values;
        int len = sprintf(buf, "{id=%d,co2=%d,temp=%f,hum=%f}\n",
            E_SENSOR_SCD4X,
            values->co2, values->temperature, values->humidity);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (yys_update) {
        int len = sprintf(buf, "{id=%d,o2=%f,co=%f,,h2s=%f,ch4=%f}\n",
            E_SENSOR_YYS,
            yys_get_o2(yys_sensor), yys_get_co(yys_sensor),
            yys_get_h2s(yys_sensor), yys_get_ch4(yys_sensor));
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (sps30_update) {
        sps30_values_t *values = &sps30->values;
        int len = sprintf(buf, "{id=%d,status=%lu,pm0_5=%f,typ_part_sz=%f",
            E_SENSOR_SPS30,
            (unsigned long)values->status, values->nc_0p5, values->typical_particle_size);
        len += sprintf(&buf[len], "pm1_0=%f,p1_0=%f,", values->mc_1p0, values->nc_1p0);
        len += sprintf(&buf[len], "pm2_5=%f,p2_5=%f,", values->mc_2p5, values->nc_2p5);
        len += sprintf(&buf[len], "pm4_0=%f,p4_0=%f,", values->mc_4p0, values->nc_4p0);
        len += sprintf(&buf[len], "pm10_0=%f,p10_0=%f}\n", values->mc_10p0, values->nc_10p0);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (adxl345_update) {
        adxl345_values_t *values = &adxl345->values;
        int len = sprintf(buf, "{id=%d,x=%f,y=%f,z=%f,abs=%f,offs=%f %f %f}\n",
            E_SENSOR_ADXL345,
            values->accel_x, values->accel_y, values->accel_z, values->accel_abs,
            values->accel_offset_x, values->accel_offset_y, values->accel_offset_z);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (qmc5883l_update) {
        qmc5883l_values_t *values = &qmc5883l->values;
        int len = sprintf(buf, "{id=%d,status=%d,x=%f,y=%f,z=%f,range=%f}\n",
            E_SENSOR_QMC5883L,
            values->status, values->mag_x, values->mag_y, values->mag_z, values->range);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
}
