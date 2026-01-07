#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include "adxl345.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "mhz19.h"
#include "nvs_flash.h"
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

static TaskHandle_t tcp_server_task_handle = NULL;
static QueueHandle_t tx_queue = NULL;
bool tcp_server_running = false;
uint8_t tcp_client_cnt = 0;


static bool send_message(const char *buf, int len)
{
    msg_t *msg = pvPortMalloc(sizeof(msg_t));
    msg->data = pvPortMalloc(len);
    memcpy(msg->data, buf, len);
    msg->len = len;
    return xQueueSend(tx_queue, msg, pdMS_TO_TICKS(100)) != pdPASS;
}

static bool send_data_to_client(int client_sock, uint8_t *data, int to_write)
{
    ESP_LOGI(TAG, "SEND %d %s", to_write, data);
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

static void tcp_server_task(void *pvParameters)
{
    char rx_buffer[BUFFER_SIZE];
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

        // Convert IP to string
        inet_ntoa_r(client_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Client connected from %s", addr_str);

        // Set Non-blocking flag
        int flags = fcntl(client_sock, F_GETFL, 0);
        fcntl(client_sock, F_SETFL, flags | O_NONBLOCK);

        int len = sprintf(rx_buffer, "{id=%d,name=\"MultiSensor V2.0\"}", E_SENSOR_INFO);
        send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);

        len = sprintf(rx_buffer, "{id=%d,lat=\"%c\",lng=\"%c\",co2=\"ppm\",temp=\"°C\",hum=\"%%\",o2=\"%%\",co=\"ppm\",h2s=\"ppm\",ch4=\"ppm\",",
            E_SENSOR_UNITS,
            gps_values.ns, gps_values.ew);
        len += sprintf(&rx_buffer[len], "pm0_5=\"#/cm3\",typ_part_sz=\"um\",pm1_0=\"ug/cm3\",p1_0=\"#/cm3\"}");
        len += sprintf(&rx_buffer[len], "pm2_5=\"ug/cm3\",p2_5=\"#/cm3\",pm4_0=\"ug/cm3\",p4_0=\"#/cm3\",pm10_0=\"ug/cm3\",p10_0=\"#/cm3\",");
        len += sprintf(&rx_buffer[len], "adxl345=\"g\",qmc5883l=\"gauss\"}");
        send_data_to_client(client_sock, (uint8_t *)rx_buffer, len);

        // Echo loop for this client
        while (true) {
            len = recv(client_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
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
            if (len > 0) {
                rx_buffer[len] = 0;  // Null-terminate
                ESP_LOGI(TAG, "Received %d bytes from %s: %s", len, addr_str, rx_buffer);
            }

            msg_t *tx_data;
            if (xQueueReceive(tx_queue, &tx_data, pdMS_TO_TICKS(10))) {
                send_data_to_client(client_sock, tx_data->data, tx_data->len);
                vPortFree(tx_data->data);
                vPortFree(tx_data);
            }
        }
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
    tx_queue = xQueueCreate(10, sizeof(msg_t *));
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
    tcp_server_running = false;
    return ESP_OK;
}

void tcp_server_publish_values()
{
    if (tcp_server_task_handle == NULL) return;
    if (!gps_update && !bmx280lo_update && !bmx280hi_update && !mhz19_update && !scd4x_calibrate &&
        !scd4x_update && !yys_update && !sps30_update && !adxl345_update && !qmc5883l_update) {
        return;
    }
 
    static char buf[256];

    ESP_LOGI(TAG, "Publish %d", bmx280lo_update);
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    get_current_date_time(&year, &month, &day, &hour, &min, &sec);
    int len = sprintf(buf, "{id=%d,date=\"%d.%d.%d\",time=\"%d:%d:%d\"}",
        E_SENSOR_TIME,
        year, month, day, hour, min, sec);
    ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    if (gps_update) {
        int len = sprintf(buf, "{id=%d,sat=%s,date=%lu,time=%lu,lat=%f,lng=%f,alt=%f,spd=%f,mode_3d=\"%c\",sats=%d,status=%d,pdop=%f,hdop=%f,vdop=%f}",
                E_SENSOR_GPS,
                gps_values.sat, (unsigned long)gps_values.date, (unsigned long)gps_values.time, gps_values.lat, gps_values.lng,
                gps_values.altitude, gps_values.speed, gps_values.mode_3d, gps_values.sats,
                gps_values.status, gps_values.pdop, gps_values.hdop, gps_values.vdop);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (bmx280lo_update) {
        bmx280_values_t *values = &bmx280lo->values;
        int len = sprintf(buf, "{id=%d,temp=%f,hum=%f,press=%f,alt=%f}",
            E_SENSOR_BMX280_LO,
            values->temperature, values->humidity, values->pressure, values->altitude);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (bmx280hi_update) {
        bmx280_values_t *values = &bmx280hi->values;
        int len = sprintf(buf, "{id=%d,temp=%f,hum=%f,press=%f,alt=%f}",
            E_SENSOR_BMX280_HI,
            values->temperature, values->humidity, values->pressure, values->altitude);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (mhz19_update) {
        mhz19_values_t *values = &mhz19->values;
        int len = sprintf(buf, "{id=%d,co2=%d,temp=%d,status=%d}",
            E_SENSOR_MHZ19,
            values->co2, values->temp, values->status);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (scd4x_update) {
        scd4x_values_t *values = &scd4x->values;
        int len = sprintf(buf, "{id=%d,co2=%d,temp=%f,hum=%f}",
            E_SENSOR_SCD4X,
            values->co2, values->temperature, values->humidity);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (yys_update) {
        int len = sprintf(buf, "{id=%d,o2=%f,co=%f,,h2s=%f,ch4=%f}",
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
        len += sprintf(&buf[len], "pm10_0=%f,p10_0=%f}", values->mc_10p0, values->nc_10p0);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (adxl345_update) {
        adxl345_values_t *values = &adxl345->values;
        int len = sprintf(buf, "{id=%d,x=%f,y=%f,z=%f,abs=%f,offs=%f %f %f}",
            E_SENSOR_ADXL345,
            values->accel_x, values->accel_y, values->accel_z, values->accel_abs,
            values->accel_offset_x, values->accel_offset_y, values->accel_offset_z);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
    if (qmc5883l_update) {
        qmc5883l_values_t *values = &qmc5883l->values;
        int len = sprintf(buf, "{id=%d,status=%d,x=%f,y=%f,z=%f,range=%f}",
            E_SENSOR_QMC5883L,
            values->status, values->mag_x, values->mag_y, values->mag_z, values->range);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_message(buf, len));
    }
}
