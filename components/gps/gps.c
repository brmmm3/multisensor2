#include <string.h>
#include "hal/uart_types.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "gps.h"

static const char *GPS_TAG = "GPS";

// UART
#define UART_BUFFER_SIZE 256

TaskHandle_t uartRxGPSHandle = NULL;

hw_serial_t gps_serial = {
    .uart_num = UART_NUM_1,
    .rx_pin = 10,
    .baudrate = 9600,
    .queue = NULL
};

gps_sensor_t gps_sensor = {
    .name = "GPS",
    .buffer = NULL,
    .cnt = 0,
    .queue = NULL,
    .serial = &gps_serial,
    .messages = NULL,
    .msg_size = 0
};


uint8_t nmea_get_checksum(const char *buf)
{
    const char *n = buf + 1; // Plus one, skip '$'
    uint8_t chk = 0;

    /* While current char isn't '*' or buf ending (newline) */
    while ('*' != *n && NMEA_END_CHAR_1 != *n) {
        if ('\0' == *n || n - buf > NMEA_MAX_LENGTH) {
            /* Sentence too long or short */
            return 0;
        }
        chk ^= (uint8_t) *n;
        n++;
    }
    return chk;
}

char *gps_parse_char(char *p, char *unit)
{
    *unit = *p++;
    if (*unit != ',') p++; else *unit = ' ';
    return p;
}

char *gps_parse_uint8(char *p, uint8_t *v)
{
    *v = (uint8_t)strtol(p, &p, 10);
    return p + 1;
}

char *gps_parse_uint16(char *p, uint16_t *v)
{
    *v = (uint16_t)strtol(p, &p, 10);
    return p + 1;
}

char *gps_parse_float(char *p, float *v)
{
    *v = strtof(p, &p);
    return p + 1;
}

char *gps_parse_float_unit(char *p, float *value, char *unit)
{
    *value = strtof(p, &p);
    p += 1;
    *unit = *p++;
    if (*unit != ',') p++; else *unit = ' ';
    return p;
}

char *gps_parse_lat(char *p, float *value, char *unit)
{
    // Format: ddmm.mmmmm
    if (p[0] == ',') {
        *value = 0.0;
        *unit = ' ';
        return p + 1;
    }
    *value = (float)(p[0] - '0') * 10.0 + (float)(p[1] - '0');
    *value += strtof(&p[2], &p) / 60.0;  // Add minutes
    p += 1;
    *unit = *p++;
    p++;
    return p;
}

char *gps_parse_lng(char *p, float *value, char *unit)
{
    // Format: dddmm.mmmmm
    if (p[0] == ',') {
        *value = 0.0;
        *unit = ' ';
        return p + 1;
    }
    *value = (float)(p[0] - '0') * 100.0 + (float)(p[1] - '0') * 10.0 + (float)(p[2] - '0');
    *value += strtof(&p[3], &p) / 60.0;  // Add minutes
    p += 1;
    *unit = *p++;
    p++;
    return p;
}

// This message transfers various infos on the receiver, such as pwr-up screen, sw version etc.
// $GPTXT,01,01,02,ANTSTATUS=OK*3B
void gps_cmd_txt(gps_sensor_t *sensor, char *p, char *end)
{
    char *n = NULL;
    uint8_t msg_cnt = strtol(p, &p, 10);
    uint8_t msg_size = end - p;

    if (sensor->msg_size < msg_size) {
        if (sensor->messages != NULL) {
            free(sensor->messages);
        }
        sensor->messages = malloc(msg_size);
        sensor->msg_size = msg_size;
    }

    char *msg = sensor->messages;

    while (msg_cnt-- > 0) {
        uint8_t id = strtol(p + 4, &p, 10);

        if (msg_cnt > 0) {
            n = strchr(p + 1, ',');
        } else {
            n = strchr(p + 1, '*');
        }
        if (n == NULL || id > 7) break;
        switch (id) {
            case 0: *msg++ = 'E'; break;
            case 1: *msg++ = 'W'; break;
            case 2: *msg++ = 'N'; break;
            case 7: *msg++ = 'U'; break;
            default: *msg++ = '?';
        }
        strncpy(msg, p + 1, n - p - 1);
        msg += n - p - 1;
        *msg++ = '\n';
        p = n + 1;
    }
    *msg = 0;
    ESP_LOGI(sensor->name, "#TXT %d %d MSG=%s", msg_size, sensor->msg_size, sensor->messages);
}

// RMC-Recommended Minimum Specific GNSS Data
// $GPRMC,161229.487,A,3723.24756,N,12158.34162,W,0.13,309.62,120598,,*10
// $GPRMC,225637.00 ,V,          , ,           , ,    ,      ,271224,,,N*7A
// $GPRMC,225637.00,V,,,,,,,271224,,,N*7A
void gps_cmd_rmc(gps_sensor_t *sensor, char *p, char *end)
{
    gps_rmc_t *rmc = &sensor->rmc;

    p = gps_parse_float_unit(p, &rmc->utc_pos, &rmc->status);
    p = gps_parse_lat(p, &rmc->lat, &rmc->ns);
    p = gps_parse_lng(p, &rmc->lng, &rmc->ew);
    p = gps_parse_float(p, &rmc->speed);
    p = gps_parse_float(p, &rmc->course);
    rmc->date = strtol(p, &p, 10);
    p += 1;
    p = gps_parse_float_unit(p, &rmc->mv, &rmc->mv_ew);
    ESP_LOGI(sensor->name, "#RMC utc_pos=%f status=%c lat=%f %c lng=%f %c speed=%f course=%f date=%lu mv=%f %c",
             rmc->utc_pos, rmc->status, rmc->lat, rmc->ns, rmc->lng, rmc->ew, rmc->speed, rmc->course, rmc->date, rmc->mv, rmc->mv_ew);
}

// GLL-Geographic Position-Latitude/Longitude
// $GPGLL, 3723.24756, N, 12158.34162, W, 161229.487, A*2C
void gps_cmd_gll(gps_sensor_t *sensor, char *p, char *end)
{
    gps_gll_t *gll = &sensor->gll;

    p = gps_parse_lat(p, &gll->lat, &gll->ns);
    p = gps_parse_lng(p, &gll->lng, &gll->ew);
    p = gps_parse_float_unit(p, &gll->utc_pos, &gll->status);
    ESP_LOGI(sensor->name, "#GLL lat=%f %c lng=%f %c utc_pos=%f status=%c",
             gll->lat, gll->ns, gll->lng, gll->ew, gll->utc_pos, gll->status);
}

// GSA-GNSS DOP and Active Satallites
// $GPGSA, A, 3, 07, 02, 26, 27, 09, 04, 15, , , , , , 1.8,1.0,1.5*33
void gps_cmd_gsa(gps_sensor_t *sensor, char *p, char *end)
{
    gps_gsa_t *gsa = &sensor->gsa;
    uint8_t *sats = gsa->sats;

    p = gps_parse_char(p, &gsa->mode_auto);
    p = gps_parse_char(p, &gsa->mode_3d);
    for (int i = 0; i < 12; i++) {
        p = gps_parse_uint8(p, &sats[i]);
    }
    p = gps_parse_float(p, &gsa->pdop);
    p = gps_parse_float(p, &gsa->hdop);
    p = gps_parse_float(p, &gsa->vdop);
    ESP_LOGI(sensor->name, "#GSA mode_auto=%c mode_3d=%c sats=%d %d %d %d %d %d %d %d %d %d %d %d pdop=%f hdop=%f vdop=%f",
             gsa->mode_auto, gsa->mode_3d,
             sats[0], sats[1], sats[2], sats[3], sats[4], sats[5], sats[6], sats[7], sats[8], sats[9], sats[10], sats[11],
             gsa->pdop, gsa->hdop, gsa->vdop);
}

// GSV-GNSS Satallites in View
// $GPGSV, 2, 1, 07, 07, 79, 048, 42, 02, 51, 062, 43, 26, 36, 256, 42, 27, 27,138, 42*71
// $GPGSV, 2, 2, 07, 09, 23, 313, 42, 04, 19, 159, 41, 15, 12, 041, 42*41
void gps_cmd_gsv(gps_sensor_t *sensor, char *p, char *end)
{
    uint8_t msg_cnt;
    uint8_t msg_num;

    p = gps_parse_uint8(p, &msg_cnt);  // 1-9
    p = gps_parse_uint8(p, &msg_num);  // 1-msg_cnt
    p = gps_parse_uint8(p, &sensor->gsv.num_sv);

    gps_gsv_sat_t *sat = &sensor->gsv.sats[msg_num - 1];

    p = gps_parse_uint8(p, &sat->svid);
    p = gps_parse_uint8(p, &sat->elv);
    p = gps_parse_uint16(p, &sat->az);
    p = gps_parse_uint8(p, &sat->snr);
    ESP_LOGI(sensor->name, "#GSV msg_cnt=%d msg_num=%d num_sv=%d svid=%d elv=%d az=%d snr=%d",
             msg_cnt, msg_num, sensor->gsv.num_sv, sat->svid, sat->elv, sat->az, sat->snr);
}

// GGA-Global Postioning System Fixed Data
// $GPGGA, 161229.487, 3723.24756, N, 12158.34162, W, 1, 07, 1.0, 9.0, M, , , ,0000*18
void gps_cmd_gga(gps_sensor_t *sensor, char *p, char *end)
{
    gps_gga_t *gga = &sensor->gga;

    p = gps_parse_float(p, &gga->utc_pos);
    p = gps_parse_lat(p, &gga->lat, &gga->ns);
    p = gps_parse_lng(p, &gga->lng, &gga->ew);
    p = gps_parse_uint8(p, &gga->pos_fix);
    p = gps_parse_uint8(p, &gga->sat_used);
    p = gps_parse_float(p, &gga->hdop);
    p = gps_parse_float_unit(p, &gga->altitude, &gga->alt_unit);
    p = gps_parse_float_unit(p, &gga->goid, &gga->goid_unit);
    p = gps_parse_float(p, &gga->age);
    gga->diff_stat_id = strtol(p, &p, 10);
    ESP_LOGI(sensor->name, "#GGA utc_pos=%f lat=%f %c lng=%f %c pos_fix=%d sat_used=%d hdop=%f alt=%f %c goid=%f %c age=%f diff_stat_id=%d",
             gga->utc_pos, gga->lat, gga->ns, gga->lng, gga->ew, gga->pos_fix, gga->sat_used, gga->hdop, gga->altitude, gga->alt_unit,
             gga->goid, gga->goid_unit, gga->age, gga->diff_stat_id);
}

// VTG-Course Over Ground and Ground Speed
// $GPVTG,360.0,T,348.7,M,000.0,N,000.0,K*43
void gps_cmd_vtg(gps_sensor_t *sensor, char *p, char *end)
{
    gps_vtg_t *vtg = &sensor->vtg;

    p = gps_parse_float_unit(p, &vtg->true_track, &vtg->true_unit);
    p = gps_parse_float_unit(p, &vtg->magnetic_track, &vtg->magnetic_unit);
    p = gps_parse_float_unit(p, &vtg->speed_knots, &vtg->knot_unit);
    p = gps_parse_float_unit(p, &vtg->speed_kmh, &vtg->kmh_unit);
    ESP_LOGI(sensor->name, "#VTG track=%f %c mag=%f %c knots=%f %c kmh=%f %c",
             vtg->true_track, vtg->true_unit, vtg->magnetic_track, vtg->magnetic_unit, vtg->speed_knots, vtg->knot_unit, vtg->speed_kmh, vtg->kmh_unit);
}

// Date & Time
// $GPZDA,hhmmss.ss,xx,xx,xxxx,xx,xx*56
void gps_cmd_zda(gps_sensor_t *sensor, char *p, char *end)
{
    gps_zda_t *zda = &sensor->zda;

    p = gps_parse_float(p, &zda->utc_pos);
    p = gps_parse_uint8(p, &zda->day);
    p = gps_parse_uint8(p, &zda->month);
    p = gps_parse_uint8(p, &zda->year);
    zda->zone_hours = strtol(p, &p, 10);
    p += 1;
    zda->zone_minutes = strtol(p, &p, 10);
    p += 1;
    ESP_LOGI(sensor->name, "#ZDA utc_pos=%f date=%04d.%02d.%02d zone: h=%d m=%d",
             zda->utc_pos, zda->year, zda->month, zda->day, zda->zone_hours, zda->zone_minutes);
}

void rx_task_gps_sensor(void *arg)
{
    gps_sensor_t *sensor = (gps_sensor_t *)arg;

    hw_serial_t *serial = sensor->serial;
    char *buf = (char *)sensor->buffer;
    uint8_t offset = 0;

    while (1) {
        const int rxBytes = uart_read_bytes(serial->uart_num, &buf[offset], UART_BUFFER_SIZE - offset, 100);
        if (rxBytes == 0) continue;
        if (rxBytes < 0) break; // UART error
        //ESP_LOG_BUFFER_CHAR(sensor->name, buf, rxBytes);
        //ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, rxBytes, ESP_LOG_INFO);

        char *p = buf;

        if (*p != '$') {
            p = strchr(p, '$');
            if (p == NULL) {
                ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, rxBytes, ESP_LOG_ERROR);
                continue;
            }
        }

        char *end = strchr(p + 1, '*');
        char *p2 = strchr(p + 1, '$');

        if (p2 != NULL && p2 < end) {
            // Skip incomplete messages
            p = p2;
        }
        while (end != NULL) {
            char *sat;
            if (strncmp(p, "$G", 2) == 0) {
                if (p[2] == 'P') {
                    sat = "GPS";
                } else if (p[2] == 'N') {
                    sat = "GPS+GLN";
                } else if (p[2] == 'L') {
                    sat = "GLN";
                } else {
                    break;
                }
            } else if (strncmp(p, "$BD", 3) == 0) {
                sat = "BDU";
            } else {
                ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, rxBytes, ESP_LOG_ERROR);
                p = strchr(end + 1, '$');
                if (p != NULL) {
                    end = strchr(p + 1, '*');
                    p2 = strchr(p + 1, '$');
                    if (p2 != NULL && p2 < end) {
                        // Skip incomplete messages
                        p = p2;
                    }
                    continue;
                }
                p = &buf[rxBytes];
                break;
            }
            ESP_LOGI(sensor->name, "#SAT=%s", sat);
            p += 3;
            if (strncmp(p, "TXT,", 4) == 0) {
                gps_cmd_txt(sensor, p + 4, end);
            } else if (strncmp(p, "RMC,", 4) == 0) {
                gps_cmd_rmc(sensor, p + 4, end);
            } else if (strncmp(p, "GLL,", 4) == 0) {
                gps_cmd_gll(sensor, p + 4, end);
            } else if (strncmp(p, "GSA,", 4) == 0) {
                gps_cmd_gsa(sensor, p + 4, end);
            } else if (strncmp(p, "GSV,", 4) == 0) {
                gps_cmd_gsv(sensor, p + 4, end);
            } else if (strncmp(p, "GGA,", 4) == 0) {
                gps_cmd_gga(sensor, p + 4, end);
            } else if (strncmp(p, "VTG,", 4) == 0) {
                gps_cmd_vtg(sensor, p + 4, end);
            } else if (strncmp(p, "ZDA,", 4) == 0) {
                gps_cmd_zda(sensor, p + 4, end);
            } else {
                ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, rxBytes, ESP_LOG_INFO);
                ESP_LOGE(sensor->name, "#UNK=%s", p);
                p = strchr(end + 1, '$');
                if (p != NULL) {
                    end = strchr(p + 1, '*');
                    p2 = strchr(p + 1, '$');
                    if (p2 != NULL && p2 < end) {
                        // Skip incomplete messages
                        p = p2;
                    }
                    continue;
                }
                p = &buf[rxBytes];
                break;
            }
            uint8_t cksum = strtol(end + 1, &p, 16);
            p = end + 5;
            //ESP_LOGI(sensor->name, "#CKSUM=%x", cksum);
            end = strchr(p + 1, '*');
        }

        uint16_t cnt = p - buf;

        if (cnt < rxBytes) {
            ESP_LOGW(sensor->name, "#OFF=%d %s", cnt, p);
            offset = rxBytes - (uint8_t)cnt;
            strncpy(buf, p, offset);
        }
    }
    ESP_LOGE(GPS_TAG, "rx_task_gps_sensor EXIT");
}

gps_sensor_t gps_init()
{
    ESP_LOGI(GPS_TAG, "Initialize GPS");
    gps_serial.queue = xQueueCreate(128, 1);
    gps_sensor.buffer = calloc(UART_BUFFER_SIZE + 1, 1);
    gps_sensor.queue = xQueueCreate(128, 1);
    memset(&gps_sensor.rmc, 0, sizeof(gps_rmc_t));
    memset(&gps_sensor.gll, 0, sizeof(gps_gll_t));
    memset(&gps_sensor.gsa, 0, sizeof(gps_gsa_t));
    memset(&gps_sensor.gsv, 0, sizeof(gps_gsv_t));
    memset(&gps_sensor.gga, 0, sizeof(gps_gga_t));
    memset(&gps_sensor.vtg, 0, sizeof(gps_vtg_t));
    memset(&gps_sensor.zda, 0, sizeof(gps_zda_t));

    uart_init(gps_serial.uart_num, gps_serial.rx_pin, -1);

    xTaskCreate(rx_task_gps_sensor, "rx_task_gps_sensor_GPS", 4096, (void *)&gps_sensor, configMAX_PRIORITIES - 1, &uartRxGPSHandle);
    ESP_LOGI(GPS_TAG, "GPS initialized");
    return gps_sensor;
}
