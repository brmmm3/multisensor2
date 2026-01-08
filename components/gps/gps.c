#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"

#include "gps.h"

static const char *TAG = "GPS";

// UART
#define UART_BUFFER_SIZE 256

//Power save modes
const unsigned char ubxPSM[] = { 0x06,0x11,0x02,0x08,0x01 }; // Power Save Mode
const unsigned char ubxEM[] =  { 0x06,0x11,0x02,0x08,0x04 }; // Eco Mode
const unsigned char ubxMPM[] = { 0x06,0x11,0x02,0x08,0x00 }; // Max Performance Mode
const unsigned char ubxSleep[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B}; // Sleep mode


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
            vPortFree(sensor->messages);
        }
        sensor->messages = pvPortMalloc(msg_size);
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
    ESP_LOGI(TAG, "TXT msg_size=%d %d\nMSG=%s", msg_size, sensor->msg_size, sensor->messages);
}

// RMC-Recommended Minimum Specific GNSS Data
// $GPRMC,161229.487,A,3723.24756,N,12158.34162,W,0.13,309.62,120598,,*10
// $GPRMC,225637.00 ,V,          , ,           , ,    ,      ,271224,,,N*7A
// $GPRMC,225637.00,V,,,,,,,271224,,,N*7A
void gps_cmd_rmc(gps_sensor_t *sensor, char *p, char *end)
{
    gps_rmc_t *rmc = &sensor->rmc;

    p = gps_parse_float_unit(p, &rmc->time, &rmc->status);
    p = gps_parse_lat(p, &rmc->lat, &rmc->ns);
    p = gps_parse_lng(p, &rmc->lng, &rmc->ew);
    p = gps_parse_float(p, &rmc->speed);
    p = gps_parse_float(p, &rmc->course);
    rmc->date = strtol(p, &p, 10);
    p += 1;
    p = gps_parse_float_unit(p, &rmc->mv, &rmc->mv_ew);
    rmc->cnt++;
}

// GLL-Geographic Position-Latitude/Longitude
// $GPGLL, 3723.24756, N, 12158.34162, W, 161229.487, A*2C
void gps_cmd_gll(gps_sensor_t *sensor, char *p, char *end)
{
    gps_gll_t *gll = &sensor->gll;

    p = gps_parse_lat(p, &gll->lat, &gll->ns);
    p = gps_parse_lng(p, &gll->lng, &gll->ew);
    p = gps_parse_float_unit(p, &gll->time, &gll->status);
    gll->cnt++;
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
    gsa->cnt++;
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
    sensor->gsv.cnt++;
}

// GGA-Global Postioning System Fixed Data
// $GPGGA, 161229.487, 3723.24756, N, 12158.34162, W, 1, 07, 1.0, 9.0, M, , , ,0000*18
void gps_cmd_gga(gps_sensor_t *sensor, char *p, char *end)
{
    gps_gga_t *gga = &sensor->gga;

    p = gps_parse_float(p, &gga->time);
    p = gps_parse_lat(p, &gga->lat, &gga->ns);
    p = gps_parse_lng(p, &gga->lng, &gga->ew);
    p = gps_parse_uint8(p, &gga->pos_fix);
    p = gps_parse_uint8(p, &gga->sat_used);
    p = gps_parse_float(p, &gga->hdop);
    p = gps_parse_float_unit(p, &gga->altitude, &gga->alt_unit);
    p = gps_parse_float_unit(p, &gga->goid, &gga->goid_unit);
    p = gps_parse_float(p, &gga->age);
    gga->diff_stat_id = strtol(p, &p, 10);
    gga->cnt++;
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
    vtg->cnt++;
}

// ZDA Date & Time
// $GPZDA,hhmmss.ss,xx,xx,xxxx,xx,xx*56
void gps_cmd_zda(gps_sensor_t *sensor, char *p, char *end)
{
    gps_zda_t *zda = &sensor->zda;

    p = gps_parse_float(p, &zda->time);
    p = gps_parse_uint8(p, &zda->day);
    p = gps_parse_uint8(p, &zda->month);
    p = gps_parse_uint8(p, &zda->year);
    zda->zone_hours = strtol(p, &p, 10);
    p++;
    zda->zone_minutes = strtol(p, &p, 10);
    zda->cnt++;
}

void rx_task_gps_sensor(void *arg)
{
    gps_sensor_t *sensor = (gps_sensor_t *)arg;
    gps_rmc_t *rmc = &sensor->rmc;
    gps_gll_t *gll = &sensor->gll;
    gps_gsa_t *gsa = &sensor->gsa;
    gps_gga_t *gga = &sensor->gga;
    gps_vtg_t *vtg = &sensor->vtg;
    gps_zda_t *zda = &sensor->zda;
    gps_status_t *status = &sensor->status;
    hw_serial_t *serial = sensor->serial;
    char *buf = (char *)sensor->buffer;
    uint8_t offset = 0;

    while (true) {
        const int rxBytes = uart_read_bytes(serial->uart_num, &buf[offset], UART_BUFFER_SIZE - offset, 100);

        if (rxBytes == 0) continue;
        if (rxBytes < 0) {
            status->error_cnt++;
            continue; // UART error
        }
        status->status++;
        if (sensor->debug & 16) {
            ESP_LOG_BUFFER_CHAR(sensor->name, buf, rxBytes);
        }
        //ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, rxBytes, ESP_LOG_INFO);

        char *p = buf;

        if (*p != '$') {
            p = strchr(p, '$');
            if (p == NULL) {
                status->error_cnt++;
                if (sensor->debug & 8) {
                    ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, rxBytes, ESP_LOG_ERROR);
                }
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
            if (strncmp(p, "$G", 2) == 0) {
                if (p[2] == 'P') {
                    status->sat = "GPS";
                } else if (p[2] == 'N') {
                    status->sat = "GPS+GLN";
                } else if (p[2] == 'L') {
                    status->sat = "GLN";
                } else {
                    break;
                }
            } else if (strncmp(p, "$BD", 3) == 0) {
                status->sat = "BDU";
            } else {
                //ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, rxBytes, ESP_LOG_ERROR);
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
            //ESP_LOGI(sensor->name, "#SAT=%s", sat);
            p += 3;
            if (strncmp(p, "TXT,", 4) == 0) {
                gps_cmd_txt(sensor, p + 4, end);
            } else if (strncmp(p, "RMC,", 4) == 0) {
                gps_cmd_rmc(sensor, p + 4, end);
                if (rmc->status == 'A') {
                    if (rmc->date > 10125) {
                        status->date = rmc->date;
                    }
                    status->time = (uint32_t)rmc->time;
                    status->lat = rmc->lat;
                    status->ns = rmc->ns;
                    status->lng = rmc->lng;
                    status->ew = rmc->ew;
                    status->speed = rmc->speed * 1.852;
                    status->status = 0;
                }
            } else if (strncmp(p, "GLL,", 4) == 0) {
                gps_cmd_gll(sensor, p + 4, end);
                if (gll->status == 'A') {
                    status->time = (uint32_t)gll->time;
                    status->lat = gll->lat;
                    status->ns = gll->ns;
                    status->lng = gll->lng;
                    status->ew = gll->ew;
                    status->status = 0;
                }
            } else if (strncmp(p, "GSA,", 4) == 0) {
                gps_cmd_gsa(sensor, p + 4, end);
                for (int i = 0; i < 12; i++) {
                    if (gsa->sats[i] == 0) {
                        status->sats = i;
                        break;
                    } else if (i == 11) {
                        status->sats = 12;
                    }
                }
                status->mode_3d = gsa->mode_3d;
                status->status = 0;
            } else if (strncmp(p, "GSV,", 4) == 0) {
                gps_cmd_gsv(sensor, p + 4, end);
            } else if (strncmp(p, "GGA,", 4) == 0) {
                gps_cmd_gga(sensor, p + 4, end);
                if (gga->alt_unit == 'M') {
                    status->time = (uint32_t)gga->time;
                    status->lat = gga->lat;
                    status->ns = gga->ns;
                    status->lng = gga->lng;
                    status->ew = gga->ew;
                    status->sats = gga->sat_used;
                    status->altitude = gga->altitude;
                    status->status = 0;
                }
            } else if (strncmp(p, "VTG,", 4) == 0) {
                gps_cmd_vtg(sensor, p + 4, end);
                if (vtg->kmh_unit == 'K') {
                    status->speed = vtg->speed_kmh;
                    status->status = 0;
                }
            } else if (strncmp(p, "ZDA,", 4) == 0) {
                gps_cmd_zda(sensor, p + 4, end);
                if (zda->year > 24) {
                    status->date = (uint32_t)zda->year * 10000 + (uint32_t)zda->month * 100 + (uint32_t)zda->day;
                    status->time = (uint32_t)zda->time;
                    status->status = 0;
                }
            } else {
                //ESP_LOG_BUFFER_HEXDUMP(sensor->name, buf, rxBytes, ESP_LOG_INFO);
                //ESP_LOGE(sensor->name, "#UNK=%s", p);
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
            //uint8_t cksum = strtol(end + 1, &p, 16);
            p = end + 5;
            //ESP_LOGI(sensor->name, "#CKSUM=%x", cksum);
            end = strchr(p + 1, '*');
        }

        uint16_t cnt = p - buf;

        if (cnt < rxBytes) {
            //ESP_LOGW(sensor->name, "#OFF=%d %s", cnt, p);
            offset = rxBytes - (uint8_t)cnt;
            strncpy(buf, p, offset);
        }
    }
}

bool gps_data_ready(gps_sensor_t *sensor)
{
    return sensor->status.status == 0;
}

esp_err_t gps_init(gps_sensor_t **sensor_ptr, uint8_t uart_num, uint8_t rx_pin, uint8_t tx_pin)
{
    hw_serial_t *gps_serial = pvPortMalloc(sizeof(hw_serial_t));
    gps_sensor_t *sensor = pvPortMalloc(sizeof(gps_sensor_t));
    memset(sensor, 0, sizeof(gps_sensor_t));

    ESP_LOGI(TAG, "Initialize GPS");
   // Serial
    gps_serial->uart_num = uart_num;
    gps_serial->rx_pin = rx_pin;
    gps_serial->tx_pin = tx_pin;
    gps_serial->baudrate = 9600;
    gps_serial->queue = xQueueCreate(128, 1);
    // Sensor
    sensor->name = "GPS";
    sensor->buffer = pvPortMalloc(UART_BUFFER_SIZE + 1);
    memset(sensor->buffer, 0, UART_BUFFER_SIZE + 1);
    sensor-> cnt = 0;
    sensor->queue = xQueueCreate(128, 1);
    sensor->serial = gps_serial;
    sensor->messages = NULL;
    sensor->msg_size = 0;
    sensor->status.sat = "?";
    *sensor_ptr = sensor;

    uart_init(gps_serial->uart_num, gps_serial->rx_pin, gps_serial->tx_pin);

    xTaskCreate(rx_task_gps_sensor, "rx_task_gps_sensor", 4096, sensor, configMAX_PRIORITIES - 1, NULL);

    ESP_LOGI(TAG, "GPS initialized");
    return ESP_OK;
}

int gps_set_power_mode(gps_sensor_t *sensor, uint8_t mode)
{
    uint8_t a = 0, b = 0;
    char buf[sizeof(ubxPSM) + 4];

    ESP_LOGI(TAG, "gps_set_power_mode %d", mode);
    if (mode > 1) return gps_power_off(sensor);
    buf[0] = 0xB5;
    buf[1] = 0x62;
    if (mode == 1) memcpy(&buf[2], ubxEM, sizeof(ubxEM));
    else memcpy(&buf[2], ubxPSM, sizeof(ubxPSM));
    for (int i = 0; i < sizeof(ubxPSM); i++) {
        a += buf[i + 2];
        b += a;
    }
    buf[sizeof(ubxPSM) + 2] = a;
    buf[sizeof(ubxPSM) + 3] = b;
    return uart_write_bytes(sensor->serial->uart_num, buf, sizeof(buf));
}

int gps_power_off(gps_sensor_t *sensor)
{
    return uart_write_bytes(sensor->serial->uart_num, ubxSleep, sizeof(ubxSleep));
}

void gps_dump_values(gps_sensor_t *sensor, bool force)
{
    if (force || sensor->debug & 1) {
        gps_status_t *status = &sensor->status;

        ESP_LOGI(TAG, "%s date=%lu time=%lu lat=%f %c lng=%f %c altitude=%f speed=%f mode_3d=%c sats=%d status=%d errors=%d",
                 status->sat, status->date, status->time, status->lat, status->ns, status->lng, status->ew, status->altitude,
                 status->speed, status->mode_3d, status->sats, status->status, status->error_cnt);
    }
    if (force || sensor->debug & 2) {
        gps_rmc_t *rmc = &sensor->rmc;
        gps_gll_t *gll = &sensor->gll;
        gps_gsa_t *gsa = &sensor->gsa;
        uint8_t *gsa_sats = gsa->sats;
        gps_gsv_t *gsv = &sensor->gsv;
        gps_gga_t *gga = &sensor->gga;
        gps_vtg_t *vtg = &sensor->vtg;
        gps_zda_t *zda = &sensor->zda;

        // RMC - Recommended Minimum Navigation Information
        ESP_LOGI(TAG, "RMC: %d date=%lu time=%lu status=%c lat=%f %c lng=%f %c speed=%f course=%f mv=%f %c",
                rmc->cnt, rmc->date, (uint32_t)rmc->time, rmc->status, rmc->lat, rmc->ns, rmc->lng, rmc->ew, rmc->speed, rmc->course, rmc->mv, rmc->mv_ew);
        // GLL-Geographic Position-Latitude/Longitude
        ESP_LOGI(TAG, "GLL: %d lat=%f %c lng=%f %c time=%lu status=%c",
                gll->cnt, gll->lat, gll->ns, gll->lng, gll->ew, (uint32_t)gll->time, gll->status);
        // GSA-GNSS DOP and Active Satallites
        ESP_LOGI(TAG, "GSA: %d auto=%c 3d=%c sats=%d %d %d %d %d %d %d %d %d %d %d %d pdop=%f hdop=%f vdop=%f",
                gsa->cnt, gsa->mode_auto, gsa->mode_3d,
                gsa_sats[0], gsa_sats[1], gsa_sats[2], gsa_sats[3], gsa_sats[4], gsa_sats[5], gsa_sats[6], gsa_sats[7],
                gsa_sats[8], gsa_sats[9], gsa_sats[10], gsa_sats[11],
                gsa->pdop, gsa->hdop, gsa->vdop);
        // GSV-GNSS Satallites in View
        ESP_LOGI(TAG, "GSV: %d num_sv=%d", gsv->cnt, gsv->num_sv);
        for (int i = 0; i < 9; i++) {
            gps_gsv_sat_t *sat = &gsv->sats[i];

            if (sat->svid == 0) break;
            ESP_LOGI(TAG, "GSV-SAT%d: svid=%d elv=%d az=%d snr=%d",
                    i, sat->svid, sat->elv, sat->az, sat->snr);
        }
        // GGA-Global Postioning System Fixed Data
        ESP_LOGI(TAG, "GGA: %d time=%lu lat=%f %c lng=%f %c pos_fix=%d sat_used=%d hdop=%f alt=%f %c goid=%f %c age=%f diff_stat_id=%d",
                gga->cnt, (uint32_t)gga->time, gga->lat, gga->ns, gga->lng, gga->ew, gga->pos_fix, gga->sat_used, gga->hdop, gga->altitude,
                gga->alt_unit, gga->goid, gga->goid_unit, gga->age, gga->diff_stat_id);
        // VTG-Course Over Ground and Ground Speed
        ESP_LOGI(TAG, "VTG: %d track=%f %c mag=%f %c knots=%f %c kmh=%f %c",
                vtg->cnt, vtg->true_track, vtg->true_unit, vtg->magnetic_track, vtg->magnetic_unit, vtg->speed_knots, vtg->knot_unit,
                vtg->speed_kmh, vtg->kmh_unit);
        // ZDA Date & Time
        ESP_LOGI(TAG, "ZDA: %d time=%lu date=%02d.%02d.%02d zone: h=%d m=%d",
                zda->cnt, (uint32_t)zda->time, zda->year, zda->month, zda->day, zda->zone_hours, zda->zone_minutes);
    }
}
