#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "main.h"

#include "mqtt_client.h"

static const char *TAG = "MQTT";

const char *MQTT_BROKER = "mqtts://192.168.1.10";

#define MS2_TAG       "MS2"

#define MQTT_CONNECTED_BIT  BIT0

static esp_mqtt5_user_property_item_t user_property_arr[] = {
        {"board", "esp32"},
        {"u", "user"},
        {"p", "password"}
    };

#define USE_PROPERTY_ARR_SIZE   sizeof(user_property_arr)/sizeof(esp_mqtt5_user_property_item_t)

static EventGroupHandle_t mqtt_event_group;

static esp_mqtt_client_handle_t mqtt_client = NULL;

bool mqtt_connected = false;


static void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)event_data;
    if (event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT Connected!");
        xEventGroupSetBits(mqtt_event_group, MQTT_CONNECTED_BIT);

        // Subscribe to RemoteControl topic
        esp_mqtt_client_subscribe(client, MS2_TAG"/rc", 0);

        // Publish a message
        esp_mqtt_client_publish(client, MS2_TAG"/info", "Hello from ESP32-C6!", 0, 1, 0);
        ui_set_label_text(ui->lbl_mqtt_status, "Connected");
        mqtt_connected = true;
    } else if (event_id == MQTT_EVENT_DISCONNECTED) {
        mqtt_connected = false;
        ESP_LOGW(TAG, "MQTT Disconnected");
        xEventGroupClearBits(mqtt_event_group, MQTT_CONNECTED_BIT);
        ui_set_label_text(ui->lbl_mqtt_status, "Disonnected");
    } else if (event_id == MQTT_EVENT_DATA) {
        esp_mqtt_event_handle_t event = event_data;
        ESP_LOGI(TAG, "Received: topic=%.*s  data=%.*s",
                 event->topic_len, event->topic,
                 event->data_len, event->data);
    }
}

void mqtt_start()
{
    esp_mqtt5_connection_property_config_t connect_property = {
        .session_expiry_interval = 10,
        .maximum_packet_size = 1024,
        .receive_maximum = 65535,
        .topic_alias_maximum = 2,
        .request_resp_info = true,
        .request_problem_info = true,
        .will_delay_interval = 10,
        .payload_format_indicator = true,
        .message_expiry_interval = 10,
        .response_topic = MS2_TAG"/response",
        .correlation_data = "0815",
        .correlation_data_len = 4,
    };

    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = config->mqtt_broker,
        // Disable certificate verification (insecure - OK for trusted local networks)
        .broker.verification.skip_cert_common_name_check = true,  // Skip CN check
        .broker.verification.use_global_ca_store = false,         // Don't use system CA
        .broker.verification.certificate = NULL,                  // No CA cert provided
        .broker.verification.crt_bundle_attach = NULL,            // No CRT bundle
        .broker.verification.certificate_len = 0,
        .broker.verification.certificate = NULL,
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .network.disable_auto_reconnect = true,
        .credentials.username = "esp32",
        .credentials.authentication.password = "esp32",
        .session.last_will.topic = MS2_TAG"/lwt",
        .session.last_will.msg = "offline",
        .session.last_will.msg_len = 7,
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
    };

    ESP_LOGI(TAG, "Connect to %s", config->mqtt_broker);
    ui_set_label_text(ui->lbl_mqtt_status, "Connecting...");

    mqtt_client = esp_mqtt_client_init(&mqtt5_cfg);

    /* Set connection properties and user properties */
    esp_mqtt5_client_set_user_property(&connect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_user_property(&connect_property.will_user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_connect_property(mqtt_client, &connect_property);

    /* If you call esp_mqtt5_client_set_user_property to set user properties, DO NOT forget to delete them.
     * esp_mqtt5_client_set_connect_property will malloc buffer to store the user_property and you can delete it after
     */
    esp_mqtt5_client_delete_user_property(connect_property.user_property);
    esp_mqtt5_client_delete_user_property(connect_property.will_user_property);

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

void mqtt_stop()
{
    if (mqtt_client == NULL) return;
    esp_mqtt_client_stop(mqtt_client);     // Clean disconnect + stop task
    vTaskDelay(pdMS_TO_TICKS(100));  // Small delay for network cleanup
    esp_mqtt_client_destroy(mqtt_client);  // Free resources
    mqtt_client = NULL;
    mqtt_connected = false;
    ui_set_label_text(ui->lbl_mqtt_status, "Stopped");
}

void mqtt_publish_values()
{
    if (!mqtt_connected && config->mqtt_auto_connect) {
        mqtt_start();
    }
    if (!mqtt_connected) return;
    if (!gps_update && !bmx280lo_update && !bmx280hi_update && !mhz19_update && !scd4x_calibrate &&
        !scd4x_update && !yys_update && !sps30_update && !adxl345_update && !qmc5883l_update) {
        return;
    }
 
    static char buf[256];

    ESP_LOGI(TAG, "Publish %u", bmx280lo_update);
    if (bmx280lo_update) {
        bmx280_values_t *values = &bmx280lo->values;
        sprintf(buf, "{temp=%f,hum=%f,press=%f,alt=%f}", values->temperature, values->humidity, values->pressure, values->altitude);
        esp_mqtt_client_publish(mqtt_client, MS2_TAG"/bmx280lo", buf, 0, 1, 0);
    }
}
