#include "mqtt_service.h"
#include "esp_log.h"

static const char *TAG = "MQTT_SERVICE";
static esp_mqtt_client_handle_t client = NULL;

#ifndef MQTT_URI
#define MQTT_URI "mqtt://thingsboard.cloud"
#endif

#ifndef ACCESS_TOKEN
#define ACCESS_TOKEN "QjScOJql9M5MeFOEc1DJ"
#endif

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    if (event_data == NULL) {
        ESP_LOGW(TAG, "mqtt_event_handler called with NULL event_data");
        return;
    }
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error");
            break;
        default:
            ESP_LOGI(TAG, "MQTT Event ID: %d", event->event_id);
            break;
    }
}

void mqtt_service_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
        .credentials.username = ACCESS_TOKEN,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to init MQTT client");
        return;
    }
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t err = esp_mqtt_client_start(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %d", err);
    }
}

void mqtt_service_stop(void) {
    if (client) {
        esp_mqtt_client_stop(client);
        esp_mqtt_client_destroy(client);
        client = NULL;
        ESP_LOGI(TAG, "MQTT client stopped and destroyed");
    }
}

void mqtt_service_publish(const char *payload) {
    if (client) {
        int msg_id = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", payload, 0, 1, 0);
        ESP_LOGI(TAG, "Published (msg_id=%d): %s", msg_id, payload);
    } else {
        ESP_LOGW(TAG, "MQTT client not initialized");
    }
}

// Dùng để gửi trạng thái (vd: {"fan_state":true})
void mqtt_service_publish_attribute(const char *payload) {
    if (client) {
        int msg_id = esp_mqtt_client_publish(client, "v1/devices/me/attributes", payload, 0, 1, 0);
        ESP_LOGI(TAG, "Published attribute (msg_id=%d): %s", msg_id, payload);
    } else {
        ESP_LOGW(TAG, "MQTT client not initialized");
    }
}

