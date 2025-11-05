#include "wifi_service.h"
#include "esp_wifi.h"
#include "esp_log.h"

#define WIFI_SSID "Free"
#define WIFI_PASS "abc@1234"

static const char *TAG = "WIFI";

void wifi_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();

    ESP_LOGI(TAG, "WiFi connecting to %s...", WIFI_SSID);
}