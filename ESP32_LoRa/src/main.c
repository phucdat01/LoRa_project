#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lora.h"
#include "nvs_flash.h"
#include "wifi_service.h"
#include "mqtt_service.h"

#define TAG "LoRa_Gateway" // Tag dùng cho log (ESP_LOGI, ESP_LOGE, ...)

void app_main(void)
{
    nvs_flash_init();       // Khởi tạo NVS (Non-Volatile Storage) để lưu cấu hình WiFi, MQTT, v.v.
    wifi_init();            // Khởi tạo WiFi    
    mqtt_service_start();   // Khởi động dịch vụ MQTT

    ESP_LOGI(TAG, "Initializing LoRa module...");
    if (!lora_init()) {
        ESP_LOGE(TAG, "LoRa init failed!");
        return;
    }

    // Cấu hình giống STM32
    lora_set_frequency(433E6);
    lora_set_spreading_factor(7);
    lora_set_bandwidth(125E3);
    lora_set_coding_rate(5);
    lora_set_sync_word(0x34);
    lora_set_preamble_length(8);
    lora_enable_crc();

    // Đặt module vào chế độ nhận liên tục
    lora_receive();

    uint8_t rx_buf[256];      // Buffer chứa dữ liệu nhận được
    int len;                  // Độ dài dữ liệu nhận được

    ESP_LOGI(TAG, "Gateway is now listening...");

    while (1) {
        if (lora_received()) {
            len = lora_receive_packet(rx_buf, sizeof(rx_buf)); // Đọc gói tin vào rx_buf
            // Nếu độ dài hợp lệ thì xử lý
            if (len > 0 && len < sizeof(rx_buf)) {
                rx_buf[len] = '\0';  // Thêm ký tự kết thúc chuỗi
                
                // In ra log nội dung gói tin + RSSI + SNR
                ESP_LOGI(TAG, "📡 Received (%d bytes): %s | RSSI: %d dBm | SNR: %.2f dB",
                           len, rx_buf, lora_packet_rssi(), lora_packet_snr());

                float temp = 0, humi = 0, soil = 0;
;
                // Parse dữ liệu theo định dạng Node gửi: "T=xx.xC,RH=yy.y%,Soil=zz.z%Vol"
                if (sscanf((char*)rx_buf, "T=%fC,RH=%f%%,Soil=%f%%Vol", &temp, &humi, &soil) == 3) {
                    ESP_LOGI(TAG, "🌡 Temp=%.1f °C | 💧 Humi=%.1f %% | 🌱 Soil=%.1f %%Vol", temp, humi, soil);
                    
                    // Chuẩn bị payload JSON để publish lên MQTT
                    char payload[64];
                    snprintf(payload, sizeof(payload),
                             "{\"temperature\":%.1f,\"humidity\":%.1f,\"soil\":%.1f}", temp, humi, soil);
                    
                    // Publish dữ liệu lên MQTT broker
                    mqtt_service_publish(payload);

                } else {
                    ESP_LOGW(TAG, "Không parse được dữ liệu Node gửi!");
                }

                // Đặt lại chế độ nhận
                lora_receive();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // giảm tải CPU
    }
}