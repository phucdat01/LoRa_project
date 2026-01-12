#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_task_wdt.h"

#include "wifi_service.h"
#include "mqtt_service.h"
#include "lora.h"

static const char *TAG = "GATEWAY_TDMA";

// --- CẤU HÌNH TIMING (SUPERFRAME) ---
#define MAX_NODES       100
#define MQTT_WINDOW_MS  3000

// [QUAN TRỌNG] Cấu hình các Phase thời gian (tính bằng ms từ lúc Start Cycle)
// Phase 1 (Downlink): 0 - 1500ms
// Phase 2 (Data): 1500 - 26500ms
#define PHASE_JOIN_START_MS 26500 

// --- GÓI TIN ---
#define PKT_BEACON    1
#define PKT_DATA      2
#define PKT_JOIN_REQ  3
#define PKT_JOIN_ACK  4

// Structs (Phải khớp 100% với STM32)
typedef struct __attribute__((packed)) {
    uint8_t  type; 
    uint16_t cycleID; 
    uint32_t serverTime; 
    uint16_t cycleDuration; 
    uint16_t reportInterval; // [MỚI] Thêm biến này để điều khiển tần suất gửi
    uint8_t  checksum;
} BeaconPacket_t;

typedef struct __attribute__((packed)) {
    uint8_t type; 
    uint8_t node_id; 
    int16_t temperature; 
    uint16_t humidity; 
    uint8_t soil_moisture; 
    uint8_t battery_level;
    uint8_t checksum;
} DataPacket_t;

typedef struct __attribute__((packed)) {
    uint8_t type; 
    uint32_t chipID; 
    uint8_t checksum;
} JoinReqPacket_t;

typedef struct __attribute__((packed)) {
    uint8_t type; 
    uint32_t chipID; 
    uint8_t assignedID; 
    uint8_t checksum;
} JoinAckPacket_t;

// --- QUEUE QUẢN LÝ JOIN ---
#define MAX_PENDING_JOINS 20
typedef struct {
    uint32_t chipID;
    uint8_t assignedID;
} JoinQueueItem_t;

JoinQueueItem_t join_queue[MAX_PENDING_JOINS];
uint8_t join_queue_count = 0;

// --- BUFFER DATA ---
typedef struct { 
    float temp; 
    float hum; 
    int soil; 
    int bat;
    bool updated; 
} NodeBuffer_t;
NodeBuffer_t node_data[MAX_NODES + 1];

// --- QUẢN LÝ NODE ---
typedef struct { 
    uint32_t chipID; 
    uint8_t nodeID; 
} NodeEntry_t;
NodeEntry_t node_table[MAX_NODES + 1];
uint8_t next_node_id = 1;

uint16_t global_cycle_id = 1;
uint16_t config_cycle_sec = 30; // 30s một chu kỳ mạng
uint16_t config_report_interval = 1; // [MỚI] Mặc định 1 (Gửi liên tục)

uint8_t XOR_Calc(const void *buf, uint8_t len) {
    uint8_t x = 0; uint8_t *p = (uint8_t*)buf;
    for (uint8_t i = 0; i < len; i++) x ^= p[i];
    return x;
}

// --- NVS FUNCTIONS ---
void save_nodes_to_nvs() {
    nvs_handle_t h; 
    if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "ntable", node_table, sizeof(node_table));
        nvs_set_u8(h, "nid", next_node_id); 
        nvs_commit(h); 
        nvs_close(h);
    }
}

void load_nodes_from_nvs() {
    nvs_handle_t h; 
    if (nvs_open("storage", NVS_READONLY, &h) == ESP_OK) {
        size_t size = sizeof(node_table);
        nvs_get_blob(h, "ntable", node_table, &size);
        nvs_get_u8(h, "nid", &next_node_id); 
        nvs_close(h);
        ESP_LOGI(TAG, "Loaded Nodes from NVS. Next ID: %d", next_node_id);
    }
}

// [CẬP NHẬT] Lưu cả Cycle và Report Interval
void save_config_to_nvs() {
    nvs_handle_t h; 
    nvs_open("storage", NVS_READWRITE, &h);
    nvs_set_u16(h, "cycle", config_cycle_sec); 
    nvs_set_u16(h, "report", config_report_interval); 
    nvs_commit(h); 
    nvs_close(h);
}

// [CẬP NHẬT] Đọc cả Cycle và Report Interval
void load_config_from_nvs() {
    nvs_handle_t h; 
    if (nvs_open("storage", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u16(h, "cycle", &config_cycle_sec); 
        nvs_get_u16(h, "report", &config_report_interval);
        nvs_close(h);
        ESP_LOGI(TAG, "Loaded Config: Cycle=%d, Report=%d", config_cycle_sec, config_report_interval);
    }
}

// [CẬP NHẬT] Thêm lệnh REPORT=...
void check_serial_command() {
    uint8_t data[64]; 
    int len = uart_read_bytes(UART_NUM_0, data, sizeof(data) - 1, 10 / portTICK_PERIOD_MS);
    if (len > 0) { 
        data[len] = '\0';
        if (strncmp((char*)data, "CYCLE=", 6) == 0) {
            config_cycle_sec = atoi((char*)data + 6); 
            save_config_to_nvs();
            ESP_LOGI(TAG, "Updated Network Cycle: %d s", config_cycle_sec);
        }
        else if (strncmp((char*)data, "REPORT=", 7) == 0) {
            config_report_interval = atoi((char*)data + 7);
            save_config_to_nvs();
            ESP_LOGI(TAG, "Updated Report Interval: Every %d cycles", config_report_interval);
        }
    }
}

// Logic cấp phát ID thông minh
uint8_t assign_id(uint32_t uid) {
    for(int i=1; i < next_node_id; i++) {
        if (node_table[i].chipID == uid) {
            ESP_LOGW(TAG, "Old Node Re-joining: UID %X -> ID %d", (unsigned int)uid, node_table[i].nodeID);
            return node_table[i].nodeID; 
        }
    }

    if (next_node_id > MAX_NODES) {
        ESP_LOGE(TAG, "MAX NODES REACHED!");
        return 0; 
    }
    
    uint8_t id = next_node_id++; 
    node_table[id].chipID = uid; 
    node_table[id].nodeID = id;
    save_nodes_to_nvs(); 
    ESP_LOGI(TAG, "New Node Assigned: UID %X -> ID %d", (unsigned int)uid, id);
    return id;
}

// --- TASK CHÍNH ---
void task_gateway(void *p) {
    lora_init();
    lora_set_frequency(433E6); 
    lora_set_tx_power(17);
    lora_set_spreading_factor(7); 
    lora_set_sync_word(0x12);
    lora_enable_crc(); 
    lora_set_bandwidth(125E3);

    uint8_t buf[128];
    esp_task_wdt_add(NULL); 

    for(;;) {
        check_serial_command();

        // -----------------------------------------------------------
        // 1. GỬI BEACON (PHASE 1 - DOWNLINK)
        // -----------------------------------------------------------
        BeaconPacket_t b = { 
            .type = PKT_BEACON, 
            .cycleID = global_cycle_id, 
            .cycleDuration = config_cycle_sec,
            .reportInterval = config_report_interval // Gửi cấu hình xuống Node
        };
        b.checksum = XOR_Calc(&b, sizeof(b)-1);
        
        lora_send_packet((uint8_t*)&b, sizeof(b));
        // Đợi gửi xong và thêm chút delay an toàn trước khi bắn ACK
        vTaskDelay(pdMS_TO_TICKS(200)); 

        ESP_LOGI(TAG, ">>> CYCLE %d START (Dur: %ds, Rep: %d)", global_cycle_id, config_cycle_sec, config_report_interval);
        global_cycle_id++;

        // -----------------------------------------------------------
        // 2. XẢ HÀNG ACK (PHASE 1 - DOWNLINK)
        // -----------------------------------------------------------
        if (join_queue_count > 0) {
            for (int i = 0; i < join_queue_count; i++) {
                JoinAckPacket_t ack = {
                    .type = PKT_JOIN_ACK,
                    .chipID = join_queue[i].chipID,
                    .assignedID = join_queue[i].assignedID
                };
                ack.checksum = XOR_Calc(&ack, sizeof(ack)-1);
                
                lora_send_packet((uint8_t*)&ack, sizeof(ack));
                ESP_LOGI(TAG, "FLUSH ACK -> Node %d (UID: %X)", ack.assignedID, (unsigned int)ack.chipID);
                
                vTaskDelay(pdMS_TO_TICKS(150)); 
            }
            join_queue_count = 0; // Xóa hàng đợi
        }

        // Chuyển sang chế độ Nhận
        lora_receive();

        uint32_t start_tick = xTaskGetTickCount();
        uint32_t rx_window_ms = (config_cycle_sec * 1000) - MQTT_WINDOW_MS;
        if (rx_window_ms > 60000) rx_window_ms = 5000; 

        // -----------------------------------------------------------
        // 3. CỬA SỔ NHẬN (PHASE 2 & PHASE 3)
        // -----------------------------------------------------------
        while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(rx_window_ms)) {
            esp_task_wdt_reset(); 

            if (lora_received()) {
                int len = lora_receive_packet(buf, sizeof(buf));
                int rssi = lora_packet_rssi();

                if (len > 0) {
                    // --- XỬ LÝ DATA (PHASE 2 - UPLINK) ---
                    if (buf[0] == PKT_DATA && len == sizeof(DataPacket_t)) {
                        DataPacket_t *d = (DataPacket_t*)buf;
                        if (XOR_Calc(d, sizeof(DataPacket_t)-1) == d->checksum) {
                             if (d->node_id > 0 && d->node_id <= MAX_NODES) {
                                node_data[d->node_id].temp = d->temperature/100.0;
                                node_data[d->node_id].hum = d->humidity/10.0;
                                node_data[d->node_id].soil = d->soil_moisture;
                                node_data[d->node_id].bat = d->battery_level;
                                node_data[d->node_id].updated = true;
                                ESP_LOGI(TAG, "RX Node %d | RSSI: %d", d->node_id, rssi);
                             }
                        } else {
                            ESP_LOGW(TAG, "Data Checksum Error Node %d", d->node_id);
                        }
                    } 
                    // --- XỬ LÝ JOIN REQ (PHASE 3 - JOIN WINDOW) ---
                    else if (buf[0] == PKT_JOIN_REQ && len == sizeof(JoinReqPacket_t)) {
                        
                        // [NEW SUPERFRAME LOGIC] Chỉ chấp nhận Join sau giây 26.5
                        uint32_t current_time_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
                        
                        if (current_time_ms >= PHASE_JOIN_START_MS) { 
                            // ĐÚNG GIỜ -> XỬ LÝ
                            JoinReqPacket_t *req = (JoinReqPacket_t*)buf;
                            if (XOR_Calc(req, sizeof(JoinReqPacket_t)-1) == req->checksum) {
                                uint8_t target_id = assign_id(req->chipID);
                                
                                bool already_queued = false;
                                for(int k=0; k<join_queue_count; k++) {
                                    if(join_queue[k].chipID == req->chipID) {
                                        already_queued = true;
                                        break;
                                    }
                                }

                                if (!already_queued && join_queue_count < MAX_PENDING_JOINS) {
                                    join_queue[join_queue_count].chipID = req->chipID;
                                    join_queue[join_queue_count].assignedID = target_id;
                                    join_queue_count++;
                                    ESP_LOGI(TAG, "QUEUED Join Req -> Node %d (Pending: %d) RSSI: %d", target_id, join_queue_count, rssi);
                                }
                            }
                        } else {
                            // SAI GIỜ (Chen ngang Data) -> TỪ CHỐI
                            ESP_LOGW(TAG, "REJECT JoinReq during DATA Phase! (Time: %d ms)", current_time_ms);
                        }
                    }
                }
                lora_receive();
            }
            vTaskDelay(pdMS_TO_TICKS(5)); 
        }

        // -----------------------------------------------------------
        // 4. GỬI MQTT (PHASE 4 - MAINTENANCE)
        // -----------------------------------------------------------
        ESP_LOGW(TAG, "--- UPLOADING MQTT ---");
        char json[128];
        for (int i = 1; i <= MAX_NODES; i++) {
            if (node_data[i].updated) {
                snprintf(json, sizeof(json), "{\"id\":%d,\"temp\":%.1f,\"hum\":%.1f,\"soil\":%d,\"bat\":%d}", 
                         i, node_data[i].temp, node_data[i].hum, node_data[i].soil, node_data[i].bat);
                mqtt_service_publish(json);
                node_data[i].updated = false;
                vTaskDelay(pdMS_TO_TICKS(20)); 
            }
        }

        uint32_t elapsed = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
        uint32_t total_wait_time = (MQTT_WINDOW_MS > elapsed) ? (MQTT_WINDOW_MS - elapsed) : 0;
        if (total_wait_time > 0) vTaskDelay(pdMS_TO_TICKS(total_wait_time));
    }
}

void app_main() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    nvs_flash_init(); 
    load_config_from_nvs(); 
    load_nodes_from_nvs();

    const uart_config_t uart_cfg = { 
        .baud_rate = 115200, 
        .data_bits = UART_DATA_8_BITS, 
        .parity = UART_PARITY_DISABLE, 
        .stop_bits = UART_STOP_BITS_1, 
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE 
    };
    uart_param_config(UART_NUM_0, &uart_cfg); 
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);

    wifi_init(); 
    mqtt_service_start(); 
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    xTaskCreate(task_gateway, "gw_task", 8192, NULL, 5, NULL);
}