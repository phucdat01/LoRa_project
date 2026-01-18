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
#define MQTT_WINDOW_MS  3000    // Dành 3 giây cuối để gửi MQTT

/// [QUAN TRỌNG] Cấu hình các Phase thời gian (tính bằng ms từ lúc Start Cycle)
// Phase 1 (Downlink - Beacon): 0 - 200ms
// Phase 2 (Uplink - Data): 1500ms - 26500ms
// Phase 3 (Uplink - Join): 26500ms - 27000ms
#define PHASE_JOIN_START_MS 26500 

// --- LOẠI GÓI TIN (HEADER) ---
#define PKT_BEACON    1 // Gói đồng bộ thời gian (Gateway -> Node)
#define PKT_DATA      2 // Gói dữ liệu cảm biến (Node -> Gateway)
#define PKT_JOIN_REQ  3 // Gói xin gia nhập mạng (Node -> Gateway)
#define PKT_JOIN_ACK  4 // Gói chấp nhận gia nhập (Gateway -> Node)

// Structs: Dùng __attribute__((packed)) để đảm bảo không bị độn byte
// Giúp khớp cấu trúc dữ liệu 100% giữa ESP32 (32-bit) và STM32 (32-bit)
typedef struct __attribute__((packed)) {
    uint8_t  type; 
    uint16_t cycleID;           // Số thứ tự chu kỳ (để debug)
    uint32_t serverTime;        // Thời gian hệ thống (nếu có)
    uint16_t cycleDuration;     // Độ dài chu kỳ (30s)
    uint16_t reportInterval;    // Cấu hình tần suất gửi (Ví dụ: 2 chu kỳ gửi 1 lần)
    uint8_t  checksum;          // Kiểm tra lỗi XOR
} BeaconPacket_t;

typedef struct __attribute__((packed)) {
    uint8_t type; 
    uint8_t node_id;            // ID định danh của Node (1, 2, 3...)
    int16_t temperature;        // Nhiệt độ nhân 100 (để gửi số nguyên)
    uint16_t humidity;          // Độ ẩm nhân 10
    uint8_t soil_moisture;      // Độ ẩm đất (%)
    uint8_t battery_level;      // Pin (%)
    uint8_t checksum;
} DataPacket_t;

typedef struct __attribute__((packed)) {
    uint8_t type; 
    uint32_t chipID;            // UUID của chip STM32 (Duy nhất)
    uint8_t checksum;
} JoinReqPacket_t;

typedef struct __attribute__((packed)) {
    uint8_t type; 
    uint32_t chipID; 
    uint8_t assignedID;         // ID mà Gateway cấp cho Node
    uint8_t checksum;
} JoinAckPacket_t;

// --- QUEUE QUẢN LÝ JOIN (HÀNG ĐỢI GIA NHẬP) ---
// Gateway không xử lý Join ngay lập tức để tránh làm gián đoạn quá trình nhận dữ liệu Data.
// Thay vào đó, nó lưu các yêu cầu Join vào hàng đợi và xử lý (gửi ACK) vào đầu chu kỳ tiếp theo.
#define MAX_PENDING_JOINS 20    // Số lượng Node tối đa có thể chờ duyệt trong 1 chu kỳ
typedef struct {
    uint32_t chipID;            // UUID của Node chờ
    uint8_t assignedID;         // ID đã dự kiến cấp phát cho Node đó
} JoinQueueItem_t;

JoinQueueItem_t join_queue[MAX_PENDING_JOINS];  // Mảng lưu hàng đợi
uint8_t join_queue_count = 0;                   // Biến đếm số lượng Node đang chờ trong hàng đợi    

// --- BUFFER DATA (BỘ ĐỆM DỮ LIỆU) ---
// Mảng lưu trữ dữ liệu mới nhất nhận được từ các Node.
// Dữ liệu này sẽ được gom lại và gửi lên MQTT Server một lần vào cuối chu kỳ (Phase 4).
typedef struct { 
    float temp; 
    float hum; 
    int soil; 
    int bat;
    bool updated;                                // Cờ đánh dấu: true = có dữ liệu mới chưa gửi
} NodeBuffer_t;
NodeBuffer_t node_data[MAX_NODES + 1];           // Index từ 1 đến MAX_NODES (bỏ qua index 0)

// --- QUẢN LÝ NODE (BẢNG ĐỊNH TUYẾN) ---
// Bảng ánh xạ giữa ChipID (UUID dài) và NodeID (ID ngắn gọn dùng trong mạng)
typedef struct { 
    uint32_t chipID; 
    uint8_t nodeID; 
} NodeEntry_t;
NodeEntry_t node_table[MAX_NODES + 1];      // Bảng lưu trữ thông tin các Node đã gia nhập
uint8_t next_node_id = 1;                   // Biến lưu ID tiếp theo sẽ được cấp phát (tăng dần)

// Biến toàn cục quản lý mạng
uint16_t global_cycle_id = 1;               // Đếm số chu kỳ đã chạy
uint16_t config_cycle_sec = 30;             // Mặc định 30s/chu kỳ
uint16_t config_report_interval = 1;        // Mặc định gửi liên tục (1)

// Hàm tính Checksum đơn giản (XOR)
// Dùng để kiểm tra tính toàn vẹn của dữ liệu nhận được
uint8_t XOR_Calc(const void *buf, uint8_t len) {
    uint8_t x = 0; uint8_t *p = (uint8_t*)buf;
    for (uint8_t i = 0; i < len; i++) x ^= p[i];
    return x;
}

// --- NVS FUNCTIONS (LƯU TRỮ FLASH) ---

// Lưu bảng danh sách Node (node_table) và ID tiếp theo (next_node_id) vào bộ nhớ Flash.
// Giúp Gateway ghi nhớ các Node đã gia nhập ngay cả khi bị mất điện hoặc khởi động lại.
void save_nodes_to_nvs() {
    nvs_handle_t h; 
    if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {         // Mở namespace "storage"
        nvs_set_blob(h, "ntable", node_table, sizeof(node_table));  // Lưu mảng struct node_table dạng binary (blob)
        nvs_set_u8(h, "nid", next_node_id);                         // Lưu biến next_node_id (8-bit)
        nvs_commit(h);                                              // Xác nhận ghi dữ liệu
        nvs_close(h);                                               // Đóng handle
    }
}

// Đọc lại bảng danh sách Node từ bộ nhớ Flash khi khởi động.
void load_nodes_from_nvs() {
    nvs_handle_t h; 
    if (nvs_open("storage", NVS_READONLY, &h) == ESP_OK) {
        size_t size = sizeof(node_table);
        nvs_get_blob(h, "ntable", node_table, &size);               // Đọc blob vào mảng node_table
        nvs_get_u8(h, "nid", &next_node_id);                        // Đọc biến next_node_id
        nvs_close(h);
        ESP_LOGI(TAG, "Loaded Nodes from NVS. Next ID: %d", next_node_id);
    }
}

// Lưu cấu hình mạng (Cycle time, Report interval) vào Flash.
// Cho phép thay đổi cấu hình động mà không cần nạp lại code, và giữ cấu hình sau khi reset.
void save_config_to_nvs() {
    nvs_handle_t h; 
    nvs_open("storage", NVS_READWRITE, &h);
    nvs_set_u16(h, "cycle", config_cycle_sec); 
    nvs_set_u16(h, "report", config_report_interval); 
    nvs_commit(h); 
    nvs_close(h);
}

// Đọc cấu hình mạng từ Flash khi khởi động.
void load_config_from_nvs() {
    nvs_handle_t h; 
    if (nvs_open("storage", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u16(h, "cycle", &config_cycle_sec); 
        nvs_get_u16(h, "report", &config_report_interval);
        nvs_close(h);
        ESP_LOGI(TAG, "Loaded Config: Cycle=%d, Report=%d", config_cycle_sec, config_report_interval);
    }
}

// Nhận lệnh cấu hình qua cổng Serial (UART) từ Terminal máy tính
// Cú pháp lệnh: CYCLE=xx (đặt chu kỳ xx giây) hoặc REPORT=x (đặt tần suất gửi x chu kỳ/lần)
void check_serial_command() {
    uint8_t data[64]; 
    // Đọc dữ liệu từ UART với timeout 10ms
    int len = uart_read_bytes(UART_NUM_0, data, sizeof(data) - 1, 10 / portTICK_PERIOD_MS);
    if (len > 0) { 
        data[len] = '\0';   // Kết thúc chuỗi ký tự
        // Kiểm tra lệnh CYCLE
        if (strncmp((char*)data, "CYCLE=", 6) == 0) {
            config_cycle_sec = atoi((char*)data + 6); 
            save_config_to_nvs();
            ESP_LOGI(TAG, "Updated Network Cycle: %d s", config_cycle_sec);
        }
        // Kiểm tra lệnh REPORT
        else if (strncmp((char*)data, "REPORT=", 7) == 0) {
            config_report_interval = atoi((char*)data + 7);
            save_config_to_nvs();
            ESP_LOGI(TAG, "Updated Report Interval: Every %d cycles", config_report_interval);
        }
    }
}

// Logic cấp phát ID thông minh: Kiểm tra xem Node đã từng gia nhập chưa
// Nếu có rồi: Trả về ID cũ. Nếu chưa: Cấp ID mới và lưu lại.
uint8_t assign_id(uint32_t uid) {
    // 1. Duyệt bảng Node để tìm xem ChipID này đã tồn tại chưa (Node cũ quay lại mạng)
    for(int i=1; i < next_node_id; i++) {
        if (node_table[i].chipID == uid) {
            ESP_LOGW(TAG, "Old Node Re-joining: UID %X -> ID %d", (unsigned int)uid, node_table[i].nodeID);
            return node_table[i].nodeID;    // Trả về ID cũ
        }
    }

    // 2. Nếu là Node mới hoàn toàn, kiểm tra xem mạng đã đầy chưa
    if (next_node_id > MAX_NODES) {
        ESP_LOGE(TAG, "MAX NODES REACHED!");
        return 0;           // Trả về 0 báo lỗi (không cấp được ID)
    }
    // 3. Cấp ID mới
    uint8_t id = next_node_id++; 
    node_table[id].chipID = uid; 
    node_table[id].nodeID = id;
    save_nodes_to_nvs();    // Lưu cập nhật vào Flash ngay lập tức  
    ESP_LOGI(TAG, "New Node Assigned: UID %X -> ID %d", (unsigned int)uid, id);
    return id;
}

// =========================================================================
// TASK CHÍNH: ĐIỀU PHỐI MẠNG TDMA (GATEWAY CORE)
// =========================================================================
void task_gateway(void *p) {
    // 1. Khởi tạo phần cứng LoRa SX1278
    lora_init();
    lora_set_frequency(433E6); 
    lora_set_tx_power(17);
    lora_set_spreading_factor(7); 
    lora_set_sync_word(0x12);
    lora_enable_crc(); 
    lora_set_bandwidth(125E3);

    uint8_t buf[128];           // Buffer nhận dữ liệu LoRa
    esp_task_wdt_add(NULL);     // Đăng ký Watchdog Timer (để chống treo)

    for(;;) {
        // [QUAN TRỌNG NHẤT] Lấy mốc thời gian bắt đầu chu kỳ (t=0)
        uint32_t cycle_start_tick = xTaskGetTickCount();
        check_serial_command(); // Kiểm tra có lệnh cấu hình không

        // -----------------------------------------------------------
        // 1. GỬI BEACON (PHASE 1 - DOWNLINK) - Bắt đầu chu kỳ
        // -----------------------------------------------------------
        // Tạo gói tin Beacon chứa thông tin đồng bộ
        BeaconPacket_t b = { 
            .type = PKT_BEACON, 
            .cycleID = global_cycle_id, 
            .cycleDuration = config_cycle_sec,
            .reportInterval = config_report_interval // Gửi cấu hình hiện tại xuống cho các Node biết
        };
        b.checksum = XOR_Calc(&b, sizeof(b)-1);      // Tính checksum
        
        lora_send_packet((uint8_t*)&b, sizeof(b));   // Phát sóng gói Beacon
        // Đợi gửi xong và thêm một khoảng trễ (Guard Time) trước khi chuyển sang việc khác.
        // Điều này đảm bảo các Node kịp nhận Beacon và đồng bộ trước khi Gateway làm việc tiếp theo.
        // Đây cũng là khoảng thời gian Phase 1 (0 - 200ms)
        vTaskDelay(pdMS_TO_TICKS(150)); 

        // Log báo hiệu bắt đầu chu kỳ mới
        ESP_LOGI(TAG, ">>> CYCLE %d START (Dur: %ds, Rep: %d)", global_cycle_id, config_cycle_sec, config_report_interval);
        global_cycle_id++;  // Tăng số đếm chu kỳ

        // -----------------------------------------------------------
        // 2. XẢ HÀNG ACK (PHASE 1 - DOWNLINK - Tiếp theo)
        // -----------------------------------------------------------
        // Kiểm tra xem có Node nào xin gia nhập từ chu kỳ TRƯỚC đang chờ không
        if (join_queue_count > 0) {
            for (int i = 0; i < join_queue_count; i++) {
                // Tạo gói tin Join ACK để trả lời
                JoinAckPacket_t ack = {
                    .type = PKT_JOIN_ACK,
                    .chipID = join_queue[i].chipID,
                    .assignedID = join_queue[i].assignedID
                };
                ack.checksum = XOR_Calc(&ack, sizeof(ack)-1);
                
                lora_send_packet((uint8_t*)&ack, sizeof(ack));      // Gửi ACK
                ESP_LOGI(TAG, "FLUSH ACK -> Node %d (UID: %X)", ack.assignedID, (unsigned int)ack.chipID);
                // Delay nhỏ giữa các gói ACK để tránh nghẽn mạng LoRa 
                vTaskDelay(pdMS_TO_TICKS(10)); 
            }
            join_queue_count = 0; // Xóa sạch hàng đợi sau khi đã xử lý xong
        }

        // Chuyển module LoRa sang chế độ Nhận (Receive Mode) để lắng nghe dữ liệu từ các Node
        lora_receive();

        // -----------------------------------------------------------
        // PHASE 3 & 4: CỬA SỔ NHẬN (RX WINDOW)
        // Dùng thời gian tuyệt đối để dừng đúng giây thứ 27
        
        
        // Tính Deadline: Phải dừng nhận lúc giây thứ 27 (27000ms) tính từ đầu chu kỳ// -----------------------------------------------------------
        uint32_t rx_deadline_ms = (config_cycle_sec * 1000) - MQTT_WINDOW_MS;

        // Vòng lặp nhận dữ liệu
        // Điều kiện: Thời gian trôi qua thực tế < Deadline (27000ms)
        while (((xTaskGetTickCount() - cycle_start_tick) * portTICK_PERIOD_MS) < rx_deadline_ms) {
            esp_task_wdt_reset();   // Reset Watchdog để không bị treo

            if (lora_received()) {
                int len = lora_receive_packet(buf, sizeof(buf));
                int rssi = lora_packet_rssi();

                if (len > 0) {
                    // --- XỬ LÝ GÓI DATA (PHASE 3 - DATA) ---
                    if (buf[0] == PKT_DATA && len == sizeof(DataPacket_t)) {
                        DataPacket_t *d = (DataPacket_t*)buf;
                        if (XOR_Calc(d, sizeof(DataPacket_t)-1) == d->checksum) {
                             // Lưu vào bộ đệm RAM để tí nữa gửi MQTT
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
                    // --- XỬ LÝ GÓI JOIN REQ (PHASE 4 - JOIN) ---
                    // Chỉ chấp nhận nếu thời gian hiện tại đã qua mốc PHASE_JOIN_START_MS (26.5s)
                    else if (buf[0] == PKT_JOIN_REQ && len == sizeof(JoinReqPacket_t)) {
                        
                        // [NEW SUPERFRAME LOGIC] Chỉ chấp nhận Join sau giây 26.5
                        uint32_t current_time_ms = (xTaskGetTickCount() - cycle_start_tick) * portTICK_PERIOD_MS;
                        if (current_time_ms >= PHASE_JOIN_START_MS) { 
                            // Đúng giờ Join -> Xử lý
                            JoinReqPacket_t *req = (JoinReqPacket_t*)buf;
                            if (XOR_Calc(req, sizeof(JoinReqPacket_t)-1) == req->checksum) {
                                uint8_t target_id = assign_id(req->chipID);
                                
                                // Kiểm tra xem đã có trong hàng đợi chưa (tránh spam)
                                bool already_queued = false;
                                for(int k=0; k<join_queue_count; k++) {
                                    if(join_queue[k].chipID == req->chipID) {
                                        already_queued = true;
                                        break;
                                    }
                                }

                                // Thêm vào hàng đợi (Pending)
                                if (!already_queued && join_queue_count < MAX_PENDING_JOINS) {
                                    join_queue[join_queue_count].chipID = req->chipID;
                                    join_queue[join_queue_count].assignedID = target_id;
                                    join_queue_count++;
                                    ESP_LOGI(TAG, "QUEUED Join Req -> Node %d (Pending: %d) RSSI: %d", target_id, join_queue_count, rssi);
                                }
                            }
                        } else {
                            // Sai giờ (đang giờ Data mà chen ngang) -> Từ chối
                            ESP_LOGW(TAG, "REJECT JoinReq during DATA Phase! (Time: %d ms)", current_time_ms);
                        }
                    }
                }
                lora_receive();             // Tiếp tục nhận gói tiếp theo
            }
            vTaskDelay(pdMS_TO_TICKS(10));  // Nhường CPU 10ms
        }

        // -----------------------------------------------------------
        // PHASE 5: GỬI MQTT (UPLOADING)
        // -----------------------------------------------------------
        
        ESP_LOGW(TAG, "--- UPLOADING MQTT ---");
        char json[128];
        for (int i = 1; i <= MAX_NODES; i++) {
            if (node_data[i].updated) {
                // Đóng gói JSON
                snprintf(json, sizeof(json), "{\"id\":%d,\"temp\":%.1f,\"hum\":%.1f,\"soil\":%d,\"bat\":%d}", 
                         i, node_data[i].temp, node_data[i].hum, node_data[i].soil, node_data[i].bat);
                mqtt_service_publish(json);         // Gửi lên Cloud
                node_data[i].updated = false;       // Xóa cờ
                vTaskDelay(pdMS_TO_TICKS(20));      // Delay tránh nghẽn WiFi
            }
        }

        // -----------------------------------------------------------
        // PRECISE WAIT (BÙ GIỜ)
        // -----------------------------------------------------------
        
        uint32_t now_tick = xTaskGetTickCount();
        uint32_t elapsed_ms = (now_tick - cycle_start_tick) * portTICK_PERIOD_MS;
        
        // Mục tiêu: Tổng chu kỳ phải đúng 30 giây (30000ms)
        uint32_t target_ms = config_cycle_sec * 1000;

        if (target_ms > elapsed_ms) {
            uint32_t wait_ms = target_ms - elapsed_ms;
            vTaskDelay(pdMS_TO_TICKS(wait_ms));                         // Ngủ bù phần còn thiếu
        } else {
            ESP_LOGW(TAG, "Cycle Overrun! (Took %d ms)", elapsed_ms);   // Cảnh báo nếu hệ thống chạy chậm
        }
    }
}

// =============================================================================
// 6. MAIN APP
// =============================================================================

void app_main() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // Tắt Brownout Detector
    nvs_flash_init(); 
    load_config_from_nvs(); 
    load_nodes_from_nvs();

    // Cấu hình UART Console
    const uart_config_t uart_cfg = { 
        .baud_rate = 115200, 
        .data_bits = UART_DATA_8_BITS, 
        .parity = UART_PARITY_DISABLE, 
        .stop_bits = UART_STOP_BITS_1, 
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE 
    };
    uart_param_config(UART_NUM_0, &uart_cfg); 
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    
    // Khởi động WiFi & MQTT
    wifi_init(); 
    mqtt_service_start(); 
    
    // Đợi 2 giây ổn định rồi chạy Gateway
    vTaskDelay(pdMS_TO_TICKS(2000));
    xTaskCreate(task_gateway, "gw_task", 8192, NULL, 5, NULL);
}