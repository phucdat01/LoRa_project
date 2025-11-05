#pragma once

#include "mqtt_client.h"

// Khởi tạo và kết nối MQTT client
void mqtt_service_start(void);

// Stop and cleanup MQTT client
void mqtt_service_stop(void);

// Gửi dữ liệu telemetry lên ThingsBoard
void mqtt_service_publish(const char *payload);

// Gửi attribute (ví dụ: trạng thái quạt, chế độ auto/manual)
void mqtt_service_publish_attribute(const char *payload);

