// config.h
#pragma once

// Cờ bật tắt ghi log
#define ENABLE_LOG  

//config ip, port PLC
#define PLC_IP "192.168.4.5"
#define PLC_PORT 5000

//config ip, port Lidar
#define LIDAR_HOST_IP "192.168.4.10"
#define LIDAR_CLIENT_IP "192.168.4.101"
#define LIDAR_PORT 2368
#define LIDAR_CLIENT_PORT 2368

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Khoảng cách an toàn (cm)
#define SAFE_DISTANCE_CM 200.0f
// Khoảng cách cảnh báo (cm)
#define WARNING_DISTANCE_CM 100.0f
// Khoảng cách dừng khẩn cấp (cm)
#define EMERGENCY_STOP_DISTANCE_CM 50.0f
