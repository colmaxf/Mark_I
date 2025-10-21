// config.h
#pragma once


//===================== Cấu hình hệ thống ==================---//
#define AGV_ID 2

#if AGV_ID == 1
    //Config AGV 1
    //config ip, port PLC
    #define PLC_IP "192.168.3.5"
    #define PLC_PORT 5000

    //config ip, port Lidar
    #define LIDAR_HOST_IP "192.168.3.10"
    #define LIDAR_CLIENT_IP "192.168.3.101"
    #define LIDAR_PORT 2368
    #define LIDAR_CLIENT_PORT 2368
//--------------------------------------------------------------------//
#elif AGV_ID == 2
    //Config AGV 2
    //config ip, port PLC
    #define PLC_IP "192.168.4.5"
    #define PLC_PORT 5000
    
    //config ip, port Lidar
    #define LIDAR_HOST_IP "192.168.4.10"
    #define LIDAR_CLIENT_IP "192.168.4.101"
    #define LIDAR_PORT 2368
    #define LIDAR_CLIENT_PORT 2368
#endif
//----------------------Server--------------------------------//
#define SERVER_IP "100.93.107.30"  // IP của server
#define SERVER_PORT 5000           // Port của server
#define COMM_SEND_INTERVAL_MS 10 //50 // Gửi status mỗi 200ms
//----------------------------------------------------//
//===================== Cấu hình hệ thống ==================---//

//Battery port
#define BATTERY_BMS_SERIAL_PORT "/dev/ttyAMA10"

//Config chung
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// Thời gian chờ kết nối lại LiDAR (giây)
#define LIDAR_RECONNECT_DELAY_SECONDS 2
#define MAX_RECONNECT_ATTEMPTS 5
// Khoảng cách an toàn (cm)
#define SAFE_DISTANCE_CM 200.0f
// Khoảng cách cảnh báo (cm)
#define WARNING_DISTANCE_CM 100.0f
// Khoảng cách dừng khẩn cấp (cm)
#define EMERGENCY_STOP_DISTANCE_CM 70.0f

// Speed control
#define SPEED_STEP 100
#define ACCEL_TIME_MS 30000 // 3 0s
#define MIN_START_SPEED 200

#define TEST_KEYBOARD_MODE 1
// Cấp quyền đọc input device
// Tạo udev rule để service có quyền đọc keyboard:
// sudo nano /etc/udev/rules.d/99-input.rules
// Thêm nội dung:
// KERNEL=="event*", SUBSYSTEM=="input", MODE="0666"
//
// Apply rule:
// sudo udevadm control --reload-rules
// sudo udevadm trigger

//
