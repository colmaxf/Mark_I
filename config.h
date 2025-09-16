// config.h
#pragma once


//===================== Cấu hình hệ thống ==================---//
#define AGV_ID 1

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
//===================== Cấu hình hệ thống ==================---//

//Config chung
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Khoảng cách an toàn (cm)
#define SAFE_DISTANCE_CM 200.0f
// Khoảng cách cảnh báo (cm)
#define WARNING_DISTANCE_CM 100.0f
// Khoảng cách dừng khẩn cấp (cm)
#define EMERGENCY_STOP_DISTANCE_CM 50.0f

// Speed control
#define SPEED_STEP 100
#define ACCEL_TIME_MS 2000
#define MIN_START_SPEED 200
