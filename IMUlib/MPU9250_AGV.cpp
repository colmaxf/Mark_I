/*
 * MPU9250_AGV.cpp
 * AGV Navigation System Implementation
 * For Raspberry Pi with MPU9250 IMU Sensor
 */

#include "MPU9250_AGV.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <sys/time.h>
#include <cstring>
#include <iomanip>      // ← THÊM DÒNG NÀY (cho setprecision)
#include <sstream> 


// =================================================================================
// MadgwickFilter Implementation
// =================================================================================

float MadgwickFilter::invSqrt(float x) {
    // Sử dụng union để tránh strict-aliasing warning
    union {
        float f;
        uint32_t i;
    } conv;
    
    float halfx = 0.5f * x;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
    return conv.f;
}

/**
 * @brief Hàm khởi tạo đối tượng MadgwickFilter.
 * @param sampleFreq Tần số lấy mẫu (Hz).
 * @param beta Hệ số khuếch đại của bộ lọc.
 */
MadgwickFilter::MadgwickFilter(float sampleFreq, float beta) {
    this->beta = beta;
    this->invSampleFreq = 1.0f / sampleFreq;
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}

/**
 * @brief Cập nhật bộ lọc với dữ liệu 9 bậc tự do (con quay, gia tốc kế, từ kế).
 * @param gx Tốc độ quay trục x của con quay hồi chuyển (độ/giây).
 * @param gy Tốc độ quay trục y của con quay hồi chuyển (độ/giây).
 * @param gz Tốc độ quay trục z của con quay hồi chuyển (độ/giây).
 * @param ax Gia tốc trục x của gia tốc kế (g).
 * @param ay Gia tốc trục y của gia tốc kế (g).
 * @param az Gia tốc trục z của gia tốc kế (g).
 * @param mx Từ trường trục x của từ kế (milligauss).
 * @param my Từ trường trục y của từ kế (milligauss).
 * @param mz Từ trường trục z của từ kế (milligauss).
 */
void MadgwickFilter::update(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
    float _4bx, _4bz, _2q0, _2q1, _2q2, _2q3;
    float _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    
    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
    // Compute feedback only if accelerometer valid
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normalize accelerometer
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Normalize magnetometer
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        
        // Auxiliary variables
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;
        
        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;
        
        // Gradient descent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    
    // Integrate rate of change to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;
    
    // Normalize quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/**
 * @brief Cập nhật bộ lọc với dữ liệu 6 bậc tự do (chỉ con quay và gia tốc kế).
 * Được sử dụng khi dữ liệu từ kế không đáng tin cậy hoặc không có sẵn.
 * @param gx Tốc độ quay trục x của con quay hồi chuyển (độ/giây).
 * @param gy Tốc độ quay trục y của con quay hồi chuyển (độ/giây).
 * @param gz Tốc độ quay trục z của con quay hồi chuyển (độ/giây).
 * @param ax Gia tốc trục x của gia tốc kế (g).
 * @param ay Gia tốc trục y của gia tốc kế (g).
 * @param az Gia tốc trục z của gia tốc kế (g).
 */
void MadgwickFilter::updateIMU(float gx, float gy, float gz,
                               float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;
    
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;
    
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;
        
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;
    
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/**
 * @brief Lấy các góc Euler đã được tính toán.
 * 
 * CẤU HÌNH TRỤC THỰC TẾ:
 * - Y: Hướng lên trên (vuông góc mặt đất)
 * - Z: Hướng đằng TRƯỚC AGV
 * - X: Hướng sang TRÁI
 * 
 * ĐỊNH NGHĨA GÓC:
 * - Roll (xoay quanh Z):  Nghiêng TRÁI/PHẢI
 *   Roll > 0: nghiêng sang TRÁI
 *   Roll < 0: nghiêng sang PHẢI
 * 
 * - Pitch (xoay quanh X): Nghiêng TRƯỚC/SAU
 *   Pitch > 0: nghiêng về phía TRƯỚC
 *   Pitch < 0: nghiêng về phía SAU
 * 
 * - Yaw (xoay quanh Y):   Hướng di chuyển 0-360°
 * 
 * @param[out] roll Góc nghiêng trái/phải -180 to +180°
 * @param[out] pitch Góc nghiêng trước/sau -90 to +90°
 * @param[out] yaw Góc hướng (heading) 0-360°
 */
void MadgwickFilter::getEulerAngles(float &roll, float &pitch, float &yaw) {
    // Với cấu hình: Y lên, Z trước, X trái
    
    // Roll: xoay quanh Z (trục trước) → nghiêng TRÁI/PHẢI
    // Roll > 0: nghiêng TRÁI
    // Roll < 0: nghiêng PHẢI
    roll = atan2(2.0f * (q0 * q3 + q1 * q2), 
                 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
    
    // Pitch: xoay quanh X (trục trái) → nghiêng TRƯỚC/SAU
    // Pitch > 0: nghiêng TRƯỚC
    // Pitch < 0: nghiêng SAU
    float sinp = 2.0f * (q0 * q1 - q2 * q3);
    if (fabs(sinp) >= 1.0f)
        pitch = copysign(90.0f, sinp); // Gimbal lock tại ±90°
    else
        pitch = asin(sinp) * 57.29578f;
    
    // Yaw: xoay quanh Y (trục lên) → HƯỚNG DI CHUYỂN
    yaw = atan2(2.0f * (q0 * q2 + q1 * q3), 
                1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
    
    // Chuẩn hóa yaw về [0, 360)
    if(yaw < 0) yaw += 360.0f;
}

/**
 * @brief Lấy định hướng hiện tại dưới dạng quaternion.
 * @param[out] w Thành phần w của quaternion.
 * @param[out] x Thành phần x của quaternion.
 * @param[out] y Thành phần y của quaternion.
 * @param[out] z Thành phần z của quaternion.
 */
void MadgwickFilter::getQuaternion(float &w, float &x, float &y, float &z) {
    w = q0;
    x = q1;
    y = q2;
    z = q3;
}

/**
 * @brief Đặt hệ số khuếch đại của bộ lọc (beta).
 * @param beta Giá trị beta mới.
 */
void MadgwickFilter::setBeta(float beta) {
    this->beta = beta;
}

/**
 * @brief Lấy hệ số khuếch đại hiện tại của bộ lọc (beta).
 * @return Giá trị beta hiện tại.
 */
float MadgwickFilter::getBeta() const {
    return beta;
}

// =================================================================================
// AGVNavigation Implementation
// =================================================================================

/** @brief Hàm khởi tạo đối tượng AGVNavigation và các giá trị mặc định. */
AGVNavigation::AGVNavigation() 
    : i2c_fd(-1), madgwick(100.0f, 0.08f),
      heading(0), roll(0), pitch(0),
      pos_x(0), pos_y(0), velocity(0),
      raw_ax(0), raw_ay(0), raw_az(0),
      raw_gx(0), raw_gy(0), raw_gz(0),
      raw_mx(0), raw_my(0), raw_mz(0),
      temperature(0),
      max_tilt_angle(30.0f),
      collision_threshold(3.0f) {
    
    accel_scale = 16384.0f;
    gyro_scale = 131.0f;
    accel_offset[0] = accel_offset[1] = accel_offset[2] = 0.0f;
    gyro_offset[0] = gyro_offset[1] = gyro_offset[2] = 0.0f;
    mag_sensitivity[0] = mag_sensitivity[1] = mag_sensitivity[2] = 1.0f;
}

/** @brief Hàm hủy đối tượng AGVNavigation, đảm bảo file I2C được đóng. */
AGVNavigation::~AGVNavigation() {
    if(i2c_fd >= 0) {
        close(i2c_fd);
    }
}

/**
 * @brief Khởi tạo cảm biến MPU9250.
 * @param i2c_device Đường dẫn đến thiết bị I2C (ví dụ: "/dev/i2c-1").
 * @return True nếu khởi tạo thành công, ngược lại là false.
 */
bool AGVNavigation::begin(const char* i2c_device) {
    i2c_fd = open(i2c_device, O_RDWR);
    if(i2c_fd < 0) {
        LOG_WARNING << "[MPU9250] Error: Could not open " << i2c_device ;
        return false;
    }
    
    if(ioctl(i2c_fd, I2C_SLAVE, MPU9250_ADDR) < 0) {
        LOG_WARNING << "[MPU9250]  Error: Failed to connect to MPU9250" ;
        return false;
    }
    
    uint8_t who_am_i = readByte(WHO_AM_I);
    if(who_am_i != 0x71 && who_am_i != 0x73) {
        LOG_WARNING << "[MPU9250] Incorrect device ID: 0x" << std::hex << (int)who_am_i << std::dec << ". Expected 0x71 or 0x73." ;
        close(i2c_fd);
        i2c_fd = -1;
        return false;
    }
    
    LOG_INFO << "[MPU9250] MPU9250 detected (ID: 0x" << std::hex << (int)who_am_i << ")" << std::dec ;
    
    // Reset device
    writeByte(PWR_MGMT_1, 0x80);
    usleep(100000);
    
    // Wake up and set clock source
    writeByte(PWR_MGMT_1, 0x01);
    usleep(200000);
    
    // Enable all sensors
    writeByte(PWR_MGMT_2, 0x00);
    usleep(100000);
    
    // Configure sensors
    writeByte(CONFIG, 0x03);           // DLPF_CFG = 3
    writeByte(GYRO_CONFIG, 0x00);      // ±250°/s
    writeByte(ACCEL_CONFIG, 0x00);     // ±2g
    writeByte(ACCEL_CONFIG2, 0x03);    // Accel DLPF
    writeByte(INT_PIN_CFG, 0x22);      // Enable bypass
    usleep(100000);
    
    // Initialize magnetometer
    if(!initMagnetometer()) {
        LOG_WARNING << "[MPU9250]  Warning: Failed to initialize magnetometer" ;
    } else {
        LOG_INFO << "[MPU9250] Magnetometer (AK8963) is ready" ;
    }
    
    LOG_INFO << "[MPU9250] MPU9250 initialized successfully!" ;
    return true;
}

/**
 * @brief Khởi tạo từ kế AK8963.
 * @return True nếu thành công, ngược lại là false.
 */
bool AGVNavigation::initMagnetometer() {
    if(ioctl(i2c_fd, I2C_SLAVE, AK8963_ADDR) < 0) {
        return false;
    }
    
    // Check WHO_AM_I
    uint8_t mag_id = readByte(MAG_WHO_AM_I);
    if(mag_id != 0x48) {
        ioctl(i2c_fd, I2C_SLAVE, MPU9250_ADDR);
        return false;
    }
    
    // Reset magnetometer
    writeByte(MAG_CNTL2, 0x01);
    usleep(100000);
    
    // Enter Fuse ROM access mode
    writeByte(MAG_CNTL1, 0x0F);
    usleep(100000);
    
    // Read sensitivity adjustment values
    for(int i = 0; i < 3; i++) {
        uint8_t asa = readByte(MAG_ASAX + i);
        mag_sensitivity[i] = ((float)(asa - 128) / 256.0f) + 1.0f;
    }
    
    // Power down
    writeByte(MAG_CNTL1, 0x00);
    usleep(100000);
    
    // Set continuous measurement mode (16-bit, 100Hz)
    writeByte(MAG_CNTL1, 0x16);
    usleep(100000);
    
    // Switch back to MPU9250
    ioctl(i2c_fd, I2C_SLAVE, MPU9250_ADDR);
    return true;
}

/**
 * @brief Hiệu chỉnh con quay hồi chuyển bằng cách lấy trung bình các giá trị đọc khi đứng yên.
 * @param samples Số lượng mẫu để lấy trung bình.
 */
void AGVNavigation::calibrateGyro(int samples) {
    LOG_INFO << "[MPU9250] Calibrating gyroscope..." 
             << "  KEEP THE AGV STATIONARY!" ;
    sleep(2);
    
    float sum[3] = {0, 0, 0};
    
    for(int i = 0; i < samples; i++) {
        uint8_t data[6];
        readBytes(GYRO_XOUT_H, data, 6);
        
        for(int j = 0; j < 3; j++) {
            int16_t raw = (data[j*2] << 8) | data[j*2+1];
            sum[j] += raw / gyro_scale;
        }
        
        if(i % 200 == 0) {
            LOG_INFO << "[MPU9250] Progress: " << (i * 100 / samples) << "%" ;
        }
        usleep(5000);
    }
    
    for(int i = 0; i < 3; i++) {
        gyro_offset[i] = sum[i] / samples;
    }
    
    LOG_INFO << "[MPU9250] Calibration complete!" ;
    LOG_INFO << "[MPU9250]  Gyroscope Offset: [" << gyro_offset[0] << ", " 
              << gyro_offset[1] << ", " << gyro_offset[2] << "]" ;
}

/**
 * @brief Hiệu chỉnh gia tốc kế bằng cách lấy trung bình các giá trị đọc khi đặt trên mặt phẳng.
 * @param samples Số lượng mẫu để lấy trung bình.
 */
void AGVNavigation::calibrateAccel(int samples) {
    LOG_INFO << "[MPU9250] Calibrating accelerometer..." 
             << "  PLACE THE AGV ON A FLAT SURFACE!" ;
    sleep(2);
    
    float sum[3] = {0, 0, 0};
    
    for(int i = 0; i < samples; i++) {
        float ax, ay, az;
        readAccel(ax, ay, az);
        sum[0] += ax;
        sum[1] += ay;
        sum[2] += az;
        usleep(5000);
    }
    
    accel_offset[0] = sum[0] / samples;
    accel_offset[1] = sum[1] / samples;
    accel_offset[2] = (sum[2] / samples) - 1.0f;  // Subtract 1g
    
    LOG_INFO << "[MPU9250] Accelerometer calibration complete!" ;
    LOG_INFO << "[MPU9250]  Accelerometer Offset: [" << accel_offset[0] << ", "
              << accel_offset[1] << ", " << accel_offset[2] << "]" ;
}

/**
 * @brief Đọc tất cả các cảm biến, cập nhật bộ lọc và thực hiện kiểm tra an toàn.
 * Hàm này nên được gọi trong một vòng lặp tần số cao.
 */
void AGVNavigation::update() {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    
    readAccel(ax, ay, az);
    readGyro(gx, gy, gz);
    bool mag_ok = readMag(mx, my, mz);
    temperature = readTemp();
    
    // Store raw data
    raw_ax = ax; raw_ay = ay; raw_az = az;
    raw_gx = gx; raw_gy = gy; raw_gz = gz;
    raw_mx = mx; raw_my = my; raw_mz = mz;
    
    std::lock_guard<std::mutex> lock(data_mutex);
    
    // Update Madgwick filter
    if(mag_ok) {
        madgwick.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    } else {
        madgwick.updateIMU(gx, gy, gz, ax, ay, az);
    }
    
    madgwick.getEulerAngles(roll, pitch, heading);
    
    // Safety checks
    checkTiltSafety();
    detectCollision(ax, ay, az);
}

/** @brief Lấy dữ liệu định hướng đã được tổng hợp. */
void AGVNavigation::getOrientation(float &h, float &r, float &p) {
    std::lock_guard<std::mutex> lock(data_mutex);
    h = heading;
    r = roll;
    p = pitch;
}

/** @brief Lấy góc hướng hiện tại (yaw). */
float AGVNavigation::getHeading() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return heading;
}

/** @brief Lấy góc cuộn hiện tại (roll). */
float AGVNavigation::getRoll() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return roll;
}

/** @brief Lấy góc nghiêng hiện tại (pitch). */
float AGVNavigation::getPitch() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return pitch;
}

/** @brief Lấy dữ liệu thô mới nhất từ gia tốc kế. */
void AGVNavigation::getRawAccel(float &ax, float &ay, float &az) {
    std::lock_guard<std::mutex> lock(data_mutex);
    ax = raw_ax;
    ay = raw_ay;
    az = raw_az;
}

/** @brief Lấy dữ liệu thô mới nhất từ con quay hồi chuyển. */
void AGVNavigation::getRawGyro(float &gx, float &gy, float &gz) {
    std::lock_guard<std::mutex> lock(data_mutex);
    gx = raw_gx;
    gy = raw_gy;
    gz = raw_gz;
}

/** @brief Lấy dữ liệu thô mới nhất từ từ kế. */
void AGVNavigation::getRawMag(float &mx, float &my, float &mz) {
    std::lock_guard<std::mutex> lock(data_mutex);
    mx = raw_mx;
    my = raw_my;
    mz = raw_mz;
}

/** @brief Lấy nhiệt độ bên trong của cảm biến. */
float AGVNavigation::getTemperature() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return temperature;
}

/** @brief Kiểm tra xem AGV có nằm trong góc nghiêng an toàn không. */
bool AGVNavigation::isSafe() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return (fabs(roll) < max_tilt_angle && fabs(pitch) < max_tilt_angle);
}

/** @brief Kiểm tra xem AGV có bị nghiêng quá ngưỡng cố định (15 độ) không. */
bool AGVNavigation::isTilted() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return (fabs(roll) > 15.0f || fabs(pitch) > 15.0f);
}

/** @brief Lấy chuỗi hướng la bàn dựa trên góc hướng hiện tại. */
const char* AGVNavigation::getCompassDirection() {
    float h = getHeading();
    if(h >= 337.5f || h < 22.5f) return "North (N)";
    if(h >= 22.5f && h < 67.5f) return "Northeast (NE)";
    if(h >= 67.5f && h < 112.5f) return "East (E)";
    if(h >= 112.5f && h < 157.5f) return "Southeast (SE)";
    if(h >= 157.5f && h < 202.5f) return "South (S)";
    if(h >= 202.5f && h < 247.5f) return "Southwest (SW)";
    if(h >= 247.5f && h < 292.5f) return "West (W)";
    return "Northwest (NW)";
}

/** @brief Đặt lại ước tính vị trí bằng phương pháp dead reckoning. */
void AGVNavigation::resetPosition() {
    std::lock_guard<std::mutex> lock(data_mutex);
    pos_x = 0;
    pos_y = 0;
    velocity = 0;
}

/** @brief Đặt góc nghiêng an toàn tối đa. */
void AGVNavigation::setMaxTiltAngle(float angle) {
    max_tilt_angle = angle;
}

/** @brief Đặt ngưỡng gia tốc để phát hiện va chạm. */
void AGVNavigation::setCollisionThreshold(float threshold) {
    collision_threshold = threshold;
}

void AGVNavigation::setFilterBeta(float beta) {
    madgwick.setBeta(beta);
}

// =================================================================================
// Private Methods
// =================================================================================

/**
 * @brief Đọc dữ liệu thô từ gia tốc kế và chuyển đổi sang đơn vị g.
 * @param[out] ax Gia tốc đã hiệu chỉnh trên trục x.
 */
void AGVNavigation::readAccel(float &ax, float &ay, float &az) {
    uint8_t data[6];
    readBytes(ACCEL_XOUT_H, data, 6);
    
    int16_t raw_x = (data[0] << 8) | data[1];
    int16_t raw_y = (data[2] << 8) | data[3];
    int16_t raw_z = (data[4] << 8) | data[5];
    
    ax = (raw_x / accel_scale) - accel_offset[0];
    ay = (raw_y / accel_scale) - accel_offset[1];
    az = (raw_z / accel_scale) - accel_offset[2];
}

/**
 * @brief Đọc dữ liệu thô từ con quay hồi chuyển và chuyển đổi sang độ/giây.
 * @param[out] gx Tốc độ quay đã hiệu chỉnh trên trục x.
 */
void AGVNavigation::readGyro(float &gx, float &gy, float &gz) {
    uint8_t data[6];
    readBytes(GYRO_XOUT_H, data, 6);
    
    int16_t raw_x = (data[0] << 8) | data[1];
    int16_t raw_y = (data[2] << 8) | data[3];
    int16_t raw_z = (data[4] << 8) | data[5];
    
    gx = (raw_x / gyro_scale) - gyro_offset[0];
    gy = (raw_y / gyro_scale) - gyro_offset[1];
    gz = (raw_z / gyro_scale) - gyro_offset[2];
}

/**
 * @brief Đọc dữ liệu thô từ từ kế và chuyển đổi sang milligauss.
 * @param[out] mx Từ trường đã hiệu chỉnh trên trục x.
 */
bool AGVNavigation::readMag(float &mx, float &my, float &mz) {
    if(ioctl(i2c_fd, I2C_SLAVE, AK8963_ADDR) < 0) {
        return false;
    }
    
    uint8_t st1 = readByte(MAG_ST1);
    if(!(st1 & 0x01)) {
        ioctl(i2c_fd, I2C_SLAVE, MPU9250_ADDR);
        return false;
    }
    
    uint8_t data[7];
    readBytes(MAG_XOUT_L, data, 7);
    
    uint8_t st2 = data[6];
    if(st2 & 0x08) {
        ioctl(i2c_fd, I2C_SLAVE, MPU9250_ADDR);
        return false;
    }
    
    int16_t raw_x = (data[1] << 8) | data[0];
    int16_t raw_y = (data[3] << 8) | data[2];
    int16_t raw_z = (data[5] << 8) | data[4];
    
    mx = raw_x * mag_sensitivity[0] * 0.15f;
    my = raw_y * mag_sensitivity[1] * 0.15f;
    mz = raw_z * mag_sensitivity[2] * 0.15f;
    
    ioctl(i2c_fd, I2C_SLAVE, MPU9250_ADDR);
    return true;
}

/**
 * @brief Đọc cảm biến nhiệt độ bên trong.
 */
float AGVNavigation::readTemp() {
    uint8_t data[2];
    readBytes(TEMP_OUT_H, data, 2);
    int16_t raw_temp = (data[0] << 8) | data[1];
    return (raw_temp / 333.87f) + 21.0f;
}

/**
 * @brief Kiểm tra góc nghiêng có vượt giới hạn an toàn không
 * 
 * CẤU HÌNH: Y lên, Z trước, X trái
 * - Roll (quanh Z):  Nghiêng TRÁI/PHẢI
 * - Pitch (quanh X): Nghiêng TRƯỚC/SAU
 */
void AGVNavigation::checkTiltSafety() {
    if(fabs(roll) > max_tilt_angle || fabs(pitch) > max_tilt_angle) {
        LOG_WARNING << "[MPU9250] ️  CẢNH BÁO NGHIÊNG: " 
                    << "Roll=" << std::fixed << std::setprecision(1) << roll << "° "
                    << "(trái/phải), "
                    << "Pitch=" << pitch << "° "
                    << "(trước/sau)";
        
        // Roll: nghiêng trái/phải (xoay quanh trục Z - trục trước)
        if (fabs(roll) > max_tilt_angle) {
            if (roll > 0) {
                LOG_WARNING << "[MPU9250]  AGV nghiêng sang TRÁI quá mức!";
            } else {
                LOG_WARNING << "[MPU9250]  AGV nghiêng sang PHẢI quá mức!";
            }
        }
        
        // Pitch: nghiêng trước/sau (xoay quanh trục X - trục trái)
        if (fabs(pitch) > max_tilt_angle) {
            if (pitch > 0) {
                LOG_WARNING << "[MPU9250] AGV nghiêng về phía TRƯỚC quá mức!";
            } else {
                LOG_WARNING << "[MPU9250] AGV nghiêng về phía SAU quá mức!";
            }
        }
    }
}

/**
 * @brief Phát hiện va chạm tiềm tàng dựa trên độ lớn của gia tốc.
 */
void AGVNavigation::detectCollision(float ax, float ay, float az) {
    float accel_magnitude = sqrt(ax*ax + ay*ay + az*az);
    if(accel_magnitude > collision_threshold) {
                LOG_WARNING << "[MPU9250]  WARNING: COLLISION DETECTED! Accel magnitude: " << accel_magnitude << "g" ;
    }
}

/**
 * @brief Lấy dấu thời gian hiện tại của hệ thống theo micro giây.
 */
unsigned long AGVNavigation::getTimestamp() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

/**
 * @brief Ghi một byte đơn vào một thanh ghi I2C.
 */
void AGVNavigation::writeByte(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    if(write(i2c_fd, data, 2) != 2) {
        LOG_WARNING << "[MPU9250] I2C register 0x write error" << std::hex << (int)reg << std::dec ;
    }
}

/**
 * @brief Đọc một byte đơn từ một thanh ghi I2C.
 */
uint8_t AGVNavigation::readByte(uint8_t reg) {
    if(write(i2c_fd, &reg, 1) != 1) {
        LOG_WARNING << "[MPU9250] Error writing register address" ;
        return 0;
    }
    uint8_t value;
    if(read(i2c_fd, &value, 1) != 1) {
        LOG_WARNING << "[MPU9250] I2C read error" ;
        return 0;
    }
    return value;
}

/**
 * @brief Đọc nhiều byte từ các thanh ghi I2C liên tiếp.
 */
void AGVNavigation::readBytes(uint8_t reg, uint8_t *buffer, int length) {
    if(write(i2c_fd, &reg, 1) != 1) {
        LOG_WARNING << "[MPU9250] Error writing register address" ;
        return;
    }
    if(read(i2c_fd, buffer, length) != length) {
        LOG_WARNING << "[MPU9250] Error reading multiple bytes from I2C" ;
    }
}