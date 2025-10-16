/**
 * @file test_imu_axis_mapping.cpp
 * @brief Test độc lập để xác định axis mapping của MPU9250 (No Logger)
 * @version 2.0 - Standalone without Logger dependency
 * 
 * CÁCH DÙNG:
 * 1. Đặt AGV trên mặt phẳng ngang
 * 2. Compile: g++ -std=c++17 -Wall -O2 -pthread -o test_axis test_imu_axis_mapping.cpp -lm
 * 3. Chạy: sudo ./test_axis
 * 4. Đọc kết quả để biết trục nào đang sai
 */

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <numeric>

// MPU9250 Registers
#define MPU9250_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define WHO_AM_I 0x75
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// Simple IMU class without Logger
class SimpleIMU {
private:
    int i2c_fd;
    float accel_scale;
    float gyro_scale;
    float gyro_offset[3];
    float accel_offset[3];
    
    // Madgwick filter
    float beta;
    float q0, q1, q2, q3;
    float invSampleFreq;
    
    void writeByte(uint8_t reg, uint8_t value) {
        uint8_t data[2] = {reg, value};
        write(i2c_fd, data, 2);
    }
    
    uint8_t readByte(uint8_t reg) {
        write(i2c_fd, &reg, 1);
        uint8_t value;
        read(i2c_fd, &value, 1);
        return value;
    }
    
    void readBytes(uint8_t reg, uint8_t *buffer, int length) {
        write(i2c_fd, &reg, 1);
        read(i2c_fd, buffer, length);
    }
    
    float invSqrt(float x) {
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
    
public:
    SimpleIMU() : i2c_fd(-1), beta(0.08f), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {
        accel_scale = 16384.0f;  // ±2g
        gyro_scale = 131.0f;     // ±250°/s
        invSampleFreq = 1.0f / 100.0f;
        for(int i = 0; i < 3; i++) {
            gyro_offset[i] = 0.0f;
            accel_offset[i] = 0.0f;
        }
    }
    
    ~SimpleIMU() {
        if(i2c_fd >= 0) close(i2c_fd);
    }
    
    bool begin(const char* device = "/dev/i2c-1") {
        i2c_fd = open(device, O_RDWR);
        if(i2c_fd < 0) return false;
        
        if(ioctl(i2c_fd, I2C_SLAVE, MPU9250_ADDR) < 0) return false;
        
        uint8_t who = readByte(WHO_AM_I);
        if(who != 0x71 && who != 0x73) {
            close(i2c_fd);
            i2c_fd = -1;
            return false;
        }
        
        // Reset
        writeByte(PWR_MGMT_1, 0x80);
        usleep(100000);
        
        // Wake up
        writeByte(PWR_MGMT_1, 0x01);
        usleep(200000);
        
        // Enable all sensors
        writeByte(PWR_MGMT_2, 0x00);
        usleep(100000);
        
        // Configure
        writeByte(CONFIG, 0x03);           // DLPF 41Hz
        writeByte(GYRO_CONFIG, 0x00);      // ±250°/s
        writeByte(ACCEL_CONFIG, 0x00);     // ±2g
        writeByte(ACCEL_CONFIG2, 0x03);    // DLPF 41Hz
        
        return true;
    }
    
    void calibrateGyro(int samples = 2000) {
        float sum[3] = {0, 0, 0};
        
        for(int i = 0; i < samples; i++) {
            uint8_t data[6];
            readBytes(GYRO_XOUT_H, data, 6);
            
            for(int j = 0; j < 3; j++) {
                int16_t raw = (data[j*2] << 8) | data[j*2+1];
                sum[j] += raw / gyro_scale;
            }
            usleep(5000);
        }
        
        for(int i = 0; i < 3; i++) {
            gyro_offset[i] = sum[i] / samples;
        }
    }
    
    void getRawAccel(float &ax, float &ay, float &az) {
        uint8_t data[6];
        readBytes(ACCEL_XOUT_H, data, 6);
        
        int16_t raw_x = (data[0] << 8) | data[1];
        int16_t raw_y = (data[2] << 8) | data[3];
        int16_t raw_z = (data[4] << 8) | data[5];
        
        ax = (raw_x / accel_scale) - accel_offset[0];
        ay = (raw_y / accel_scale) - accel_offset[1];
        az = (raw_z / accel_scale) - accel_offset[2];
    }
    
    void getRawGyro(float &gx, float &gy, float &gz) {
        uint8_t data[6];
        readBytes(GYRO_XOUT_H, data, 6);
        
        int16_t raw_x = (data[0] << 8) | data[1];
        int16_t raw_y = (data[2] << 8) | data[3];
        int16_t raw_z = (data[4] << 8) | data[5];
        
        gx = (raw_x / gyro_scale) - gyro_offset[0];
        gy = (raw_y / gyro_scale) - gyro_offset[1];
        gz = (raw_z / gyro_scale) - gyro_offset[2];
    }
    
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
        float q0q0, q1q1, q2q2, q3q3;
        
        // Convert gyroscope degrees/sec to radians/sec
        gx *= 0.0174533f;
        gy *= 0.0174533f;
        gz *= 0.0174533f;
        
        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
        
        // Compute feedback only if accelerometer measurement valid
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;
            
            // Auxiliary variables
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
            
            // Gradient descent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            
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
        
        // Integrate rate of change of quaternion
        q0 += qDot1 * invSampleFreq;
        q1 += qDot2 * invSampleFreq;
        q2 += qDot3 * invSampleFreq;
        q3 += qDot4 * invSampleFreq;
        
        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }
    
    void getOrientation(float &heading, float &roll, float &pitch) {
        // Roll (rotation around X-axis)
        roll = atan2(2.0f * (q0 * q3 + q1 * q2), 
                     1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
        
        // Pitch (rotation around Y-axis)
        float sinp = 2.0f * (q0 * q1 - q2 * q3);
        if (fabs(sinp) >= 1.0f)
            pitch = copysign(90.0f, sinp);
        else
            pitch = asin(sinp) * 57.29578f;
        
        // Yaw (rotation around Z-axis)
        heading = atan2(2.0f * (q0 * q2 + q1 * q3), 
                        1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
        
        if(heading < 0) heading += 360.0f;
    }
    
    std::string getCompassDirection() {
        float heading, roll, pitch;
        getOrientation(heading, roll, pitch);
        
        const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
        int idx = (int)((heading + 22.5f) / 45.0f) % 8;
        return directions[idx];
    }
    
    void update() {
        float ax, ay, az, gx, gy, gz;
        getRawAccel(ax, ay, az);
        getRawGyro(gx, gy, gz);
        updateIMU(gx, gy, gz, ax, ay, az);
    }
};

void printHeader(const std::string& title) {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║ " << std::left << std::setw(62) << title << " ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n";
    std::cout << "\n";
}

void printSection(const std::string& title) {
    std::cout << "\n";
    std::cout << "┌────────────────────────────────────────────────────────────────┐\n";
    std::cout << "│ " << std::left << std::setw(62) << title << " │\n";
    std::cout << "└────────────────────────────────────────────────────────────────┘\n";
}

int main() {
    printHeader("MPU9250 AXIS MAPPING TEST");
    
    std::cout << "📋 HƯỚNG DẪN:\n";
    std::cout << "  1. Đặt AGV trên mặt phẳng ngang\n";
    std::cout << "  2. Giữ AGV hoàn toàn đứng yên\n";
    std::cout << "  3. Chờ đo trong 5 giây\n";
    std::cout << "\n";
    std::cout << "Press Enter to start...";
    std::cin.get();
    
    // ========================================================================
    // KHỞI TẠO IMU
    // ========================================================================
    SimpleIMU imu;
    
    std::cout << "\n🔧 Initializing MPU9250..." << std::flush;
    if (!imu.begin("/dev/i2c-1")) {
        std::cerr << " ❌ FAILED\n";
        std::cerr << "\n";
        std::cerr << "Possible issues:\n";
        std::cerr << "  - I2C not enabled: sudo raspi-config → Interface → I2C\n";
        std::cerr << "  - Permission denied: sudo chmod 666 /dev/i2c-1\n";
        std::cerr << "  - Wrong address: Check if MPU9250 is at 0x68 or 0x69\n";
        std::cerr << "  - Connection issue: Check wiring\n";
        return 1;
    }
    std::cout << " ✅ OK\n";
    
    // ========================================================================
    // CALIBRATE GYROSCOPE
    // ========================================================================
    std::cout << "\n⚙️  Calibrating Gyroscope (keep AGV still)..." << std::flush;
    imu.calibrateGyro(2000);
    std::cout << " ✅ OK\n";
    
    // ========================================================================
    // ĐO DỮ LIỆU
    // ========================================================================
    printSection("MEASURING DATA (5 seconds)...");
    
    std::cout << "Keep AGV still on flat surface...\n";
    std::cout << "Progress: [";
    
    const int total_samples = 500;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_roll = 0, sum_pitch = 0, sum_yaw = 0;
    
    for (int i = 0; i < total_samples; i++) {
        // Update IMU
        imu.update();
        
        // Read raw accel
        float ax, ay, az;
        imu.getRawAccel(ax, ay, az);
        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        
        // Read raw gyro
        float gx, gy, gz;
        imu.getRawGyro(gx, gy, gz);
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        
        // Read orientation
        float heading, roll, pitch;
        imu.getOrientation(heading, roll, pitch);
        sum_roll += roll;
        sum_pitch += pitch;
        sum_yaw += heading;
        
        // Progress bar
        if (i % 10 == 0) {
            std::cout << "=" << std::flush;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "] Done\n";
    
    // Tính trung bình
    float avg_ax = sum_ax / total_samples;
    float avg_ay = sum_ay / total_samples;
    float avg_az = sum_az / total_samples;
    float avg_gx = sum_gx / total_samples;
    float avg_gy = sum_gy / total_samples;
    float avg_gz = sum_gz / total_samples;
    float avg_roll = sum_roll / total_samples;
    float avg_pitch = sum_pitch / total_samples;
    float avg_yaw = sum_yaw / total_samples;
    
    // ========================================================================
    // HIỂN THỊ KẾT QUẢ
    // ========================================================================
    printHeader("TEST RESULTS");
    
    // RAW ACCELEROMETER
    printSection("RAW ACCELEROMETER (When AGV is flat)");
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  ax = " << std::setw(7) << avg_ax << " g";
    if (fabs(avg_ax) > 0.8f) std::cout << "  ← THIS AXIS POINTS UP!";
    std::cout << "\n";
    std::cout << "  ay = " << std::setw(7) << avg_ay << " g";
    if (fabs(avg_ay) > 0.8f) std::cout << "  ← THIS AXIS POINTS UP!";
    std::cout << "\n";
    std::cout << "  az = " << std::setw(7) << avg_az << " g";
    if (fabs(avg_az) > 0.8f) std::cout << "  ← THIS AXIS POINTS UP!";
    std::cout << "\n";
    
    // RAW GYROSCOPE
    printSection("RAW GYROSCOPE (After calibration)");
    std::cout << "  gx = " << std::setw(7) << avg_gx << " °/s";
    if (fabs(avg_gx) > 1.0f) std::cout << "  ⚠️  High drift!";
    std::cout << "\n";
    std::cout << "  gy = " << std::setw(7) << avg_gy << " °/s";
    if (fabs(avg_gy) > 1.0f) std::cout << "  ⚠️  High drift!";
    std::cout << "\n";
    std::cout << "  gz = " << std::setw(7) << avg_gz << " °/s";
    if (fabs(avg_gz) > 1.0f) std::cout << "  ⚠️  High drift!";
    std::cout << "\n";
    
    // ORIENTATION
    printSection("COMPUTED ORIENTATION (From Madgwick filter)");
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  Roll  = " << std::setw(7) << avg_roll << " °";
    if (fabs(avg_roll) > 10.0f) {
        std::cout << "  ❌ Should be ~0° when flat!";
    } else if (fabs(avg_roll) > 5.0f) {
        std::cout << "  ⚠️  Slightly tilted";
    } else {
        std::cout << "  ✅ Good";
    }
    std::cout << "\n";
    
    std::cout << "  Pitch = " << std::setw(7) << avg_pitch << " °";
    if (fabs(avg_pitch) > 10.0f) {
        std::cout << "  ❌ Should be ~0° when flat!";
    } else if (fabs(avg_pitch) > 5.0f) {
        std::cout << "  ⚠️  Slightly tilted";
    } else {
        std::cout << "  ✅ Good";
    }
    std::cout << "\n";
    
    std::cout << "  Yaw   = " << std::setw(7) << avg_yaw << " °";
    std::cout << "  (Direction: " << imu.getCompassDirection() << ")";
    std::cout << "\n";
    
    // ========================================================================
    // PHÂN TÍCH VÀ CHẨN ĐOÁN
    // ========================================================================
    printHeader("DIAGNOSIS & RECOMMENDATIONS");
    
    // Xác định trục nào hướng lên
    float abs_ax = fabs(avg_ax);
    float abs_ay = fabs(avg_ay);
    float abs_az = fabs(avg_az);
    
    std::cout << "🔍 AXIS MAPPING CHECK:\n\n";
    
    if (abs_ay > abs_ax && abs_ay > abs_az && abs_ay > 0.8f) {
        std::cout << "  ✅ Y-axis points UP (CORRECT!)\n";
        std::cout << "     Current mapping: Y=UP, X=?, Z=?\n";
        std::cout << "\n";
    } else if (abs_ax > abs_ay && abs_ax > abs_az && abs_ax > 0.8f) {
        std::cout << "  ❌ X-axis points UP (WRONG!)\n";
        std::cout << "     Should be: Y=UP\n";
        std::cout << "     Current:   X=UP\n";
        std::cout << "\n";
        std::cout << "  🔧 FIX NEEDED:\n";
        std::cout << "     Need to SWAP X ↔ Y axes in getEulerAngles()\n";
        std::cout << "\n";
    } else if (abs_az > abs_ax && abs_az > abs_ay && abs_az > 0.8f) {
        std::cout << "  ❌ Z-axis points UP (WRONG!)\n";
        std::cout << "     Should be: Y=UP\n";
        std::cout << "     Current:   Z=UP\n";
        std::cout << "\n";
        std::cout << "  🔧 FIX NEEDED:\n";
        std::cout << "     Need to SWAP Z ↔ Y axes in getEulerAngles()\n";
        std::cout << "\n";
    } else {
        std::cout << "  ⚠️  NO clear UP axis detected!\n";
        std::cout << "     ax=" << avg_ax << "g, ay=" << avg_ay << "g, az=" << avg_az << "g\n";
        std::cout << "\n";
        std::cout << "  Possible issues:\n";
        std::cout << "     - AGV is not on a flat surface\n";
        std::cout << "     - IMU mounting is very tilted\n";
        std::cout << "     - Accelerometer needs calibration\n";
        std::cout << "\n";
    }
    
    // Kiểm tra Roll và Pitch
    std::cout << "🎯 ORIENTATION CHECK:\n\n";
    
    bool roll_ok = fabs(avg_roll) < 5.0f;
    bool pitch_ok = fabs(avg_pitch) < 5.0f;
    
    if (roll_ok && pitch_ok) {
        std::cout << "  ✅ Roll and Pitch are acceptable\n";
        std::cout << "     Roll="  << avg_roll << "°, Pitch=" << avg_pitch << "°\n";
        std::cout << "     → Orientation calculation is CORRECT\n";
        std::cout << "\n";
    } else {
        std::cout << "  ❌ Roll or Pitch is too large!\n";
        std::cout << "     Roll="  << avg_roll << "° (should be ~0°)\n";
        std::cout << "     Pitch=" << avg_pitch << "° (should be ~0°)\n";
        std::cout << "\n";
        std::cout << "  🔧 FIX OPTIONS:\n";
        std::cout << "     1. Check if AGV is truly on flat surface\n";
        std::cout << "     2. Remount IMU to be more level\n";
        std::cout << "     3. OR: Add accelerometer offset calibration\n";
        std::cout << "\n";
    }
    
    // Gyro drift check
    if (fabs(avg_gx) > 1.0f || fabs(avg_gy) > 1.0f || fabs(avg_gz) > 1.0f) {
        std::cout << "⚠️  GYROSCOPE DRIFT WARNING:\n\n";
        std::cout << "     High drift detected after calibration\n";
        std::cout << "     gx=" << avg_gx << "°/s, gy=" << avg_gy << "°/s, gz=" << avg_gz << "°/s\n";
        std::cout << "\n";
        std::cout << "  Recommendations:\n";
        std::cout << "     - Recalibrate with AGV completely still\n";
        std::cout << "     - Check for vibrations or movement\n";
        std::cout << "     - Temperature may affect drift\n";
        std::cout << "\n";
    }
    
    // ========================================================================
    // SUMMARY
    // ========================================================================
    printHeader("SUMMARY");
    
    int issues = 0;
    
    std::cout << "Status Summary:\n\n";
    
    if (abs_ay > 0.8f && abs_ay > abs_ax && abs_ay > abs_az) {
        std::cout << "  ✅ Axis mapping: CORRECT\n";
    } else {
        std::cout << "  ❌ Axis mapping: INCORRECT - needs code fix\n";
        issues++;
    }
    
    if (roll_ok && pitch_ok) {
        std::cout << "  ✅ Orientation:  GOOD\n";
    } else {
        std::cout << "  ⚠️  Orientation:  NEEDS CALIBRATION\n";
        issues++;
    }
    
    if (fabs(avg_gx) < 1.0f && fabs(avg_gy) < 1.0f && fabs(avg_gz) < 1.0f) {
        std::cout << "  ✅ Gyro drift:   LOW\n";
    } else {
        std::cout << "  ⚠️  Gyro drift:   HIGH - recalibrate recommended\n";
        issues++;
    }
    
    std::cout << "\n";
    
    if (issues == 0) {
        std::cout << "🎉 ALL CHECKS PASSED!\n";
        std::cout << "   IMU is configured correctly and ready to use.\n";
    } else {
        std::cout << "⚠️  " << issues << " ISSUE(S) FOUND\n";
        std::cout << "   Review the recommendations above.\n";
    }
    
    std::cout << "\n";
    std::cout << "═══════════════════════════════════════════════════════════════\n";
    std::cout << "\n";
    
    return 0;
}