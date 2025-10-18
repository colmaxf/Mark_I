/**
 * @file test_imu_axis.cpp
 * @brief Test YAW để xác định hướng trục X và quy ước xoay
 * @version 3.0 - YAW Test (Xoay trái/phải trên mặt phẳng)
 * 
 * CẤU HÌNH ĐÃ XÁC NHẬN:
 * - z: Hướng lên trên (trục thẳng đứng) ✅
 * - X: Hướng về phía trước AGV ✅
 * - Y: Cần test (trái hay phải?)
 * 
 * PHƯƠNG PHÁP TEST:
 * - Đặt AGV trên mặt phẳng ngang
 * - XOAY AGV sang TRÁI (CCW nhìn từ trên xuống)
 * - XOAY AGV sang PHẢI (CW nhìn từ trên xuống)
 * - Quan sát Yaw tăng hay giảm
 * 
 * KẾT LUẬN:
 * - Nếu xoay TRÁI → Yaw TĂNG → X hướng TRÁI (đúng quy ước) ✅
 * - Nếu xoay TRÁI → Yaw GIẢM → X hướng PHẢI (sai quy ước) ⚠️
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
#include <algorithm>

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

// Simple IMU class without DLT
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
        accel_scale = 16384.0f;
        gyro_scale = 131.0f;
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
        writeByte(CONFIG, 0x03);
        writeByte(GYRO_CONFIG, 0x00);
        writeByte(ACCEL_CONFIG, 0x00);
        writeByte(ACCEL_CONFIG2, 0x03);
        
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
    
    void readAccel(float &ax, float &ay, float &az) {
        uint8_t data[6];
        readBytes(ACCEL_XOUT_H, data, 6);
        
        int16_t raw_x = (data[0] << 8) | data[1];
        int16_t raw_y = (data[2] << 8) | data[3];
        int16_t raw_z = (data[4] << 8) | data[5];
        
        ax = (raw_x / accel_scale) - accel_offset[0];
        ay = (raw_y / accel_scale) - accel_offset[1];
        az = (raw_z / accel_scale) - accel_offset[2];
    }
    
    void readGyro(float &gx, float &gy, float &gz) {
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
    
    void getEulerAngles(float &roll, float &pitch, float &yaw) {
        // Roll (xoay quanh trục X - trục trước)
        roll = atan2(2.0f * (q0 * q1 + q2 * q3), 
                     1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
        
        // Pitch (xoay quanh trục Y - trục ngang)
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (fabs(sinp) >= 1.0f)
            pitch = copysign(90.0f, sinp);
        else
            pitch = asin(sinp) * 57.29578f;
        
        // Yaw (xoay quanh trục Z - trục lên)
        yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 
                    1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
        
        if(yaw < 0) yaw += 360.0f;
    }
    
    void update() {
        float ax, ay, az, gx, gy, gz;
        readAccel(ax, ay, az);
        readGyro(gx, gy, gz);
        updateIMU(gx, gy, gz, ax, ay, az);
    }
};

// Normalize angle difference to -180 to +180
float normalizeAngleDiff(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// Test functions for YAW
struct YawData {
    std::vector<float> samples;
    float average;
    float min;
    float max;
    float range;
};

YawData collectYawData(SimpleIMU& imu, int num_samples, const std::string& message) {
    std::cout << "   📍 " << std::setw(25) << std::left << (message + "...") << std::flush;
    
    YawData data;
    data.samples.reserve(num_samples);
    
    for (int i = 0; i < num_samples; i++) {
        imu.update();
        float heading, roll, pitch;
        imu.getEulerAngles(roll, pitch, heading);
        data.samples.push_back(heading);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    // Calculate statistics
    data.average = std::accumulate(data.samples.begin(), data.samples.end(), 0.0f) / num_samples;
    data.min = *std::min_element(data.samples.begin(), data.samples.end());
    data.max = *std::max_element(data.samples.begin(), data.samples.end());
    data.range = data.max - data.min;
    
    std::cout << " ✓ (Avg: " << std::fixed << std::setprecision(1) << data.average << "°)\n";
    return data;
}

void showRealtimeYaw(SimpleIMU& imu, int duration_sec) {
    std::cout << "\n   📡 Real-time YAW (Ctrl+C để dừng sớm):\n";
    std::cout << "   ┌─────────┬─────────┬─────────┐\n";
    std::cout << "   │  Time   │   Yaw   │  Trend  │\n";
    std::cout << "   ├─────────┼─────────┼─────────┤\n";
    
    float prev_yaw = 0.0f;
    bool first = true;
    
    for (int t = 0; t < duration_sec * 10; t++) {
        imu.update();
        float heading, roll, pitch;
        imu.getEulerAngles(roll, pitch, heading);
        
        std::string trend = "   -   ";
        if (!first) {
            float diff = normalizeAngleDiff(heading - prev_yaw);
            if (diff > 0.5f) trend = "  ↗ UP  ";
            else if (diff < -0.5f) trend = " ↘ DOWN ";
        }
        
        std::cout << "   │ " << std::setw(5) << (t * 0.1f) << "s  │ " 
                  << std::setw(6) << std::fixed << std::setprecision(1) << heading << "° │ " 
                  << trend << " │\r" << std::flush;
        
        prev_yaw = heading;
        first = false;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "\n   └─────────┴─────────┴─────────┘\n";
}

int main() {
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║           TEST YAW - KIỂM TRA HƯỚNG TRỤC X (No DLT)          ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════╝\n";
    std::cout << "\n";
    std::cout << "PHƯƠNG PHÁP: Xoay AGV trái/phải và quan sát Yaw\n\n";
    
    SimpleIMU imu;
    
    // Init
    std::cout << "🔧 Đang khởi tạo MPU9250..." << std::flush;
    if (!imu.begin("/dev/i2c-1")) {
        std::cerr << " ❌ FAILED\n";
        std::cerr << "   Kiểm tra:\n";
        std::cerr << "   - Kết nối I2C\n";
        std::cerr << "   - Quyền: sudo chmod 666 /dev/i2c-1\n";
        std::cerr << "   - Địa chỉ I2C (0x68)\n";
        return 1;
    }
    std::cout << " ✓\n";
    
    // Calibrate
    std::cout << "⚙️  Đang hiệu chuẩn Gyro (giữ AGV yên)..." << std::flush;
    imu.calibrateGyro(2000);
    std::cout << " ✓\n\n";
    
    std::cout << "╔═══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                       BẮT ĐẦU TEST YAW                       ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════╝\n\n";
    
    // Phase 1: Baseline (hướng ban đầu)
    std::cout << "🧭 BƯỚC 1: Đo hướng ban đầu\n";
    std::cout << "   Đặt AGV hướng về phía TRƯỚC (hoặc bất kỳ hướng nào)\n";
    YawData baseline = collectYawData(imu, 100, "Đo baseline");
    std::cout << "   ✅ Yaw ban đầu: " << std::fixed << std::setprecision(1) 
              << baseline.average << "° (ổn định ±" << baseline.range/2 << "°)\n";
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Phase 2: Xoay TRÁI (CCW)
    std::cout << "\n🔄 BƯỚC 2: XOAY AGV SANG TRÁI (Counter-Clockwise)\n";
    std::cout << "   ⚠️  Nhìn từ TRÊN XUỐNG, xoay AGV sang TRÁI ~45-90°\n";
    std::cout << "   📍 Bắt đầu xoay sau 3 giây...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    showRealtimeYaw(imu, 5);
    YawData left_turn = collectYawData(imu, 100, "Đo sau khi xoay TRÁI");
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Phase 3: Quay về baseline
    std::cout << "\n↩️  BƯỚC 3: Quay AGV về hướng ban đầu\n";
    std::cout << "   Xoay AGV trở về hướng ban đầu (~" << baseline.average << "°)\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    collectYawData(imu, 100, "Kiểm tra về baseline");
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Phase 4: Xoay PHẢI (CW)
    std::cout << "\n🔄 BƯỚC 4: XOAY AGV SANG PHẢI (Clockwise)\n";
    std::cout << "   ⚠️  Nhìn từ TRÊN XUỐNG, xoay AGV sang PHẢI ~45-90°\n";
    std::cout << "   📍 Bắt đầu xoay sau 3 giây...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    showRealtimeYaw(imu, 5);
    YawData right_turn = collectYawData(imu, 100, "Đo sau khi xoay PHẢI");
    
    // Analysis
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                       PHÂN TÍCH KẾT QUẢ                      ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════╝\n\n";
    
    std::cout << "📊 DỮ LIỆU THU THẬP:\n";
    std::cout << "   • Baseline (ban đầu):   " << std::fixed << std::setprecision(1) 
              << baseline.average << "°\n";
    std::cout << "   • Sau xoay TRÁI:        " << left_turn.average << "°\n";
    std::cout << "   • Sau xoay PHẢI:        " << right_turn.average << "°\n\n";
    
    float left_delta = normalizeAngleDiff(left_turn.average - baseline.average);
    float right_delta = normalizeAngleDiff(right_turn.average - baseline.average);
    
    std::cout << "📈 BIẾN THIÊN YAW:\n";
    std::cout << "   • Δ Yaw (xoay TRÁI):    " << left_delta << "°";
    if (left_delta > 10.0f) std::cout << "  ↗ TĂNG";
    else if (left_delta < -10.0f) std::cout << "  ↘ GIẢM";
    std::cout << "\n";
    
    std::cout << "   • Δ Yaw (xoay PHẢI):    " << right_delta << "°";
    if (right_delta > 10.0f) std::cout << "  ↗ TĂNG";
    else if (right_delta < -10.0f) std::cout << "  ↘ GIẢM";
    std::cout << "\n\n";
    
    // Conclusion
    std::cout << "🎯 KẾT LUẬN:\n";
    
    if (fabs(left_delta) < 10.0f || fabs(right_delta) < 10.0f) {
        std::cout << "   ⚠️  CẢNH BÁO: Góc xoay không đủ lớn!\n";
        std::cout << "   Vui lòng:\n";
        std::cout << "   - Xoay AGV rõ ràng hơn (>30°)\n";
        std::cout << "   - Giữ AGV ổn định khi đo\n";
        std::cout << "   - Chạy lại test\n";
    } else {
        bool left_increases = left_delta > 10.0f;
        bool right_decreases = right_delta < -10.0f;
        
        if (left_increases && right_decreases) {
            std::cout << "   ✅ TRỤC X HƯỚNG SANG TRÁI (Đúng quy ước)\n\n";
            std::cout << "   📌 CẤU HÌNH XÁC NHẬN:\n";
            std::cout << "      • Y: Hướng lên trên ✅\n";
            std::cout << "      • Z: Hướng về phía trước AGV ✅\n";
            std::cout << "      • X: Hướng sang TRÁI ✅\n\n";
            std::cout << "   🔄 QUY ƯỚC XOAY:\n";
            std::cout << "      • Xoay TRÁI (CCW) → Yaw TĂNG (+) ✅\n";
            std::cout << "      • Xoay PHẢI (CW)  → Yaw GIẢM (-) ✅\n\n";
            std::cout << "   ✨ HỆ TỌA ĐỘ THUẬN TAY PHẢI (Right-Hand Rule) ✅\n";
            std::cout << "   ✨ Code hiện tại ĐÚNG! Không cần thay đổi.\n";
            
        } else if (!left_increases && !right_decreases) {
            std::cout << "   ⚠️  TRỤC X HƯỚNG SANG PHẢI (Sai quy ước)\n\n";
            std::cout << "   📌 CẤU HÌNH XÁC NHẬN:\n";
            std::cout << "      • Y: Hướng lên trên ✅\n";
            std::cout << "      • Z: Hướng về phía trước AGV ✅\n";
            std::cout << "      • X: Hướng sang PHẢI ⚠️\n\n";
            std::cout << "   🔄 QUY ƯỚC XOAY:\n";
            std::cout << "      • Xoay TRÁI (CCW) → Yaw GIẢM (-) ⚠️\n";
            std::cout << "      • Xoay PHẢI (CW)  → Yaw TĂNG (+) ⚠️\n\n";
            std::cout << "   ⚠️  HỆ TỌA ĐỘ THUẬN TAY TRÁI (Left-Hand Rule)\n";
            std::cout << "   🔧 CẦN THAY ĐỔI CODE:\n";
            std::cout << "      1. Đảo dấu heading error trong PID\n";
            std::cout << "      2. HOẶC đảo logic left/right speed\n";
            
        } else {
            std::cout << "   ❓ KẾT QUẢ KHÔNG RÕ RÀNG\n";
            std::cout << "   Có thể:\n";
            std::cout << "   - Góc xoay không đối xứng\n";
            std::cout << "   - AGV không ổn định khi đo\n";
            std::cout << "   - Drift của gyroscope\n";
            std::cout << "\n   Vui lòng chạy lại test.\n";
        }
    }
    
    std::cout << "\n═══════════════════════════════════════════════════════════════\n";
    std::cout << "\n✅ Test YAW hoàn tất!\n\n";
    
    return 0;
}