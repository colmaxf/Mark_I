/*
 * MPU9250_AGV.h
 * AGV Navigation System Header File
 * For Raspberry Pi with MPU9250 IMU Sensor
 */
#ifndef MPU9250_AGV_H
#define MPU9250_AGV_H

#include <cstdint>
#include <mutex>
#include <string>
#include "../logger/Logger.h"

// MPU9250 Register Addresses
#define MPU9250_ADDR 0x68
#define AK8963_ADDR 0x0C
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define INT_PIN_CFG 0x37
#define WHO_AM_I 0x75
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define TEMP_OUT_H 0x41
#define MAG_WHO_AM_I 0x00
#define MAG_ST1 0x02
#define MAG_XOUT_L 0x03
#define MAG_ST2 0x09
#define MAG_CNTL1 0x0A
#define MAG_CNTL2 0x0B
#define MAG_ASAX 0x10

/**
 * @class MadgwickFilter
 * @brief Triển khai thuật toán Madgwick AHRS để tổng hợp dữ liệu cảm biến.
 *
 * Lớp này tổng hợp dữ liệu từ gia tốc kế, con quay hồi chuyển và từ kế để cung cấp
 * một ước tính định hướng ổn định dưới dạng quaternion. Nó cũng hỗ trợ chế độ
 * chỉ có IMU (6 bậc tự do) khi không có dữ liệu từ kế.
 */
class MadgwickFilter {
private:
    float beta;           ///< Hệ số khuếch đại của bộ lọc, điều khiển ảnh hưởng của gia tốc kế/từ kế so với con quay hồi chuyển.
    float q0, q1, q2, q3; ///< Các thành phần quaternion biểu diễn định hướng (w, x, y, z).
    float invSampleFreq;  ///< Nghịch đảo của tần số lấy mẫu (tính bằng giây).

    /**
     * @brief Thuật toán căn bậc hai nghịch đảo nhanh.
     * @param x Số cần tính căn bậc hai nghịch đảo.
     * @return Căn bậc hai nghịch đảo của x.
     */
    float invSqrt(float x);

public:
    /**
     * @brief Hàm khởi tạo đối tượng MadgwickFilter.
     * @param sampleFreq Tần số lấy mẫu (Hz).
     * @param beta Hệ số khuếch đại của bộ lọc.
     */
    MadgwickFilter(float sampleFreq = 100.0f, float beta = 0.1f);

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
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz);

    /**
     * @brief Cập nhật bộ lọc với dữ liệu 6 bậc tự do (chỉ con quay và gia tốc kế).
     * @param gx Tốc độ quay trục x của con quay hồi chuyển (độ/giây).
     * @param gy Tốc độ quay trục y của con quay hồi chuyển (độ/giây).
     * @param gz Tốc độ quay trục z của con quay hồi chuyển (độ/giây).
     * @param ax Gia tốc trục x của gia tốc kế (g).
     * @param ay Gia tốc trục y của gia tốc kế (g).
     * @param az Gia tốc trục z của gia tốc kế (g).
     */
    void updateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az);

    /**
     * @brief Lấy các góc Euler đã được tính toán.
     * @param[out] roll Góc cuộn (roll) theo độ.
     * @param[out] pitch Góc nghiêng (pitch) theo độ.
     * @param[out] yaw Góc lệch (yaw/heading) theo độ (0-360).
     */
    void getEulerAngles(float &roll, float &pitch, float &yaw);

    /**
     * @brief Lấy góc Yaw thô (chưa chuẩn hóa) để debug.
     * @return Góc yaw theo độ, có thể âm.
     */
    float getRawYaw();


    /**
     * @brief Lấy định hướng hiện tại dưới dạng quaternion.
     * @param[out] w Thành phần w của quaternion.
     * @param[out] x Thành phần x của quaternion.
     * @param[out] y Thành phần y của quaternion.
     * @param[out] z Thành phần z của quaternion.
     */
    void getQuaternion(float &w, float &x, float &y, float &z);

    /**
     * @brief Đặt hệ số khuếch đại của bộ lọc (beta).
     * @param beta Giá trị beta mới.
     */
    void setBeta(float beta);

    /**
     * @brief Lấy hệ số khuếch đại hiện tại của bộ lọc (beta).
     * @return Giá trị beta hiện tại.
     */
    float getBeta() const;
};

/**
 * @class AGVNavigation
 * @brief Quản lý cảm biến IMU MPU9250 cho việc điều hướng và an toàn của AGV.
 *
 * Lớp này xử lý giao tiếp I2C, hiệu chỉnh cảm biến, thu thập dữ liệu,
 * tổng hợp cảm biến bằng bộ lọc Madgwick, và cung cấp dữ liệu cấp cao như
 * định hướng (hướng, cuộn, nghiêng) và kiểm tra an toàn (độ nghiêng, va chạm).
 */
class AGVNavigation {
private:
    int i2c_fd; ///< Định danh file I2C để giao tiếp.

    // Sensor configuration
    float accel_scale;        ///< Hệ số tỷ lệ để chuyển đổi dữ liệu thô của gia tốc kế sang g.
    float gyro_scale;         ///< Hệ số tỷ lệ để chuyển đổi dữ liệu thô của con quay hồi chuyển sang độ/giây.
    float mag_sensitivity[3]; ///< Giá trị điều chỉnh độ nhạy của từ kế từ Fuse ROM.

    // Calibration data
    float accel_offset[3]; ///< Độ lệch hiệu chỉnh của gia tốc kế (x, y, z).
    float gyro_offset[3];  ///< Độ lệch hiệu chỉnh của con quay hồi chuyển (x, y, z).

    MadgwickFilter madgwick; ///< Thể hiện của bộ lọc Madgwick để tổng hợp cảm biến.

    std::mutex data_mutex; ///< Mutex để truy cập dữ liệu chia sẻ một cách an toàn luồng.

    // AGV State Data
    float heading;  ///< Góc lệch (yaw) đã tổng hợp (0-360 độ).
    float roll;     ///< Góc cuộn (roll) đã tổng hợp theo độ.
    float pitch;    ///< Góc nghiêng (pitch) đã tổng hợp theo độ.
    float pos_x;    ///< Vị trí X ước tính qua dead reckoning.
    float pos_y;    ///< Vị trí Y ước tính qua dead reckoning (m).
    float velocity_x; ///< Vận tốc X ước tính trong hệ tọa độ thế giới (m/s).
    float velocity_y; ///< Vận tốc Y ước tính trong hệ tọa độ thế giới (m/s).
    
    unsigned long last_update_time; ///< Dấu thời gian của lần cập nhật cuối cùng (us).

    // Raw Sensor Data
    float raw_ax, raw_ay, raw_az; ///< Dữ liệu thô của gia tốc kế (g).
    float raw_gx, raw_gy, raw_gz; ///< Dữ liệu thô của con quay hồi chuyển (dps).
    float raw_mx, raw_my, raw_mz; ///< Dữ liệu thô của từ kế (mG).
    float temperature;            ///< Nhiệt độ cảm biến theo độ C.

    // Safety Parameters
    float max_tilt_angle;      ///< Góc nghiêng tối đa cho phép (độ) trước khi có cảnh báo.
    float collision_threshold; ///< Độ lớn gia tốc (g) để kích hoạt cảnh báo phát hiện va chạm.

    // --- Private Methods ---
    void readAccel(float &ax, float &ay, float &az);
    void readGyro(float &gx, float &gy, float &gz);
    bool readMag(float &mx, float &my, float &mz);
    float readTemp();
    void updateDeadReckoning(float ax, float ay, float az, float dt);

    void checkTiltSafety();
    void detectCollision(float ax, float ay, float az);
    unsigned long getTimestamp();

    void writeByte(uint8_t reg, uint8_t value);
    uint8_t readByte(uint8_t reg);
    void readBytes(uint8_t reg, uint8_t *buffer, int length);

public:
    /** @brief Hàm khởi tạo đối tượng AGVNavigation và các giá trị mặc định. */
    AGVNavigation();
    /** @brief Hàm hủy đối tượng AGVNavigation, đảm bảo file I2C được đóng. */
    ~AGVNavigation();

    /**
     * @brief Khởi tạo cảm biến MPU9250.
     * @param i2c_device Đường dẫn đến thiết bị I2C (ví dụ: "/dev/i2c-1").
     * @return True nếu khởi tạo thành công, ngược lại là false.
     */
    bool begin(const char* i2c_device = "/dev/i2c-1");

    /**
     * @brief Khởi tạo từ kế AK8963.
     * @return True nếu thành công, ngược lại là false.
     */
    bool initMagnetometer();

    /**
     * @brief Hiệu chỉnh con quay hồi chuyển bằng cách lấy trung bình các giá trị đọc khi đứng yên.
     * @param samples Số lượng mẫu để lấy trung bình.
     */
    void calibrateGyro(int samples = 2000);

    /**
     * @brief Hiệu chỉnh gia tốc kế bằng cách lấy trung bình các giá trị đọc khi đặt trên mặt phẳng.
     * @param samples Số lượng mẫu để lấy trung bình.
     */
    void calibrateAccel(int samples = 2000);

    /**
     * @brief Đọc tất cả các cảm biến, cập nhật bộ lọc và thực hiện kiểm tra an toàn.
     * Hàm này nên được gọi trong một vòng lặp tần số cao.
     */
    void update();

    /** @brief Lấy dữ liệu định hướng đã được tổng hợp. */
    void getOrientation(float &heading, float &roll, float &pitch);
    /** @brief Lấy góc hướng hiện tại (yaw). */
    float getHeading();
    /** @brief Lấy góc Yaw thô (chưa chuẩn hóa) để debug. */
    float getRawYaw();
    /** @brief Lấy góc cuộn hiện tại (roll). */
    float getRoll();
    /** @brief Lấy góc nghiêng hiện tại (pitch). */
    float getPitch();

    /** @brief Lấy vận tốc ước tính hiện tại. */
    void getVelocity(float &vx, float &vy);
    /** @brief Lấy vị trí ước tính hiện tại. */
    void getPosition(float &px, float &py);
    /** @brief Lấy dữ liệu thô mới nhất từ gia tốc kế. */
    void getRawAccel(float &ax, float &ay, float &az);
    /** @brief Lấy dữ liệu thô mới nhất từ con quay hồi chuyển. */
    void getRawGyro(float &gx, float &gy, float &gz);
    /** @brief Lấy dữ liệu thô mới nhất từ từ kế. */
    void getRawMag(float &mx, float &my, float &mz);
    /** @brief Lấy nhiệt độ bên trong của cảm biến. */
    float getTemperature();

    /** @brief Kiểm tra xem AGV có nằm trong góc nghiêng an toàn không. */
    bool isSafe();
    /** @brief Kiểm tra xem AGV có bị nghiêng quá ngưỡng cố định (15 độ) không. */
    bool isTilted();

    /** @brief Lấy chuỗi hướng la bàn dựa trên góc hướng hiện tại. */
    const char* getCompassDirection();
    /** @brief Đặt lại ước tính vị trí bằng phương pháp dead reckoning. */
    void resetPosition();

    /** @brief Đặt góc nghiêng an toàn tối đa. */
    void setMaxTiltAngle(float angle);
    /** @brief Đặt ngưỡng gia tốc để phát hiện va chạm. */
    void setCollisionThreshold(float threshold);
    /** @brief Đặt hệ số beta cho bộ lọc Madgwick. */
    void setFilterBeta(float beta);
};

#endif // MPU9250_AGV_H