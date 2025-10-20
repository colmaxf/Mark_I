// SystemManager.h
#ifndef SYSTEM_MANAGER_H
#define SYSTEM_MANAGER_H

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <chrono>
#include <boost/lockfree/spsc_queue.hpp>

#if TEST_KEYBOARD_MODE == 1
#include <linux/input.h>  // Cho input_event
#endif

#include "config.h"
#include "logger/Logger.h"
#include "MCprotocollib/MCprotocol.h"
#include "Lidarlib/Lidarlib.h"
#include "Serverlib/servercommunicator.h"
#include "Batterylib/BatteryJBD.h"
#include "IMUlib/MPU9250_AGV.h"

/**
 * @struct Point2D
 * @brief Cấu trúc dữ liệu đơn giản cho một điểm trong không gian 2D.
 * Thường được sử dụng để biểu diễn dữ liệu từ cảm biến LiDAR.
 */
// struct Point2D {
//     float x; ///< Tọa độ X của điểm.
//     float y; ///< Tọa độ Y của điểm.
// };

/**
 * @struct SystemState
 * @brief Cấu trúc dữ liệu trung tâm, chia sẻ trạng thái của toàn bộ hệ thống giữa các luồng.
 * @details Mọi truy cập vào các thành viên của cấu trúc này phải được bảo vệ bởi `state_mutex`
 *          để đảm bảo an toàn luồng (thread-safety).
 */
struct SystemState {
    std::mutex state_mutex; ///< Mutex để bảo vệ tất cả các thành viên trong struct này.

    // Dữ liệu Pin
    double battery_level = 0.0; ///< Mức pin hiện tại (%).
    bool battery_connected = false; ///< Trạng thái kết nối với BMS.

    // Dữ liệu PLC
    bool plc_connected = false; ///< Trạng thái kết nối với PLC.
    std::string last_plc_status = "Chưa có dữ liệu PLC"; ///< Trạng thái hoặc phản hồi cuối cùng từ PLC.

    // Dữ liệu Server - connect
    bool server_connected = false; ///< Trạng thái kết nối với server trung tâm.

    // Dữ liệu LiDAR
    std::string last_lidar_data = "Chưa có dữ liệu Lidar"; ///< Chuỗi trạng thái cuối cùng từ LiDAR.
    bool lidar_connected = false; ///< Trạng thái kết nối của LiDAR.
    std::vector<ServerComm::Point2D> latest_convex_hull; ///< Tập hợp các điểm ổn định gần nhất từ LiDAR.
    int total_stable_hulls = 0; ///< Tổng số lần dữ liệu LiDAR được xác định là ổn định.

    // Dữ liệu an toàn
    bool is_safe_to_move = false;           ///< Cờ an toàn từ LiDAR, `true` nếu không có vật cản trong vùng nguy hiểm.
    float current_front_distance = -1.0f;  ///< Khoảng cách gần nhất đến vật cản phía trước (cm).
    long last_safety_update = 0;           ///< Dấu thời gian của lần cập nhật trạng thái an toàn cuối cùng.

    // Dữ liệu điều khiển chuyển động
    bool is_moving = false; ///< Cờ cho biết AGV có đang trong quá trình di chuyển hay không.
    std::chrono::steady_clock::time_point movement_start_time; ///< Thời điểm bắt đầu di chuyển, dùng để tính toán gia tốc.
    int current_speed = 0; ///< Tốc độ hiện tại của AGV.
    int target_speed = 0; ///< Tốc độ mục tiêu mà AGV đang hướng tới.
    bool movement_command_active = false;  ///< Cờ này được set khi có lệnh di chuyển (tiến/lùi/xoay) và reset khi dừng.
    bool movement_pending = false; ///< Cờ báo hiệu một lệnh di chuyển mới vừa được đưa ra, đang chờ PLC thực thi.

    // Dữ liệu IMU
    bool imu_connected = false;
    float current_heading = 0.0f;      // Hướng hiện tại (0-360°)
    float current_roll = 0.0f;         // Góc nghiêng trái/phải
    float current_pitch = 0.0f;        // Góc nghiêng lên/xuống
    bool imu_tilt_warning = false;     // Cảnh báo nghiêng nguy hiểm
    std::string last_imu_data = "Chưa có dữ liệu IMU";
};

/**
 * @class SystemManager
 * @brief Lớp trung tâm quản lý toàn bộ hệ thống AGV.
 * @details Lớp này chịu trách nhiệm khởi tạo, chạy và dừng tất cả các luồng con
 * (PLC, LiDAR, Server, An toàn, Pin, v.v.). Nó đóng vai trò là "bộ não" điều phối
 * hoạt động của toàn bộ phần mềm, quản lý trạng thái chung và luồng dữ liệu
 * giữa các module.
 */
class SystemManager {
public:
    /**
     * @brief Hàm khởi tạo của SystemManager.
     * @param server_ip Địa chỉ IP của server trung tâm.
     * @param server_port Cổng của server trung tâm.
     */
    SystemManager(const std::string& server_ip, int server_port);

    /**
     * @brief Hàm hủy, đảm bảo gọi `stop()` để dọn dẹp tài nguyên.
     */
    ~SystemManager();

    /**
     * @brief Khởi tạo các thành phần cốt lõi của hệ thống.
     * @details Bao gồm kết nối đến server, PLC và thiết lập các callback.
     * @return `true` nếu khởi tạo thành công, `false` nếu thất bại.
     */
    bool initialize();

    /**
     * @brief Bắt đầu vòng lặp chính của hệ thống.
     * @details Khởi chạy tất cả các luồng và duy trì hoạt động cho đến khi nhận được tín hiệu dừng.
     */
    void run();

    /**
     * @brief Dừng toàn bộ hệ thống một cách an toàn.
     * @details Gửi tín hiệu dừng đến tất cả các luồng và chờ chúng kết thúc.
     */
    void stop();

private:
    // --- Các hàm thực thi của luồng ---
    /** @brief Ghim một luồng vào một lõi CPU cụ thể để tối ưu hiệu năng. */
    void pin_thread_to_core(int core_id);
    /** @brief Luồng xử lý giao tiếp với PLC. */
    void plc_thread_func();
    /** @brief Luồng xử lý dữ liệu từ cảm biến LiDAR. */
    void lidar_thread_func();
    /** @brief Luồng quản lý giao tiếp (gửi/nhận) với server. */
    void server_communication_thread();
    /** @brief Luồng xử lý các lệnh điều khiển nhận được từ server. */
    void command_handler_thread();
    /** @brief Luồng giám sát an toàn, đưa ra quyết định dừng/tiếp tục. */
    void safety_monitor_thread();
    /** @brief Luồng gửi các tác vụ định kỳ đến PLC (ví dụ: heartbeat). */
    void plc_periodic_tasks_thread();
    /** @brief Luồng giám sát và đọc dữ liệu từ BMS của pin. */
    void battery_thread_func();

    // --- Các hàm tiện ích ---
    /** @brief Gửi lệnh khởi tạo đến PLC khi hệ thống bắt đầu. */
    void initializeSystem();
    /** @brief Tính toán tốc độ mục tiêu dựa trên khoảng cách đến vật cản. */
    double calculateSpeed(double distance);
    /** @brief Tính toán tốc độ mượt mà, có gia tốc và giảm tốc. */
    int calculateSmoothSpeed(float distance_cm, bool movement_active);
    /** @brief Phân tích và thực thi một chuỗi lệnh trên PLC. */
    std::string parseAndExecutePlcCommand(const std::string& command, MCProtocol& plc);
    
    /**
     * @brief Thread loop that monitors and corrects the system heading.
     */
    void heading_correction_thread();

#if TEST_KEYBOARD_MODE == 1
    // Cấp quyền đọc input device
    // Tạo udev rule để service có quyền đọc keyboard:
    // SUBSYSTEM=="input", KERNEL=="event*", MODE="660", GROUP="input"
    // sudo usermod -a -G input <username>
    /** @brief Luồng đọc lệnh từ bàn phím để điều khiển AGV*/
    void keyboard_control_thread();
    std::string findKeyboardDevice();
#endif      

    // --- Biến thành viên ---
    SystemState state_; ///< Đối tượng trạng thái chung, được chia sẻ giữa các luồng.
    std::shared_ptr<MCProtocol> plc_ptr_; ///< Con trỏ chia sẻ đến đối tượng PLC.
    std::unique_ptr<ServerComm::CommunicationServer> comm_server_; ///< Con trỏ duy nhất đến đối tượng giao tiếp server.

    /// @brief Hàng đợi không khóa (lock-free) để truyền các điểm LiDAR ổn định từ luồng LiDAR đến luồng server.
    boost::lockfree::spsc_queue<std::vector<ServerComm::Point2D>, boost::lockfree::capacity<128>> stable_points_queue_;
    /// @brief Hàng đợi không khóa (lock-free) để truyền các điểm LiDAR thời gian thực từ luồng LiDAR đến luồng server.
    boost::lockfree::spsc_queue<std::vector<ServerComm::Point2D>, boost::lockfree::capacity<128>> realtime_points_queue_;
    /// @brief Hàng đợi không khóa để gửi lệnh từ các luồng khác đến luồng PLC.
    boost::lockfree::spsc_queue<std::string, boost::lockfree::capacity<512>> plc_command_queue_;
    /// @brief Hàng đợi không khóa để nhận kết quả từ luồng PLC.
    boost::lockfree::spsc_queue<std::string, boost::lockfree::capacity<512>> plc_result_queue_;

    std::mutex plc_ptr_mutex_; ///< Mutex để bảo vệ truy cập vào `plc_ptr_`.
    std::string last_movement_command_; ///< Lưu trữ lệnh di chuyển cuối cùng để có thể tự động tiếp tục.
    ServerComm::NavigationCommand::Type last_processed_cmd_type_; ///< Lưu loại lệnh cuối cùng đã xử lý để tránh xử lý lặp lại.
    std::mutex last_command_mutex_; ///< Mutex để bảo vệ truy cập vào `last_movement_command_`.

    std::atomic<bool> running_{true}; ///< Biến toàn cục để điều khiển việc dừng tất cả các luồng một cách an toàn.
    std::atomic<bool> plc_in_error_state_{false}; ///< Cờ báo hiệu PLC đang ở trạng thái lỗi (ví dụ: D102=5).
    std::atomic<bool> system_initialized_{false}; ///< Cờ đảm bảo hệ thống chỉ được khởi tạo một lần.

    std::vector<std::thread> threads_; ///< Vector chứa tất cả các luồng đang chạy của hệ thống.
    std::vector<ServerComm::Point2D> sampleImportantPoints(const std::vector<ServerComm::Point2D>& points); ///< Lấy mẫu các điểm quan trọng từ tập hợp điểm LiDAR.

    //**************************************************************************************************/
    // IMU/MPU9250
    std::unique_ptr<AGVNavigation> imu_ptr_; 
    std::mutex imu_data_mutex_;
    
    // IMU Data
    struct IMUData {
        float heading = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        float accel_x = 0.0f;
        float accel_y = 0.0f;
        float accel_z = 0.0f;
        float gyro_x = 0.0f;
        float gyro_y = 0.0f;
        float gyro_z = 0.0f;
        float temperature = 0.0f;
        bool is_tilted = false;
        bool is_safe = true;
        std::string compass_direction = "Unknown";
        unsigned long timestamp = 0;
    };
    
    IMUData latest_imu_data_;
    
    // Thread function
    void imu_thread_func();

    // Movement tracking với IMU
    struct MovementTarget {
        bool is_active = false;
        float target_heading = 0.0f;         // Góc mục tiêu cần giữ
        float heading_tolerance = 2.0f;       // Dung sai ±2°

        bool is_forward = true;               // true: tiến, false: lùi

        bool is_turning = false;              // true nếu đang thực hiện rẽ vòng cung
        float turn_start_heading = 0.0f;     // Góc bắt đầu khi rẽ

        std::chrono::steady_clock::time_point start_time;
    };
    
    MovementTarget current_movement_;
    std::mutex movement_target_mutex_;
    
    // PID Controller cho heading
    struct HeadingPID {
        float kp = 50.0f;      // Proportional gain
        float ki = 0.1f;       // Integral gain  
        float kd = 10.0f;      // Derivative gain
        
        float integral = 0.0f;
        float last_error = 0.0f;
        float max_integral = 100.0f;
        
        void reset() {
            integral = 0.0f;
            last_error = 0.0f;
        }
    };
    
    HeadingPID heading_pid_;

    // Arc turn control
    enum class ArcDirection {
        NONE,
        LEFT,
        RIGHT
    };
    ArcDirection arc_direction_;
    std::mutex arc_direction_mutex_;

    // Avoidance state
    enum class AvoidanceState {
        NONE,
        AVOIDING_LEFT,
        AVOIDING_RIGHT,
        RETURNING
    };
    AvoidanceState avoidance_state_ = AvoidanceState::NONE;
    std::mutex avoidance_mutex_;
    
    // Helper functions
    float normalizeAngle(float angle);
    float calculateHeadingError(float current, float target);
    void calculateDifferentialSpeed(float heading_error, int base_speed, 
                                   int &left_speed, int &right_speed);
    void applyHeadingCorrection();
};

#endif // SYSTEM_MANAGER_H
