#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>
#include <memory>
#include <cmath>

#include "config.h"
#ifdef ENABLE_LOG
#include "logger/Logger.h"
#endif

#include "MCprotocollib/MCprotocol.h"
#include "Lidarlib/Lidarlib.h"

// THREAD-SAFE QUEUE
template <typename T>
class ThreadSafeQueue {
public:
    void push(T value) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(std::move(value));
        m_cond.notify_one();
    }

    bool pop(T& value, int timeout_ms = 0) {
        std::unique_lock<std::mutex> lock(m_mutex);
        if (timeout_ms > 0) {
            if (m_cond.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                               [this]{ return !m_queue.empty(); })) {
                value = std::move(m_queue.front());
                m_queue.pop();
                return true;
            }
            return false; // Timeout
        } else {
            m_cond.wait(lock, [this]{ return !m_queue.empty(); });
            value = std::move(m_queue.front());
            m_queue.pop();
            return true;
        }
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size();
    }

private:
    std::queue<T> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cond;
};


// --- PLC Command Parsing and Execution ---
/**
 * @brief Phân tích chuỗi lệnh và thực thi hành động tương ứng trên PLC.
 * @param command Chuỗi lệnh (ví dụ: "READ_D100", "WRITE_M50_1").
 * @param plc Đối tượng MCProtocol đã kết nối.
 * @return Chuỗi kết quả của hành động.
 */
std::string parseAndExecutePlcCommand(const std::string& command, MCProtocol& plc) {
    std::stringstream ss(command);
    std::string command_part;
    std::string device_part;
    uint32_t address;
    uint16_t value;

    // Tách lệnh theo dấu '_'
    std::getline(ss, command_part, '_');
    std::getline(ss, device_part, '_');

    if (command_part.empty() || device_part.empty()) {
        return "Invalid command format: " + command;
    }

    // Lấy device và address
    std::string device_type = device_part.substr(0, 1);
    try {
        address = std::stoul(device_part.substr(1));
    } catch (const std::invalid_argument& e) {
        return "Invalid address in command: " + command;
    }

    // Kiểm tra các device được hỗ trợ
    if (device_type != "D" && device_type != "M" && device_type != "C" && device_type != "Y") {
        return "Unsupported device type: " + device_type;
    }

    if (command_part == "READ") {
        // Các thanh ghi D, C là word
        if (device_type == "D" || device_type == "C") {
            uint16_t read_value = plc.readSingleWord(device_type, address);
            return device_part + " = " + std::to_string(read_value);
        }
        // Các thanh ghi M, Y là bit, đọc theo word
        else if (device_type == "M" || device_type == "Y") {
            auto words = plc.readBits(device_type, address, 1);
            return device_part + " (word) = " + std::to_string(words[0]);
        }
    } else if (command_part == "WRITE") {
        // Đọc giá trị từ phần còn lại của command
        std::string remaining;
        std::getline(ss, remaining);
        
        // Loại bỏ dấu '_' đầu tiên nếu có
        if (!remaining.empty() && remaining[0] == '_') {
            remaining = remaining.substr(1);
        }
        
        try {
            value = std::stoul(remaining);
        } catch (const std::invalid_argument& e) {
            return "Invalid value for WRITE command: " + command;
        } catch (const std::out_of_range& e) {
            return "Value out of range for WRITE command: " + command;
        }

        bool success = plc.writeSingleWord(device_type, address, value);
        return success ? "Write " + device_part + " = " + std::to_string(value) + " OK"
                       : "Write " + device_part + " FAILED";
    }
    else {
        return "Unknown command: " + command_part;
    }
    
    //tránh warning
    return "Unexpected error";
};

// Biến toàn cục để điều khiển việc dừng các luồng một cách an toàn
std::atomic<bool> global_running{true};
// Cấu trúc Point2D đơn giản để tương thích với SystemState
struct Point2D {
    float x, y;
};
// SYSTEM STATE STRUCT
struct SystemState {
    std::mutex state_mutex;

     // Battery data
    double battery_level = 0.0;
    
    // PLC data
    bool plc_connected = false;
    std::string last_plc_status = "Chưa có dữ liệu PLC";


    // LiDAR data
    std::string last_lidar_data = "Chưa có dữ liệu Lidar";
    bool lidar_connected = false;
    std::vector<Point2D> latest_convex_hull;
    int total_stable_hulls = 0;
    int total_scan_count = 0;
    
    // SLAM mapping data
    std::vector<std::vector<Point2D>> global_map_hulls;
    long last_hull_timestamp = 0;

};

// Queue để truyền convex hull giữa các thread
// Communication queues
ThreadSafeQueue<std::vector<Point2D>> convex_hull_queue;
ThreadSafeQueue<std::string> lidar_status_queue;

// ENHANCED PLC THREAD WITH PROPER ERROR HANDLING
void plc_thread_func(
    SystemState& state,
    ThreadSafeQueue<std::string>& command_queue,
    ThreadSafeQueue<std::string>& result_queue)
{
#ifdef ENABLE_LOG
    LOG_INFO << "[PLC Thread] Starting...";
#else
    std::cout << "[PLC Thread] Starting..." << std::endl;
#endif

    // Create MC Protocol client instance
    MCProtocol plc(PLC_IP, PLC_PORT);
    
    // Try to establish connection to PLC
    bool connection_established = false;
    int connection_attempts = 0;
    const int max_attempts = 3;
    
    while (connection_attempts < max_attempts && !connection_established) {
        connection_attempts++;
#ifdef ENABLE_LOG
        LOG_INFO << "[PLC Thread] Connection attempt " << connection_attempts << "/" << max_attempts;
#else
        std::cout << "[PLC Thread] Connection attempt " << connection_attempts << "/" << max_attempts << std::endl;
#endif
        
        if (plc.connect()) {
            connection_established = true;
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.plc_connected = true;
                state.last_plc_status = "PLC connected successfully";
            }
#ifdef ENABLE_LOG
            LOG_INFO << "[PLC Thread] Connected to PLC successfully";
#else
            std::cout << "[PLC Thread] Connected to PLC successfully" << std::endl;
#endif
        } else {
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.plc_connected = false;
                state.last_plc_status = "PLC connection failed, attempt " + std::to_string(connection_attempts);
            }
#ifdef ENABLE_LOG
            LOG_ERROR << "[PLC Thread] Failed to connect to PLC, attempt " << connection_attempts;
#else
            std::cerr << "[PLC Thread] Failed to connect to PLC, attempt " << connection_attempts << std::endl;
#endif
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
    
    // If connection failed after all attempts, continue with simulation mode
    if (!connection_established) {
#ifdef ENABLE_LOG
        LOG_WARNING << "[PLC Thread] Running in simulation mode (PLC not connected)";
#else
        std::cout << "[PLC Thread] Running in simulation mode (PLC not connected)" << std::endl;
#endif
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.last_plc_status = "Running in simulation mode";
        }
    }

    // Main processing loop
    while (true) {
        std::string command;
        
        // Wait for command with timeout
        if (command_queue.pop(command, 100)) { // 100ms timeout
#ifdef ENABLE_LOG
            LOG_INFO << "[PLC Thread] Received command: " << command;
#else
            std::cout << "[PLC Thread] Received command: " << command << std::endl;
#endif

            std::string plc_response;
            
            if (connection_established && plc.isConnected()) {
                // Real PLC communication
                try {
                    // Sử dụng hàm tái cấu trúc để xử lý lệnh
                    plc_response = parseAndExecutePlcCommand(command, plc);

                } catch (const std::exception& e) {
                    plc_response = "PLC Error: " + std::string(e.what());
#ifdef ENABLE_LOG
                    LOG_ERROR << "[PLC Thread] PLC communication error: " << e.what();
#else
                    std::cerr << "[PLC Thread] PLC communication error: " << e.what() << std::endl;
#endif
                }
            } else {
                // Simulation mode
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                plc_response = "[SIMULATION] Response for '" + command + "' (timestamp: " + 
                              std::to_string(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())) + ")";
            }
            
            // Update system state
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.last_plc_status = plc_response;
            }
            
            // Send response back
            result_queue.push(plc_response);
        }
        
        // Small delay to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// LIDAR THREAD WITH SLAM PROCESSING
void lidar_thread_func(
    SystemState& state,
    ThreadSafeQueue<std::vector<Point2D>>& points_queue, 
    ThreadSafeQueue<std::string>& plc_command_queue)
{
#ifdef ENABLE_LOG
    LOG_INFO << "[LiDAR Thread] Starting...";
#else
    std::cout << "[LiDAR Thread] Starting..." << std::endl;
#endif

    // Khởi tạo LidarProcessor với các IP/Port mặc định
    std::string lidar_host_ip = LIDAR_HOST_IP;
    std::string lidar_port = std::to_string(LIDAR_PORT);
    std::string lidar_client_ip = LIDAR_CLIENT_IP; 
    std::string lidar_client_port = std::to_string(LIDAR_PORT);

    auto lidar_processor = std::unique_ptr<LidarProcessor>(new LidarProcessor(lidar_host_ip, lidar_port, lidar_client_ip, lidar_client_port));

    // Bước 1: Khởi tạo kết nối với LiDAR
    if (!lidar_processor->initialize()) {
#ifdef ENABLE_LOG
        LOG_ERROR << "[LiDAR Thread] Failed to initialize LidarProcessor.";
#else
        std::cerr << "[LiDAR Thread] Failed to initialize LidarProcessor." << std::endl;
#endif
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.lidar_connected = false;
            state.last_lidar_data = "Lidar initialization failed";
        }
        return; // Kết thúc luồng nếu không thể khởi tạo
    }

    // Bước 2: Bắt đầu luồng xử lý dữ liệu ngầm của thư viện LiDAR
    if (!lidar_processor->start()) {
#ifdef ENABLE_LOG
        LOG_ERROR << "[LiDAR Thread] Failed to start LidarProcessor.";
#else
        std::cerr << "[LiDAR Thread] Failed to start LidarProcessor." << std::endl;
#endif
         {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.lidar_connected = false;
            state.last_lidar_data = "Lidar start failed";
        }
        return;
    }

    // Cập nhật trạng thái thành công
    {
        std::lock_guard<std::mutex> lock(state.state_mutex);
        state.lidar_connected = true;
        state.last_lidar_data = "Lidar connected and running";
    }


    // Bước 3: Vòng lặp chính để xử lý dữ liệu
    while (global_running) {
        // Lấy dữ liệu điểm đã được ổn định từ thư viện
        std::vector<LidarPoint> stable_points = lidar_processor->getStablePoints();

        if (stable_points.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue; // Bỏ qua nếu chưa có dữ liệu
        }

        // --- Cập nhật trạng thái hệ thống ---
        std::vector<Point2D> current_points_2d;
        current_points_2d.reserve(stable_points.size());
        for(const auto& p : stable_points) {
            current_points_2d.push_back({p.x, p.y});
        }

        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.last_lidar_data = "Received " + std::to_string(stable_points.size()) + " stable points.";
            state.latest_convex_hull = current_points_2d; // Sử dụng lại trường này để lưu các điểm
        }

        // Gửi dữ liệu điểm sang các luồng khác nếu cần
        points_queue.push(current_points_2d);
        std::cout<<"[LiDAR Thread] Pushed " << current_points_2d.size() << " points to queue." << std::endl;
        std::cout<<current_points_2d.data()<<std::endl;

        // --- Phân tích vật cản phía trước ---
        float min_distance_in_front = 100.0f; // Khởi tạo khoảng cách lớn
        bool obstacle_found = false;

        // Khởi tạo khoảng cách tối thiểu cho từng hướng
        float min_front_left = 999.0;   // 270°-315°
        float min_front_right = 999.0;  // 45°-90°
        float min_left = 999.0;         // 225°-270°
        float min_right = 999.0;        // 90°-135°
        float min_front = 999.0;         // 135°-225°
        float min_front_distance = 0;
        int count_front_left = 0, count_front_right = 0;
        int count_left = 0, count_right = 0, count_back = 0;
            
        for (const auto& point : stable_points) {
            float angle = point.angle * 180.0f / M_PI; // Radian sang độ

             if (angle >= 270.0 && angle <= 315.0) {
                        // Trái
                min_left = std::min(min_left, point.distance);
                count_left++;
            }
            else if (angle >= 45.0 && angle <= 90.0) {
                           // Phải
                min_right = std::min(min_right, point.distance);
                count_right++;
            }
            else if (angle >= 225.0 && angle < 270.0) {


                // Phía trước-trái
                min_front_left = std::min(min_front_left, point.distance);
                count_front_left++;
            }
            else if (angle > 90.0 && angle <= 135.0) {
                            // Phía trước-phải
                min_front_right = std::min(min_front_right, point.distance);
                count_front_right++;
            }
            else if (angle > 135.0 && angle < 225.0) {

                // Phía trước
                min_front = std::min(min_front, point.distance);
                count_back++;
            }
            
        }


            // Hiển thị kết quả
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "=== PHÁT HIỆN VẬT CẢN ===" << std::endl;
    // std::cout << "Tổng điểm: " << points.size() << std::endl;
    std::cout << "Trước-Trái (270°-315°): " << min_front_left << "m (" << count_front_left << " điểm)" << std::endl;
    std::cout << "Trước-Phải (45°-90°):   " << min_front_right << "m (" << count_front_right << " điểm)" << std::endl;
    std::cout << "Trái (225°-270°):       " << min_left << "m (" << count_left << " điểm)" << std::endl;
    std::cout << "Phải (90°-135°):        " << min_right << "m (" << count_right << " điểm)" << std::endl;
    std::cout << "Sau (135°-225°):        " << min_front << "m (" << count_back << " điểm)" << std::endl;

        if (min_front < 1.0) {
            std::cout << "*** CẢNH BÁO: Vật cản gần phía trước (" << min_front << "m) ***" << std::endl;
        }
        // --- Ra quyết định và gửi lệnh cho PLC ---
        // Chuyển đổi khoảng cách từ mét sang centimet
        float min_dist_cm = min_front * 100.0f;

        // Nếu không có vật cản hoặc vật cản ở xa hơn 50cm
        if ( min_dist_cm > 50.0f) {
            // Gửi lệnh GHI giá trị 1 vào thanh ghi D100
           // plc_command_queue.push("WRITE_D_100_1"); //comment for test
           std::cout << "[LiDAR Thread] Path is clear (dist > 50cm). Sending command WRITE_D_100_1" << std::endl;
#ifdef ENABLE_LOG
            LOG_INFO << "[LiDAR Thread] Path is clear (dist > 50cm). Sending command WRITE_D_100_1";
#endif
        } else {
            // Ngược lại, có vật cản trong phạm vi 50cm
            // Gửi lệnh GHI giá trị 0 vào thanh ghi D100
           // plc_command_queue.push("WRITE_D_100_0");
           std::cout << "[LiDAR Thread] Obstacle detected at " << min_dist_cm << " cm. Sending command WRITE_D100_0" << std::endl;
#ifdef ENABLE_LOG
            LOG_WARNING << "[LiDAR Thread] Obstacle detected at " << min_dist_cm << " cm. Sending command WRITE_D100_0";
#endif
        }

        // Chờ một khoảng thời gian ngắn để giảm tải CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Dừng xử lý LiDAR trước khi kết thúc luồng
    lidar_processor->stop();
#ifdef ENABLE_LOG
    LOG_INFO << "[LiDAR Thread] Stopped.";
#else
    std::cout << "[LiDAR Thread] Stopped." << std::endl;
#endif
}


// Luồng giám sát Pin
// void battery_thread_func(SystemState& state) {
//     std::cout << "[Battery Thread] Khởi động." << std::endl;
//     while (true) {
//         // --- Placeholder: Đọc giá trị từ cảm biến Pin ---
//         double current_battery = 85.5 + (rand() % 10) / 10.0;
//         {
//             std::lock_guard<std::mutex> lock(state.state_mutex);
//             state.battery_level = current_battery;
//         }
//         std::this_thread::sleep_for(std::chrono::seconds(5));
//     }
// }

// Luồng Webserver
// void webserver_thread_func(SystemState& state) {
//     std::cout << "[Webserver Thread] Khởi động." << std::endl;
//     // --- Placeholder: Sử dụng thư viện như Crow hoặc Pistache ---
//     // Giả lập web server chạy
//     while(true) {
//         std::this_thread::sleep_for(std::chrono::seconds(10));
//     }
// }

// HÀM MAIN
int main() {
    std::cout << "[Main Thread] Control system starting..." << std::endl;

#ifdef ENABLE_LOG
    // Initialize logger
    Logger::get_instance().set_log_directory("./logger/log");
    LOG_INFO << "[Main Thread] System initialized";
#endif


    SystemState shared_state;
    ThreadSafeQueue<std::string> plc_command_queue;
    ThreadSafeQueue<std::string> plc_result_queue;

    // Start all threads
    // std::thread plc_thread(plc_thread_func, std::ref(shared_state), 
    //                       std::ref(plc_command_queue), std::ref(plc_result_queue));
    // KHỞI CHẠY LUỒNG LIDAR
    std::thread lidar_thread(lidar_thread_func, std::ref(shared_state),
                           std::ref(convex_hull_queue), std::ref(plc_command_queue));

    // Vòng lặp chính của main thread có thể để trống hoặc làm nhiệm vụ giám sát
    while (global_running) {
        // Có thể in ra trạng thái hệ thống ở đây để theo dõi
        std::this_thread::sleep_for(std::chrono::seconds(5));
        {
            std::lock_guard<std::mutex> lock(shared_state.state_mutex);
            std::cout << "\n--- SYSTEM STATUS ---" << std::endl;
            std::cout << "PLC Connected: " << (shared_state.plc_connected ? "Yes" : "No") << std::endl;
            std::cout << "LiDAR Connected: " << (shared_state.lidar_connected ? "Yes" : "No") << std::endl;
            std::cout << "Last LiDAR Data: " << shared_state.last_lidar_data << std::endl;
            std::cout << "---------------------\n" << std::endl;
        }

        // Có thể thêm điều kiện để dừng chương trình, ví dụ: nhấn phím
    }

    
    // // Test commands
    // std::vector<std::string> test_commands = {
    //     "READ_D100",
    //     "READ_D101",
    //     "WRITE_D100_1234",
    //     "READ_D100",
    //     "WRITE_D101_5678",
    //     "READ_D101",
    //     "INVALID_COMMAND"
    // };

    // int command_index = 0;
    // int total_commands = test_commands.size();
    // auto last_command_time = std::chrono::steady_clock::now();
    // const auto command_interval = std::chrono::seconds(5);

    // while (global_running) {
    //     auto current_time = std::chrono::steady_clock::now();
        
    //     // Send test commands periodically
    //     if (current_time - last_command_time >= command_interval) {
    //         std::string command = test_commands[command_index % total_commands];
    //         std::cout << "\n[Main Thread] Sending PLC command: " << command << std::endl;
    //         plc_command_queue.push(command);

    //         // Wait for response
    //         std::string result;
    //         if (plc_result_queue.pop(result, 3000)) { // 3 second timeout
    //             std::cout << "[Main Thread] PLC Response: " << result << std::endl;
    //         } else {
    //             std::cout << "[Main Thread] Timeout waiting for PLC response!" << std::endl;
    //         }

    //         command_index++;
    //         last_command_time = current_time;
    //     }
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    std::cout << "\n[Main Thread] Initiating shutdown..." << std::endl;
    
    // Stop all threads gracefully
    global_running = false;
    
    // Join all threads
    // if (plc_thread.joinable()) {
    //     plc_thread.join();
    //     std::cout << "[Main Thread] PLC thread stopped." << std::endl;
    // }

    if (lidar_thread.joinable()) {
        lidar_thread.join();
        std::cout << "[Main Thread] LiDAR thread stopped." << std::endl;
    }

    std::cout << "[Main Thread] System shutdown complete." << std::endl;
    return 0;
}