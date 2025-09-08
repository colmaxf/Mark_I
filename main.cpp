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
#include "logger/Logger.h"


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

    // THÊM CÁC TRƯỜNG MỚI
    bool is_safe_to_move = false;           // Cờ an toàn từ LiDAR realtime
    float current_front_distance = -1.0f;  // Khoảng cách phía trước hiện tại (cm)
    long last_safety_update = 0;           // Timestamp cập nhật an toàn cuối cùng
    
    // SLAM mapping data
    std::vector<std::vector<Point2D>> global_map_hulls;
    long last_hull_timestamp = 0;

};
/**
 * @brief Xử lý input từ bàn phím và gửi lệnh tương ứng xuống PLC
 * @param input Ký tự nhập từ bàn phím (1, 2, 3, 4)
 * @param plc_command_queue Queue để gửi lệnh PLC
 * @param plc_result_queue Queue để nhận kết quả từ PLC
 * @return true nếu xử lý thành công, false nếu input không hợp lệ
 */
bool handleKeyboardInput(char input, 
                        ThreadSafeQueue<std::string>& plc_command_queue,
                        ThreadSafeQueue<std::string>& plc_result_queue,
                        SystemState& system_state) {
    
    std::string command;
    bool need_safety_check = false;
    
    switch(input) {
        case '1':
            // Lệnh đặc biệt: cần kiểm tra an toàn trước khi ghi D100
            command = "WRITE_D100_1";
            need_safety_check = true;
            LOG_INFO << "[Keyboard] User pressed 1 -> Request to write 1 to D100 (safety check required)";
            break;
        case '2':
            command = "WRITE_D101_1";
            LOG_INFO << "[Keyboard] User pressed 2 -> Writing 1 to D101";
            break;
        case '3':
            command = "WRITE_D102_1";
            LOG_INFO << "[Keyboard] User pressed 3 -> Writing 1 to D102";
            break;
        case '4':
            command = "WRITE_D103_1";
            LOG_INFO << "[Keyboard] User pressed 4 -> Writing 1 to D103";
            break;
        default:
            LOG_WARNING << "[Keyboard] Invalid input: " << input << " (Expected: 1, 2, 3, or 4)";
            return false;
    }
    
    // Kiểm tra an toàn cho lệnh ghi D100
    if (need_safety_check) {
        bool is_safe = false;
        float front_distance = -1.0f;
        long last_update = 0;
        
        // Lấy cờ an toàn và dữ liệu từ system state
        {
            std::lock_guard<std::mutex> lock(system_state.state_mutex);
            is_safe = system_state.is_safe_to_move;
            front_distance = system_state.current_front_distance;
            last_update = system_state.last_safety_update;
        }
        
        // Kiểm tra tính cập nhật của dữ liệu (trong vòng 2 giây)
        long current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        bool data_is_fresh = (current_time - last_update) < 2000; // 2 seconds
        
        LOG_INFO << "[Keyboard] Safety check for D100: Safe=" << (is_safe ? "YES" : "NO") 
                << ", Front=" << front_distance << "cm, DataAge=" << (current_time - last_update) << "ms";
        
        // Kiểm tra điều kiện an toàn
        if (!data_is_fresh) {
            LOG_ERROR << "[Keyboard] SAFETY BLOCK: LiDAR safety data is stale (>" 
                     << (current_time - last_update) << "ms old)";
            std::cout << "❌ SAFETY BLOCK: LiDAR safety data is too old!" << std::endl;
            return false;
        }
        
        if (front_distance < 0) {
            LOG_ERROR << "[Keyboard] SAFETY BLOCK: Invalid front distance data";
            std::cout << "❌ SAFETY BLOCK: LiDAR distance data invalid!" << std::endl;
            return false;
        }
        
        if (!is_safe) {
            LOG_WARNING << "[Keyboard] SAFETY BLOCK: LiDAR safety flag is FALSE. Front distance: " 
                       << front_distance << "cm";
            std::cout << "❌ SAFETY BLOCK: Path not clear! Front distance: " << front_distance 
                     << "cm (must be > 50cm)" << std::endl;
            return false;
        }
        
        // An toàn - cho phép thực hiện lệnh
        LOG_INFO << "[Keyboard] SAFETY OK: LiDAR reports safe conditions. Front distance: " 
                << front_distance << "cm. Proceeding with D100 write command.";
        std::cout << "✅ SAFETY OK: Path clear (" << front_distance 
                 << "cm). Writing to D100..." << std::endl;
    }
    
    // Gửi lệnh vào queue
    plc_command_queue.push(command);
    
    // Chờ phản hồi từ PLC (timeout 3 giây)
    std::string result;
    if (plc_result_queue.pop(result, 3000)) {
        LOG_INFO << "[Keyboard] PLC Response: " << result;
        return true;
    } else {
        LOG_ERROR << "[Keyboard] Timeout waiting for PLC response!";
        return false;
    }
}


/**
 * @brief Thread function để xử lý input từ bàn phím
 * @param plc_command_queue Queue để gửi lệnh PLC
 * @param plc_result_queue Queue để nhận kết quả từ PLC
 * @param system_state Tham chiếu đến system state để kiểm tra dữ liệu LiDAR
 */
void keyboard_input_thread(ThreadSafeQueue<std::string>& plc_command_queue,
                          ThreadSafeQueue<std::string>& plc_result_queue,
                          SystemState& system_state) {
    
    LOG_INFO << "[Keyboard Thread] Started. Press 1-4 to send commands to PLC, 'q' to quit";
    LOG_INFO << "[Keyboard Thread]=== PLC Control Interface ===" ;
    LOG_INFO << "[Keyboard Thread]Available commands:" ;
    LOG_INFO << "[Keyboard Thread]  1 -> Write 1 to D100 (with safety check)" ;
    LOG_INFO << "[Keyboard Thread]  2 -> Write 1 to D101" ;
    LOG_INFO << "[Keyboard Thread]  3 -> Write 1 to D102" ;
    LOG_INFO << "[Keyboard Thread]  4 -> Write 1 to D103" ;
    LOG_INFO << "[Keyboard Thread]  q -> Quit program" ;
    LOG_INFO << "[Keyboard Thread]==============================" ;
    
    char input;
    while (global_running) {
        LOG_INFO << "[Keyboard Thread] Enter command (1-4, q=quit): ";
        std::cout << "Enter command (1-4, q=quit): ";
        std::cin >> input;
        
        // Xử lý lệnh quit
        if (input == 'q' || input == 'Q') {
            LOG_INFO << "[Keyboard Thread] Quit command received";
            global_running = false;
            break;
        }
        
        // Xử lý các lệnh PLC với kiểm tra an toàn
        bool success = handleKeyboardInput(input, plc_command_queue, plc_result_queue, system_state);
        
        if (success) {
            LOG_INFO << "[Keyboard Thread] Command executed successfully!";
            std::cout << "Command executed successfully!\n" << std::endl;
        } else {
            LOG_ERROR << "[Keyboard Thread] Command failed, blocked, or invalid input!";
            std::cout << "✗ Command failed, blocked, or invalid input!\n" << std::endl;
        }
        
        // Ngắt nghỉ ngắn để tránh busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    LOG_INFO << "[Keyboard Thread] Stopped";
}

/**
 * @brief Phiên bản non-blocking của keyboard input (không cần thread riêng)
 * Thêm vào main loop để kiểm tra input định kỳ
 */
void checkKeyboardInputNonBlocking(ThreadSafeQueue<std::string>& plc_command_queue,
                                  ThreadSafeQueue<std::string>& plc_result_queue,
                                  SystemState& system_state) {
    
    static bool first_call = true;
    if (first_call) {
        LOG_INFO << "[Keyboard Thread]=== PLC Control Interface ===" ;
        LOG_INFO << "[Keyboard Thread]Available commands:" ;
        LOG_INFO << "[Keyboard Thread]  1 -> Write 1 to D100 (with safety check)" ;
        LOG_INFO << "[Keyboard Thread]  2 -> Write 1 to D101" ;
        LOG_INFO << "[Keyboard Thread]  3 -> Write 1 to D102" ;
        LOG_INFO << "[Keyboard Thread]  4 -> Write 1 to D103" ;
        LOG_INFO << "[Keyboard Thread]  q -> Quit program" ;
        LOG_INFO << "[Keyboard Thread]==============================" ;
        first_call = false;
    }
    
    // Kiểm tra xem có input từ stdin không (non-blocking)
    fd_set read_fds;
    struct timeval timeout;
    
    FD_ZERO(&read_fds);
    FD_SET(STDIN_FILENO, &read_fds);
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    
    int result = select(STDIN_FILENO + 1, &read_fds, NULL, NULL, &timeout);
    
    if (result > 0 && FD_ISSET(STDIN_FILENO, &read_fds)) {
        char input;
        if (std::cin >> input) {
            if (input == 'q' || input == 'Q') {
                LOG_INFO << "[Main] Quit command received";
                global_running = false;
                return;
            }
            
            bool success = handleKeyboardInput(input, plc_command_queue, plc_result_queue, system_state);
            
            if (success) {
                LOG_INFO << "[Keyboard Thread] Command executed successfully!" ;
            } else {
                LOG_ERROR << "[Keyboard Thread] Command failed, blocked, or invalid input!" ;
            }
            LOG_INFO << "[Keyboard Thread] Enter next command (1-4, q=quit): ";
            std::cout.flush();
        }
    }
}

// Queue để truyền convex hull giữa các thread
// Communication queues
ThreadSafeQueue<std::vector<Point2D>> stable_points_queue;
ThreadSafeQueue<std::string> lidar_status_queue;

// ENHANCED PLC THREAD WITH PROPER ERROR HANDLING
void plc_thread_func(
    SystemState& state,
    ThreadSafeQueue<std::string>& command_queue,
    ThreadSafeQueue<std::string>& result_queue)
{
    LOG_INFO << "[PLC Thread] Starting...";

    // Create MC Protocol client instance
    MCProtocol plc(PLC_IP, PLC_PORT);
    
    // Try to establish connection to PLC
    bool connection_established = false;
    int connection_attempts = 0;
    const int max_attempts = 3;
    
    while (connection_attempts < max_attempts && !connection_established) {
        connection_attempts++;

        LOG_INFO << "[PLC Thread] Connection attempt " << connection_attempts << "/" << max_attempts;
        
        if (plc.connect()) {
            connection_established = true;
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.plc_connected = true;
                state.last_plc_status = "PLC connected successfully";
            }

            LOG_INFO << "[PLC Thread] Connected to PLC successfully";

        } else {
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.plc_connected = false;
                state.last_plc_status = "PLC connection failed, attempt " + std::to_string(connection_attempts);
            }

            LOG_ERROR << "[PLC Thread] Failed to connect to PLC, attempt " << connection_attempts;

            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
    
    // If connection failed after all attempts, continue with simulation mode
    if (!connection_established) {

        LOG_WARNING << "[PLC Thread] Running in simulation mode (PLC not connected)";

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

            LOG_INFO << "[PLC Thread] Received command: " << command;

            std::string plc_response;
            
            if (connection_established && plc.isConnected()) {
                // Real PLC communication
                try {
                    // Sử dụng hàm tái cấu trúc để xử lý lệnh
                    plc_response = parseAndExecutePlcCommand(command, plc);

                } catch (const std::exception& e) {
                    plc_response = "PLC Error: " + std::string(e.what());
                    LOG_ERROR << "[PLC Thread] PLC communication error: " << e.what();
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

    LOG_INFO << "[LiDAR Thread] Starting...";

    // Khởi tạo LidarProcessor với các IP/Port mặc định
    std::string lidar_host_ip = LIDAR_HOST_IP;
    std::string lidar_port = std::to_string(LIDAR_PORT);
    std::string lidar_client_ip = LIDAR_CLIENT_IP; 
    std::string lidar_client_port = std::to_string(LIDAR_PORT);

    auto lidar_processor = std::unique_ptr<LidarProcessor>(new LidarProcessor(lidar_host_ip, lidar_port, lidar_client_ip, lidar_client_port));

    // Bước 1: Khởi tạo kết nối với LiDAR
    if (!lidar_processor->initialize()) {

        LOG_ERROR << "[LiDAR Thread] Failed to initialize LidarProcessor.";

        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.lidar_connected = false;
            state.last_lidar_data = "Lidar initialization failed";
        }
        return; // Kết thúc luồng nếu không thể khởi tạo
    }

    // Bước 2: Bắt đầu luồng xử lý dữ liệu ngầm của thư viện LiDAR
    if (!lidar_processor->start()) {

        LOG_ERROR << "[LiDAR Thread] Failed to start LidarProcessor.";

        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.lidar_connected = false;
            state.last_lidar_data = "Lidar start failed";
        }
        return;
    }

    // ====== THIẾT LẬP DUAL MODE PROCESSING ======
    
    // 1. REALTIME CALLBACK - Phản hồi nhanh cho obstacle detection
    lidar_processor->setRealtimeCallback([&state, &plc_command_queue](const std::vector<LidarPoint>& points) {
        // Khởi tạo khoảng cách tối thiểu cho từng hướng
        float min_front_left = 999.0f;   // 270°-315°
        float min_front_right = 999.0f;  // 45°-90°
        float min_left = 999.0f;         // 225°-270°
        float min_right = 999.0f;        // 90°-135°
        float min_front = 999.0f;        // 135°-225°
        
        // Phân tích từng điểm
        for (const auto& point : points) {
            float angle_deg = point.angle * 180.0f / M_PI;
            
            if (angle_deg >= 270.0 && angle_deg <= 315.0) {// Trái
                min_left = std::min(min_left, point.distance);
            }
            else if (angle_deg >= 45.0 && angle_deg <= 90.0) {// Phải
                min_right = std::min(min_right, point.distance);
            }
            else if (angle_deg >= 225.0 && angle_deg < 270.0) {// Phía trước-trái
                min_front_left = std::min(min_front_left, point.distance);
            }
            else if (angle_deg > 90.0 && angle_deg <= 135.0) {// Phía trước-phải
                min_front_right = std::min(min_front_right, point.distance);
            }
            else if (angle_deg > 135.0 && angle_deg < 225.0) {// Phía trước
                min_front = std::min(min_front, point.distance);
            }
        }
        
        // Xử lý phản hồi NHANH cho vật cản phía trước
        if (min_front < 999.0f) {
            float min_dist_cm = min_front * 100.0f;
            
            // Gửi lệnh PLC ngay lập tức
            if (min_dist_cm > 50.0f) {
                plc_command_queue.push("WRITE_D100_1");
                // Debug output với màu xanh cho an toàn
                LOG_INFO << "[REALTIME] Path clear: " << min_dist_cm << "cm";

            } else {
                plc_command_queue.push("WRITE_D100_0");
                // Debug output với màu đỏ cho cảnh báo
                LOG_WARNING << "[REALTIME WARNING] Obstacle detected: " << min_dist_cm << "cm";
            }
            
            // Cập nhật state với thông tin realtime
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.last_lidar_data = "[RT] Front: " + std::to_string(min_dist_cm) + "cm | " +
                                       "L: " + std::to_string((int)(min_left*100)) + "cm | " +
                                       "R: " + std::to_string((int)(min_right*100)) + "cm";
            }
        }
    });
// 2. STABLE CALLBACK - Dữ liệu ổn định cho server
    lidar_processor->setStablePointsCallback([&state, &points_queue](const std::vector<LidarPoint>& stable_points) {
        if (stable_points.empty()) return;
        
        // Chuyển đổi sang Point2D
        std::vector<Point2D> stable_2d;
        stable_2d.reserve(stable_points.size());
        
        for(const auto& p : stable_points) {
            stable_2d.push_back({p.x, p.y});
        }
        
        // Cập nhật state cho monitoring
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.latest_convex_hull = stable_2d;
            state.total_stable_hulls++;
        }
        
        // Gửi vào queue cho server
        points_queue.push(stable_2d);
        
        // Log với màu xanh dương cho stable data
        LOG_INFO << "[STABLE DATA] Processed " << stable_2d.size() << " points for server (Total stable scans: " << state.total_stable_hulls << ")";
    });
    
    // 3. Điều chỉnh tham số cho cân bằng giữa realtime và stability
    lidar_processor->setStabilizerParams(
        3,      // history_window - đủ để ổn định nhưng không quá chậm
        0.05f,  // stability_threshold - độ chính xác vừa phải
        0.15f   // outlier_threshold - loại bỏ nhiễu
    );
    
    // Thiết lập noise filter cho dữ liệu sạch hơn
    lidar_processor->setNoiseFilterParams(
        0.05f,  // min_distance: 5cm
        50.0f,  // max_distance: 50m
        0.5f,   // max_angle_jump
        2       // min_neighbors
    );

    // Bắt đầu xử lý
    if (!lidar_processor->start()) {
        LOG_ERROR << "[LiDAR Thread] Failed to start LidarProcessor.";
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
        state.last_lidar_data = "Lidar connected - Dual mode active";
    }

    LOG_INFO << "[32m[LiDAR Thread] Successfully started with dual-mode processing:"; 
    LOG_INFO << "  - REALTIME mode: Obstacle detection with immediate response";
    LOG_INFO << "  - STABLE mode: Filtered data for server upload";

    // Biến để theo dõi hiệu suất
    auto last_stats_time = std::chrono::steady_clock::now();
    const auto stats_interval = std::chrono::seconds(10);

    // Main monitoring loop
    while (global_running) {
        auto current_time = std::chrono::steady_clock::now();
        
        // In thống kê định kỳ
        if (current_time - last_stats_time >= stats_interval) {
            if (lidar_processor->isProcessing()) {

                LOG_INFO << "[=== LIDAR PERFORMANCE STATS ===";
                LOG_INFO << "Processing Rate: " << lidar_processor->getProcessingRate() << " Hz"
                         << " | Stability Score: " << lidar_processor->getStabilityScore() * 100 << "%" 
                         << " | Data Validity: " << lidar_processor->getDataValidityRatio() * 100 << "%" 
                         << " | Total Scans: " << lidar_processor->getTotalScans() 
                         << " | Valid: " << lidar_processor->getValidScans()
                         << " | Stable: " << lidar_processor->getStableScans() 
                         << " | Uptime: " << lidar_processor->getUptime() << " seconds" ;
                LOG_INFO << "[===============================" ;

            }
            last_stats_time = current_time;
        }
        
        // Kiểm tra kết nối
        if (!lidar_processor->isConnected()) {
            LOG_WARNING << "[LiDAR Thread] Connection lost! Attempting to reconnect...";
            // TODO: Implement reconnection logic if needed
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Dừng xử lý LiDAR
    LOG_INFO << "[LiDAR Thread] Shutting down...";
    lidar_processor->stop();
    
    LOG_INFO << "[LiDAR Thread] Stopped successfully.";
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

    // Initialize logger
// Lấy instance duy nhất của Logger (singleton)
    Logger& logger = Logger::get_instance();
    
    // Configure logging
    // logger.set_log_directory("./logger/log/");
    // logger.set_max_file_count(15);
    // logger.set_max_file_size(100); // 100MB per file

    // // IMPORTANT: Enable both offline and network logging
    // // Or use combined mode:
    //  //logger.set_log_mode(2); // 2 = BOTH (file + network)

    //  logger.enable_offline_logging(true);  // Save to .dlt files
    //  logger.enable_network_logging(true, "0.0.0.0", 3490); // Enable network access
    

    
    // Register MAIN app and context
    LOG_REGISTER_APP("MAIN", "Main AGV Application");
    LOG_REGISTER_CONTEXT("MAIN", "Main Control Context");
    LOG_SET_APP("MAIN");
    LOG_SET_CONTEXT("MAIN");
    LOG_INFO << "[Main Thread] System initialized";


    SystemState shared_state;
    ThreadSafeQueue<std::string> plc_command_queue;
    ThreadSafeQueue<std::string> plc_result_queue;

    // Start all threads
     std::thread plc_thread(plc_thread_func, std::ref(shared_state), 
                           std::ref(plc_command_queue), std::ref(plc_result_queue));
    // KHỞI CHẠY LUỒNG LIDAR
    std::thread lidar_thread(lidar_thread_func, std::ref(shared_state),
                           std::ref(stable_points_queue), std::ref(plc_command_queue));
    // 3. KEYBOARD INPUT THREAD (MỚI THÊM)
    std::thread keyboard_thread(keyboard_input_thread, 
                               std::ref(plc_command_queue), 
                               std::ref(plc_result_queue), 
                               std::ref(shared_state));
    //Server thread
    // std::thread webserver_thread(webserver_thread_func, std::ref(shared_state), std::ref(stable_points_queue));
    //Battery thread
    // std::thread battery_thread(battery_thread_func, std::ref(shared_state)); 


    // Vòng lặp chính của main thread có thể để trống hoặc làm nhiệm vụ giám sát
    while (global_running) {
        // Có thể in ra trạng thái hệ thống ở đây để theo dõi
        std::this_thread::sleep_for(std::chrono::seconds(5));
        {
            std::lock_guard<std::mutex> lock(shared_state.state_mutex);
            LOG_INFO << "--- SYSTEM STATUS ---" ;
            LOG_INFO << "PLC Connected: " << (shared_state.plc_connected ? "Yes" : "No") ;
            LOG_INFO << "LiDAR Connected: " << (shared_state.lidar_connected ? "Yes" : "No") ;
            LOG_INFO << "Last LiDAR Data: " << shared_state.last_lidar_data ;
            LOG_INFO << "---------------------";
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
    if (plc_thread.joinable()) {
        plc_thread.join();
        std::cout << "[Main Thread] PLC thread stopped." << std::endl;
    }

    if (lidar_thread.joinable()) {
        lidar_thread.join();
        std::cout << "[Main Thread] LiDAR thread stopped." << std::endl;
    }

       if (keyboard_thread.joinable()) {
        keyboard_thread.join();
        std::cout << "[Main Thread] Keyboard thread stopped." << std::endl;
    }

    std::cout << "[Main Thread] System shutdown complete." << std::endl;
    return 0;
}