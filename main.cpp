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

#include <atomic> // Cần cho std::atomic
#include <memory> // Cần cho std::unique_ptr
#include <sstream> // Cần cho std::stringstream

// --- Thư viện cần cho Keyboard Listener ---
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "MCprotocollib/MCprotocol.h"
#include "Lidarlib/Lidarlib.h"
#include "Serverlib/servercommunicator.h"

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

//------------------------------------- Global variables-------------------------------------//
// Biến để đảm bảo khởi tạo hệ thống chỉ một lần
std::atomic<bool> plc_in_error_state{false};
std::atomic<bool> system_initialized{false};
std::shared_ptr<MCProtocol> global_plc_ptr;  // Shared pointer cho PLC
std::mutex plc_ptr_mutex;  // Mutex để bảo vệ truy cập

std::string last_movement_command = "";
std::mutex last_command_mutex;

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
    //int total_scan_count = 0;

    // Safety data
    bool is_safe_to_move = false;           // Cờ an toàn từ LiDAR realtime
    float current_front_distance = -1.0f;  // Khoảng cách phía trước hiện tại (cm)
    long last_safety_update = 0;           // Timestamp cập nhật an toàn cuối cùng

    //Tracking cho smooth acceleration
    bool is_moving = false;
    std::chrono::steady_clock::time_point movement_start_time;
    int current_speed = 0;
    int target_speed = 0;
    // Kiểm soát việc có đang trong lệnh di chuyển hay không
    bool movement_command_active = false;  // Cờ này sẽ được set khi nhận lệnh W/S/A/D
};

// Queue để truyền convex hull giữa các thread
// Communication queues
ThreadSafeQueue<std::vector<Point2D>> stable_points_queue;
//ThreadSafeQueue<std::string> lidar_status_queue;

//-----------------------------------------------------------------------------//

// --- LỚP KEYBOARD LISTENER ---
class SimpleKeyboardListener {
private:
    struct termios oldSettings, newSettings;
public:
    SimpleKeyboardListener() {
        tcgetattr(STDIN_FILENO, &oldSettings);
        newSettings = oldSettings;
        newSettings.c_lflag &= ~(ICANON | ECHO);
        newSettings.c_cc[VMIN] = 0;
        newSettings.c_cc[VTIME] = 1;
        tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);
    }
    ~SimpleKeyboardListener() {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    }
    char getChar() {
        char ch = 0;
        if (read(STDIN_FILENO, &ch, 1) > 0) {
            if (ch == 27) { // Escape sequence for arrow keys
                char seq[2];
                if (read(STDIN_FILENO, &seq[0], 1) > 0 && read(STDIN_FILENO, &seq[1], 1) > 0 && seq[0] == '[') {
                    switch (seq[1]) {
                        case 'A': return 'W'; // Up
                        case 'B': return 'S'; // Down
                        case 'C': return 'D'; // Right
                        case 'D': return 'A'; // Left
                    }
                }
                return 27; // ESC key
            }
            return toupper(ch);
        }
        return 0; // No input
    }
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
        // Ghi giá trị
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


/**
 * @brief Hàm khởi tạo hệ thống - gửi lệnh D110_1 khi bắt đầu (CHỈ MỘT LẦN)
 */
void initializeSystem(ThreadSafeQueue<std::string>& plc_command_queue,
                     ThreadSafeQueue<std::string>& plc_result_queue) {
    if (system_initialized.exchange(true)) {
        LOG_INFO << "[System Init] Already initialized, skipping D110_1";
        return;
    }

    LOG_INFO << "[System Init] Sending D110_1";
    plc_command_queue.push("WRITE_D110_1");
    std::string result;
    if (plc_result_queue.pop(result, 2000)) {
        LOG_INFO << "[System Init] D110_1 response: " << result;
    } else {
        LOG_ERROR << "[System Init] No response for D110_1";
    }
}

double calculateSpeed(double distance) {
    // Đảm bảo khoảng cách nằm trong range [50, 400]
    distance = std::max(50.0, std::min(400.0, distance));
    
    // Hàm ánh xạ tuyến tính: y = y1 + ((x - x1) * (y2 - y1)) / (x2 - x1)
    if (distance >= 50.0 && distance < 100.0) {
        // Khoảng 50-100 cm ánh xạ tốc độ 0-500
        return 0.0 + ((distance - 50.0) * (500.0 - 0.0)) / (100.0 - 50.0);
    } else if (distance >= 100.0 && distance < 200.0) {
        // Khoảng 100-200 cm ánh xạ tốc độ 500-1500
        return 500.0 + ((distance - 100.0) * (1500.0 - 500.0)) / (200.0 - 100.0);
    } else if (distance >= 200.0 && distance < 300.0) {
        // Khoảng 200-300 cm ánh xạ tốc độ 1500-2500
        return 1500.0 + ((distance - 200.0) * (2500.0 - 1500.0)) / (300.0 - 200.0);
    } else if (distance >= 300.0 && distance < 400.0) {
        // Khoảng 300-400 cm ánh xạ tốc độ 2500-3000
        return 2500.0 + ((distance - 300.0) * (3000.0 - 2500.0)) / (400.0 - 300.0);
    } else {
        return 200.0; // An toàn
    }
}

int calculateSmoothSpeed(SystemState& state, float distance_cm, bool movement_active) {
    // Nếu không có lệnh di chuyển, trả về 0
    if (!movement_active) {
        state.is_moving = false;
        state.current_speed = 0;
        return 0;
    }
    
    // Tính tốc độ mục tiêu dựa trên khoảng cách
    int target = static_cast<int>(calculateSpeed(distance_cm));
    
    // Nếu vừa bắt đầu di chuyển
    if (!state.is_moving && target > 0) {
        LOG_INFO << "[Speed Control] Starting movement towards target speed: " << target << " from 0";
        state.is_moving = true;
        state.movement_start_time = std::chrono::steady_clock::now();
        state.current_speed = 0;
    }
    
    // Nếu đang di chuyển
    if (state.is_moving) {
        LOG_INFO << "[Speed Control] Current speed: " << state.current_speed << ", Target speed: " << target;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - state.movement_start_time).count();
        
        // Tăng tốc dần trong 2 giây đầu
        if (elapsed < 2000) {
            float t = elapsed / 2000.0f;
            float curve = t * t * (3.0f - 2.0f * t);  // smoothstep
            state.current_speed = static_cast<int>(target * curve);
        } else {
            // Điều chỉnh mượt về tốc độ mục tiêu
            int speed_diff = target - state.current_speed;
            if (abs(speed_diff) > 5) {
                state.current_speed += (speed_diff > 0) ? 5 : -5;
            } else {
                state.current_speed = target;
            }
        }
    }
    
    // Reset khi dừng
    if (target == 0) {
        state.is_moving = false;
        state.current_speed = 0;
    }
    
    LOG_INFO << "[Speed Control] Returning speed: " << state.current_speed;
    return state.current_speed;
}

/**
 * @brief Background safety monitor thread - kiểm tra khoảng cách liên tục
 * và gửi lệnh dừng khẩn cấp khi cần thiết
 */
void safety_monitor_thread(ThreadSafeQueue<std::string>& plc_command_queue,
                           SystemState& system_state) {
    
    
    LOG_INFO << "[Safety Monitor] Started - monitoring obstacle distance and auto-resume is ACTIVE";
    
    bool was_safe = true;
    auto last_warning_time = std::chrono::steady_clock::now();
    const auto warning_interval = std::chrono::seconds(2);
    
    while (global_running) {
        bool is_safe = true; // Mặc định là an toàn
        float front_distance = -1.0f;
        
        {
            std::lock_guard<std::mutex> lock(system_state.state_mutex);
            is_safe = system_state.is_safe_to_move;
            front_distance = system_state.current_front_distance;
        }
        
        auto current_time = std::chrono::steady_clock::now();
        
        // --- LOGIC MỚI ---
        
        // 1. Chuyển từ AN TOÀN sang NGUY HIỂM
        if (was_safe && !is_safe) {
            LOG_WARNING << "[Safety Monitor] DANGER DETECTED! Distance: " << front_distance << "cm";
            
            // Gửi lệnh dừng khẩn cấp
            plc_command_queue.push("WRITE_D100_0");
            plc_command_queue.push("WRITE_D101_0");

            // // Xóa lệnh di chuyển cuối cùng để ngăn resume không mong muốn
            // {
            //     std::lock_guard<std::mutex> lock(last_command_mutex);
            //     last_movement_command.clear();
            // }
            
        } 
        // 2. Chuyển từ NGUY HIỂM sang AN TOÀN -> TỰ ĐỘNG RESUME
        else if (!was_safe && is_safe) {
            LOG_INFO << "[Safety Monitor] Path clear again - Distance: " << front_distance << "cm. Checking for command to resume...";

            // Đọc và gửi lại lệnh cuối cùng nếu có
            std::lock_guard<std::mutex> lock(last_command_mutex);
            if (!last_movement_command.empty() && !plc_in_error_state) {
                LOG_INFO << "[Safety Monitor] Resuming last command: " << last_movement_command;
                plc_command_queue.push(last_movement_command);
            }
        }
        
        // 3. Cảnh báo định kỳ khi vẫn đang trong vùng nguy hiểm
        if (!is_safe && (current_time - last_warning_time >= warning_interval)) {
            LOG_WARNING << "[Safety Monitor] Still unsafe - Distance: " << front_distance << "cm";
            last_warning_time = current_time;
        }
        
        was_safe = is_safe;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    LOG_INFO << "[Safety Monitor] Stopped";
}

/**
 * @brief Luồng cho các tác vụ PLC định kỳ (ví dụ: heartbeat, ghi dữ liệu liên tục)
 */
void plc_periodic_tasks_thread(ThreadSafeQueue<std::string>& plc_command_queue) {
    LOG_INFO << "[PLC Periodic] Thread started. Will write D200=1 every 500ms.";

    // Đợi một chút để PLC có thời gian kết nối
    std::this_thread::sleep_for(std::chrono::seconds(5));

    while (global_running) {
        // Ghi giá trị 1 vào D200 mỗi 200ms
        plc_command_queue.push("WRITE_D200_1");

        // Ngủ 200ms
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    LOG_INFO << "[PLC Periodic] Thread stopped.";
}


/**
 * @brief PLC Thread Function (UPDATED)
 */
void plc_thread_func(SystemState& state,
                    ThreadSafeQueue<std::string>& command_queue,
                    ThreadSafeQueue<std::string>& result_queue) {
    
    LOG_REGISTER_CONTEXT("PLC", "PLC Communication Thread");
    LOG_SET_CONTEXT("PLC");
    
    LOG_INFO << "[PLC Thread] Starting...";
    
    // Create MC Protocol client instance
    auto plc = std::make_shared<MCProtocol>(PLC_IP, PLC_PORT);
    
    // Try to establish connection to PLC
    bool connection_established = false;
    
    for (int attempt = 1; attempt <= 3 && !connection_established && global_running; ++attempt) {
        LOG_INFO << "[PLC Thread] Connection attempt " << attempt << "/3";
        if (plc->connect()) {
            connection_established = true;
            std::lock_guard<std::mutex> lock(plc_ptr_mutex);
            global_plc_ptr = plc;
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.plc_connected = true;
                state.last_plc_status = "PLC connected";
            }
            LOG_INFO << "[PLC Thread] Connected to PLC";
            
            // 1. Gửi lệnh khởi tạo D110=1 khi kết nối thành công lần đầu
            LOG_INFO << "[PLC Thread] PLC connected. Sending initial command WRITE_D110_1.";
            command_queue.push("WRITE_D110_1");

            // 2. Bắt đầu giám sát thanh ghi D102 để xử lý logic reset
            plc->monitorRegister("D", 102, 
                [&command_queue](const std::string& dev, uint32_t addr, uint16_t old_val, uint16_t new_val) {
                    LOG_INFO << "[PLC Monitor] " << dev << addr 
                             << " changed from " << old_val 
                             << " to " << new_val;

                    // Biến static để lưu trạng thái lỗi trước đó
                    static bool d102_was_5 = false;

                    // Nếu D102 = 5 (trạng thái lỗi)
                    if (new_val == 5) {
                        LOG_WARNING << "[PLC Monitor] D102 is 5 (Error State). Sending WRITE_D110_2 to acknowledge.";
                        plc_in_error_state = true; // SET CỜ LỖI
                        command_queue.push("WRITE_D110_2");
                        d102_was_5 = true;
                    } 
                    // Nếu D102 = 0 và trước đó nó đã là 5
                    else if (new_val == 0 && d102_was_5) {
                        LOG_INFO << "[PLC Monitor] D102 is 0 after error. Sending WRITE_D110_1 to restart.";
                        plc_in_error_state = false; // XÓA CỜ LỖI
                        command_queue.push("WRITE_D110_1");
                        d102_was_5 = false; // Reset cờ
                    }
                }, 250); // Kiểm tra mỗi 250ms

        } else {
            LOG_ERROR << "[PLC Thread] Failed to connect, attempt " << attempt;
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
    
    // If connection failed, run in simulation mode
    if (!connection_established) {
        LOG_WARNING << "[PLC Thread] Running in simulation mode (PLC not connected)";
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.plc_connected = false;
            state.last_plc_status = "Running in simulation mode";
        }
    }
    
    // Main processing loop
    int command_count = 0;
    while (global_running) {
        std::string command;
        
        if (command_queue.pop(command, 100)) {
            command_count++;
            LOG_INFO << "[PLC Thread] Processing command #" << command_count << ": " << command;
            
            std::string plc_response;
            
            if (connection_established && plc->isConnected()) {
                try {
                    plc_response = parseAndExecutePlcCommand(command, *plc);
                } catch (const std::exception& e) {
                    plc_response = "PLC Error: " + std::string(e.what());
                    LOG_ERROR << "[PLC Thread] Error: " << e.what();
                }
            } else {
                // Simulation mode
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                plc_response = "[SIM] OK for '" + command + "' (#" + 
                              std::to_string(command_count) + ")";
            }
            
            // Update system state
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.last_plc_status = plc_response;
            }
            
            // Send response back
            result_queue.push(plc_response);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Cleanup
    if (connection_established) {
        plc->disconnect();
        
        // Clear global pointer
        {
            std::lock_guard<std::mutex> lock(plc_ptr_mutex);
            global_plc_ptr.reset();
        }
    }
    
    LOG_INFO << "[PLC Thread] Stopped. Total commands processed: " << command_count;
}

/**
 * @brief Luồng quản lý keyboard - lắng nghe bàn phím và điều phối các worker
 */
void keyboard_control_thread(ThreadSafeQueue<std::string>& plc_command_queue,
                            ThreadSafeQueue<std::string>& plc_result_queue,
                            SystemState& system_state) {
    
    
    if (!system_initialized) {
        initializeSystem(plc_command_queue, plc_result_queue);
        std::string result;
        plc_result_queue.pop(result, 1000);
    }
    
    LOG_INFO << "[Keyboard Manager] Started. Controls:";
    LOG_INFO << "  W/↑ : Move Forward";
    LOG_INFO << "  S/↓ : Move Backward";
    LOG_INFO << "  A/← : Turn Left";
    LOG_INFO << "  D/→ : Turn Right";
    LOG_INFO << "  0   : Emergency STOP";
    LOG_INFO << "  ESC : Exit program";
        
    SimpleKeyboardListener listener;
    
    // Statistics tracking
    int total_commands = 0;
    auto session_start = std::chrono::steady_clock::now();
    
    while (global_running) {
        char key = listener.getChar();
        
        if (key != 0) {
            bool is_safe = true;
            float front_distance = -1.0f;
            {
                std::lock_guard<std::mutex> lock(system_state.state_mutex);
                is_safe = system_state.is_safe_to_move;
                front_distance = system_state.current_front_distance;
            }
            
            // --- LOGIC MỚI ---

            switch (key) {

                case 'W': {
                    std::string cmd = "WRITE_D100_1";
    
                    // Lưu lệnh di chuyển
                    {
                       std::lock_guard<std::mutex> lock(last_command_mutex);
                       last_movement_command = cmd;
                    } 

                    if (plc_in_error_state) {
                        LOG_ERROR << "[Keyboard] Movement disabled! PLC is in error state (D102=5). Waiting for reset.";
                        break;
                    }

                    if (is_safe) {
                        LOG_INFO << "Moving FORWARD";

                        // SET CỜ movement_command_active
                        {
                            std::lock_guard<std::mutex> lock(system_state.state_mutex);
                            system_state.movement_command_active = true;
                            system_state.is_moving = false;  // Reset để bắt đầu smooth acceleration
                            system_state.target_speed = 0;
                        }
                        
                        plc_command_queue.push("WRITE_D101_0");
                        plc_command_queue.push(cmd);
                        
                        total_commands++;
                    } else {
                        LOG_WARNING << "Cannot move forward - obstacle at " << front_distance 
                                    << "cm! Will auto-start when clear.";
                    }
                    break;
                
                } // <--- Dấu ngoặc bị thiếu được thêm vào đây
                
                case 'S': {
                    LOG_INFO << "→ Moving BACKWARD";
                    if (plc_in_error_state) {
                        LOG_ERROR << "[Keyboard] Movement disabled! PLC is in error state (D102=5). Waiting for reset.";
                        break;
                    }

                    std::string cmd = "WRITE_D100_2";
                    plc_command_queue.push(cmd);
                    
                    {
                        std::lock_guard<std::mutex> lock(last_command_mutex);
                        last_movement_command = cmd;
                    }
                    // SET CỜ movement_command_active cho lùi
                    {
                        std::lock_guard<std::mutex> lock(system_state.state_mutex);
                        system_state.movement_command_active = true;
                        system_state.is_moving = false;
                        system_state.target_speed = 0;
                    }

                    total_commands++;
                    break;
                }
                
                case 'A':
                case 'D': {

                    if (plc_in_error_state) {
                        LOG_ERROR << "[Keyboard] Movement disabled! PLC is in error state (D102=5). Waiting for reset.";
                        break;
                    }

                    if (!is_safe) {
                        LOG_WARNING << "Cannot turn - obstacle at " << front_distance << "cm";
                        break;
                    }

                    LOG_INFO << "→ Turning " << (key == 'A' ? "LEFT" : "RIGHT");
                    
                    // SET CỜ movement_command_active cho xoay
                    {
                        std::lock_guard<std::mutex> lock(system_state.state_mutex);
                        system_state.movement_command_active = true;
                        system_state.is_moving = false;
                        system_state.target_speed = 0;
                    }
                    
                    plc_command_queue.push("WRITE_D100_1");
                    plc_command_queue.push((key == 'A') ? "WRITE_D101_1" : "WRITE_D101_2");
                    
                    total_commands++;
                    break;
                } // <--- Dấu ngoặc bị thiếu được thêm vào đây
                
                case '0': {
                    LOG_INFO << "[Keyboard] EMERGENCY STOP (0) pressed";
                    plc_command_queue.push("WRITE_D100_0");
                    plc_command_queue.push("WRITE_D101_0");
                    
                    {
                        std::lock_guard<std::mutex> lock(last_command_mutex);
                        last_movement_command.clear();
                    }
                    // CLEAR CỜ movement_command_active
                    {
                        std::lock_guard<std::mutex> lock(system_state.state_mutex);
                        system_state.movement_command_active = false;
                        system_state.is_moving = false;
                        system_state.current_speed = 0;
                    }

                    break;
                }
                
                case 27: { // ESC key
                    global_running = false;
                    break;
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    // Cleanup khi thoát
    plc_command_queue.push("WRITE_D100_0");
    plc_command_queue.push("WRITE_D101_0");

    {
        std::lock_guard<std::mutex> lock(last_command_mutex);
        last_movement_command.clear();
    }
    
    auto session_duration = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - session_start);
    
    LOG_INFO << "[Keyboard] Session ended. Total commands: " << total_commands 
            << " in " << session_duration.count() << " seconds";
}


// LIDAR THREAD WITH SLAM PROCESSING
void lidar_thread_func(
    SystemState& state,
    ThreadSafeQueue<std::vector<Point2D>>& points_queue, 
    ThreadSafeQueue<std::string>& plc_command_queue)
{

    LOG_INFO << "[LiDAR Thread] Starting...";

    // Khởi tạo LidarProcessor với các IP/Port mặc định
    auto lidar_processor = std::make_unique<LidarProcessor>(LIDAR_HOST_IP, std::to_string(LIDAR_PORT),
                                                          LIDAR_CLIENT_IP, std::to_string(LIDAR_CLIENT_PORT));

    if (!lidar_processor->initialize() || !lidar_processor->start()) {
        LOG_ERROR << "[LiDAR Thread] Failed to initialize/start LidarProcessor";
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.lidar_connected = false;
            state.last_lidar_data = "Lidar initialization failed";
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
        
        if (min_front >= 999.0f || min_front <= 0.0f) return;
        // Xử lý phản hồi NHANH cho vật cản phía trước
        // 1. Chuyển đổi khoảng cách từ mét sang centimet.
        float min_dist_cm = min_front * 100.0f;
        int smooth_speed = 0;
        bool movement_active = false;

        // 2. Cập nhật trạng thái hệ thống một cách an toàn (thread-safe).
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.current_front_distance = min_dist_cm;
            state.is_safe_to_move = min_dist_cm > EMERGENCY_STOP_DISTANCE_CM;
            state.last_safety_update = std::chrono::steady_clock::now().time_since_epoch().count();
            state.last_lidar_data = "[RT] Front: " + std::to_string(min_dist_cm) + "cm | L: " +
                                   std::to_string(static_cast<int>(min_left * 100)) + "cm | R: " +
                                   std::to_string(static_cast<int>(min_right * 100)) + "cm";
            
            // ĐỌC CỜ movement_command_active
            movement_active = state.movement_command_active;
            
            // CHỈ tính toán tốc độ khi:
            // 1. An toàn di chuyển
            // 2. CÓ lệnh di chuyển đang active
            if (state.is_safe_to_move && movement_active) {
                LOG_INFO << "[Speed Control] Movement command ACTIVE - calculating speed for distance: " << min_dist_cm << "cm";
                smooth_speed = calculateSmoothSpeed(state, min_dist_cm, true);
            } else if (!movement_active) {
                // Nếu không có lệnh di chuyển, reset tốc độ
                smooth_speed = 0;
                state.is_moving = false;
                state.current_speed = 0;
                LOG_DEBUG << "[Speed Control] No movement command - speed set to 0";
            } else if (!state.is_safe_to_move) {
                // Nếu không an toàn, cũng reset
                smooth_speed = 0;
                LOG_WARNING << "[Speed Control] Unsafe to move - speed set to 0";
            }
        }
        
        // Gửi lệnh tốc độ
        static int last_sent_speed = -1;
        if (smooth_speed != last_sent_speed) {
            plc_command_queue.push("WRITE_D103_" + std::to_string(smooth_speed));
            last_sent_speed = smooth_speed;
            LOG_INFO << "[Speed Control] Sent D103=" << smooth_speed << " (Distance: " << min_dist_cm << "cm, Movement: " << (movement_active ? "ACTIVE" : "IDLE") << ")";
        }
        
        LOG_INFO << (min_dist_cm > EMERGENCY_STOP_DISTANCE_CM ? "[REALTIME] Path clear: " : "[REALTIME] Obstacle: ") << min_dist_cm << "cm";
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


    // Cập nhật trạng thái thành công
    {
        std::lock_guard<std::mutex> lock(state.state_mutex);
        state.lidar_connected = true;
        state.last_lidar_data = "Lidar connected - Dual mode active";
    }

    LOG_INFO << "[LiDAR Thread] Successfully started with dual-mode processing:"; 
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
/**
 * @brief Thread giao tiếp với server
 */
void server_communication_thread(SystemState& state,
                                ThreadSafeQueue<std::vector<Point2D>>& lidar_points_queue) {
    
    LOG_INFO << "[Server Comm] Starting communication thread...";
    
    // Tạo CommunicationServer instance
    auto comm_server = std::make_unique<ServerComm::CommunicationServer>(
        SERVER_IP, SERVER_PORT, COMM_SEND_INTERVAL_MS);
    
    // Đăng ký callback để lấy status từ hệ thống
    comm_server->setStatusCallback([&state]() -> ServerComm::AGVStatusPacket {
        ServerComm::AGVStatusPacket packet;
        
        // Lock state để đọc dữ liệu an toàn
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            
            // Vị trí hiện tại (giả định có thêm trong SystemState)
            packet.current_x = 0.0f;  // TODO: Lấy từ odometry/SLAM
            packet.current_y = 0.0f;
            packet.current_angle = 0.0f;
            
            // Trạng thái PLC - Đọc các thanh ghi quan trọng
            packet.plc_registers["D100"] = 0;  // Direction
            packet.plc_registers["D101"] = 0;  // Turn
            packet.plc_registers["D102"] = 0;  // Status
            packet.plc_registers["D103"] = state.current_speed;  // Speed
            packet.plc_registers["D110"] = 0;  // System state
            
            // Copy LiDAR points
            packet.lidar_points.reserve(state.latest_convex_hull.size());
            for (const auto& p : state.latest_convex_hull) {
                packet.lidar_points.push_back(p);
            }
            
            // System status
            packet.is_moving = state.is_moving;
            packet.is_safe = state.is_safe_to_move;
            packet.battery_level = state.battery_level;
            packet.current_speed = state.current_speed;
            packet.timestamp = state.last_safety_update;
        }
        
        return packet;
    });
    
    // Đăng ký callback xử lý lệnh từ server
    // comm_server->setCommandCallback([&state, &plc_command_queue](const NavigationCommand& cmd) {
    //     LOG_INFO << "[Server Comm] Received command type: " << cmd.type;
        
    //     switch (cmd.type) {
    //         case NavigationCommand::MOVE_TO_POINT: {
    //             LOG_INFO << "[Server Comm] Move to point: (" 
    //                     << cmd.target_x << ", " << cmd.target_y << ")";
                
    //             // TODO: Implement path planning and movement
    //             // Tính toán góc cần xoay
    //             float angle_to_target = atan2(cmd.target_y, cmd.target_x);
                
    //             // Gửi lệnh PLC để di chuyển
    //             if (cmd.speed > 0) {
    //                 plc_command_queue.push("WRITE_D103_" + std::to_string((int)cmd.speed));
    //                 plc_command_queue.push("WRITE_D100_1");  // Forward
    //             }
    //             break;
    //         }
            
    //         case NavigationCommand::ROTATE_TO_ANGLE: {
    //             LOG_INFO << "[Server Comm] Rotate to angle: " << cmd.target_angle;
                
    //             // Xác định hướng xoay
    //             // TODO: Tính toán dựa trên góc hiện tại
    //             plc_command_queue.push("WRITE_D100_1");
    //             plc_command_queue.push("WRITE_D101_1"); // Turn left or right
    //             break;
    //         }
            
    //         case NavigationCommand::FOLLOW_PATH: {
    //             LOG_INFO << "[Server Comm] Follow path with " << cmd.path.size() << " points";
    //             // TODO: Implement path following algorithm
    //             break;
    //         }
            
    //         case NavigationCommand::STOP: {
    //             LOG_INFO << "[Server Comm] Stop command";
    //             plc_command_queue.push("WRITE_D100_0");
    //             plc_command_queue.push("WRITE_D101_0");
    //             break;
    //         }
            
    //         case NavigationCommand::EMERGENCY_STOP: {
    //             LOG_WARNING << "[Server Comm] Emergency stop!";
    //             plc_command_queue.push("WRITE_D100_0");
    //             plc_command_queue.push("WRITE_D101_0");
                
    //             // Clear movement state
    //             {
    //                 std::lock_guard<std::mutex> lock(state.state_mutex);
    //                 state.movement_command_active = false;
    //                 state.is_moving = false;
    //                 state.current_speed = 0;
    //             }
    //             break;
    //         }
    //     }
    // });
    
    // Đăng ký callback kết nối
    comm_server->setConnectionCallback([&state](bool connected) {
        LOG_INFO << "[Server Comm] Connection status: " << (connected ? "Connected" : "Disconnected");
        
        // Update system state
        std::lock_guard<std::mutex> lock(state.state_mutex);
        // Có thể thêm biến server_connected vào SystemState
    });
    
    // Bắt đầu giao tiếp
    if (!comm_server->start()) {
        LOG_ERROR << "[Server Comm] Failed to start communication";
        return;
    }
    
    LOG_INFO << "[Server Comm] Successfully connected to server";
    
    // Main loop - xử lý dữ liệu LiDAR để gửi lên server
    while (global_running) {
        // Gửi LiDAR points khi có dữ liệu mới
        std::vector<Point2D> points;
        if (lidar_points_queue.pop(points, 100)) {
            comm_server->sendLidarPoints(points);
            LOG_DEBUG << "[Server Comm] Sent " << points.size() << " LiDAR points";
        }
        
        // Định kỳ gửi PLC registers (mỗi 500ms)
        static auto last_plc_send = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_plc_send).count() >= 500) {
            // Lấy giá trị PLC từ global pointer
            std::map<std::string, uint16_t> plc_data;
            
            {
                std::lock_guard<std::mutex> lock(plc_ptr_mutex);
                if (global_plc_ptr && global_plc_ptr->isConnected()) {
                    try {
                        // Đọc các thanh ghi quan trọng
                        plc_data["D100"] = global_plc_ptr->readSingleWord("D", 100);
                        plc_data["D101"] = global_plc_ptr->readSingleWord("D", 101);
                        plc_data["D102"] = global_plc_ptr->readSingleWord("D", 102);
                        plc_data["D103"] = global_plc_ptr->readSingleWord("D", 103);
                        plc_data["D110"] = global_plc_ptr->readSingleWord("D", 110);
                        plc_data["D200"] = global_plc_ptr->readSingleWord("D", 200);
                    } catch (const std::exception& e) {
                        LOG_ERROR << "[Server Comm] Failed to read PLC: " << e.what();
                    }
                }
            }
            
            if (!plc_data.empty()) {
                comm_server->sendPLCRegisters(plc_data);
                LOG_DEBUG << "[Server Comm] Sent PLC registers update";
            }
            
            last_plc_send = now;
        }
        
        // Kiểm tra lệnh từ server
        ServerComm::NavigationCommand cmd;
        while (comm_server->getNextCommand(cmd)) {
            LOG_INFO << "[Server Comm] Processing queued command type: " << cmd.type;
            // Command đã được xử lý trong callback, chỉ log ở đây
        }
        
        // In thống kê định kỳ
        static auto last_stats = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_stats).count() >= 10) {
            LOG_INFO << "[Server Comm] Stats - Sent: " << comm_server->getTotalPacketsSent()
                    << " Received: " << comm_server->getTotalPacketsReceived()
                    << " Data rate: " << comm_server->getDataRate() << " KB/s";
            last_stats = now;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Cleanup
    comm_server->stop();
    LOG_INFO << "[Server Comm] Thread stopped";
}


// HÀM MAIN
int main() {
    // Initialize logging system
    // Register MAIN app and context
    LOG_REGISTER_APP("MAIN", "Main AGV Application");
    LOG_REGISTER_CONTEXT("MAIN", "Main Control Context");
    LOG_SET_APP("MAIN");
    LOG_SET_CONTEXT("MAIN");
    LOG_INFO << "[Main Thread] Control system starting...";



    SystemState shared_state;

    ThreadSafeQueue<std::string> plc_command_queue;
    ThreadSafeQueue<std::string> plc_result_queue;

    // Start PLC thread
    std::thread plc_thread(plc_thread_func, 
                          std::ref(shared_state), 
                          std::ref(plc_command_queue), 
                          std::ref(plc_result_queue));
    
    // Start LiDAR thread  
    std::thread lidar_thread(lidar_thread_func, 
                            std::ref(shared_state),
                            std::ref(stable_points_queue), 
                            std::ref(plc_command_queue));
    
    // Start Keyboard thread - KHÔNG TRUYỀN plc_ptr
    std::thread keyboard_thread(keyboard_control_thread, 
                               std::ref(plc_command_queue),
                               std::ref(plc_result_queue),
                               std::ref(shared_state));

    std::thread safety_thread(safety_monitor_thread,
                             std::ref(plc_command_queue),
                             std::ref(shared_state));

    // Start PLC periodic tasks thread
    std::thread plc_periodic_thread(plc_periodic_tasks_thread,
                                   std::ref(plc_command_queue));

    //Server thread
    std::thread server_thread(server_communication_thread,
                            std::ref(shared_state),
                            std::ref(stable_points_queue));
    
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

            // Kiểm tra global PLC pointer
            {
                std::lock_guard<std::mutex> plc_lock(plc_ptr_mutex);
                LOG_INFO << "PLC Pointer Valid: " << (global_plc_ptr ? "Yes" : "No");
            }

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

    LOG_INFO << "[Main Thread] Initiating shutdown...";
    
    // Stop all threads gracefully
    global_running = false;
    
    // Join all threads
     for (auto& thread : {&plc_thread, &lidar_thread, &keyboard_thread, &safety_thread, &plc_periodic_thread, &server_thread}) {
        if (thread->joinable()) {
            thread->join();
            LOG_INFO << "[Main Thread] Thread joined";
        }
    }

    LOG_INFO << "[Main Thread] System shutdown complete.";
    return 0;
}