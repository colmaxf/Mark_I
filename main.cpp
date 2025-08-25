#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>
#include <memory>

#include "config.h"
#ifdef ENABLE_LOG
#include "logger/Logger.h"
#endif

#include "MCprotocollib/MCprotocol.h"

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

// SYSTEM STATE STRUCT
struct SystemState {
    std::mutex state_mutex;
    double battery_level = 0.0;
    std::string last_lidar_data = "Chưa có dữ liệu Lidar";
    std::string last_plc_status = "Chưa kết nối PLC";
    bool plc_connected = false;
};

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
    MCProtocol plc("192.168.3.5", 5000);
    
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
                    if (command.find("READ_D") != std::string::npos) {
                        // Example: READ_D100 -> read D register 100
                        size_t pos = command.find("READ_D");
                        if (pos != std::string::npos) {
                            std::string addr_str = command.substr(pos + 6);
                            uint32_t addr = std::stoul(addr_str);
                            uint16_t value = plc.readSingleWord("D", addr);
                            plc_response = "D" + addr_str + " = " + std::to_string(value);
                        }
                    } else if (command.find("WRITE_D") != std::string::npos) {
                        // Example: WRITE_D100_1234 -> write 1234 to D100
                        size_t pos1 = command.find("WRITE_D");
                        size_t pos2 = command.find("_", pos1 + 7);
                        if (pos1 != std::string::npos && pos2 != std::string::npos) {
                            std::string addr_str = command.substr(pos1 + 7, pos2 - pos1 - 7);
                            std::string value_str = command.substr(pos2 + 1);
                            uint32_t addr = std::stoul(addr_str);
                            uint16_t value = std::stoul(value_str);
                            bool success = plc.writeSingleWord("D", addr, value);
                            plc_response = success ? "Write D" + addr_str + " = " + value_str + " OK" 
                                                  : "Write D" + addr_str + " FAILED";
                        }
                    } else {
                        plc_response = "Unknown command: " + command;
                    }
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

// Luồng xử lý Lidar
// void lidar_thread_func(SystemState& state) {
//     std::cout << "[Lidar Thread] Khởi động." << std::endl;
//     // --- Placeholder: Logic kết nối đến Lidar TCP Server ---
//     while (true) {
//         // --- Placeholder: Nhận và xử lý dữ liệu từ Lidar ---
//         std::string raw_lidar_data = "Dữ liệu Lidar tại " + std::to_string(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
//         {
//             std::lock_guard<std::mutex> lock(state.state_mutex);
//             state.last_lidar_data = raw_lidar_data;
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
// }

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

    std::thread plc_thread(plc_thread_func, std::ref(shared_state), std::ref(plc_command_queue), std::ref(plc_result_queue));
    // Test commands
    std::vector<std::string> test_commands = {
        "READ_D100",
        "READ_D101",
        "WRITE_D100_1234",
        "READ_D100",
        "WRITE_D101_5678",
        "READ_D101",
        "INVALID_COMMAND"
    };

    int command_index = 0;
    int total_commands = test_commands.size();

    while (true) {
        // Send test commands in sequence
        std::string command = test_commands[command_index % total_commands];
        std::cout << "\n[Main Thread] Sending command: " << command << std::endl;
        plc_command_queue.push(command);

        // Wait for response
        std::string result;
        if (plc_result_queue.pop(result, 5000)) { // 5 second timeout
            std::cout << "[Main Thread] Received response: " << result << std::endl;
        } else {
            std::cout << "[Main Thread] Timeout waiting for response!" << std::endl;
        }

        // Print current system state
        {
            std::lock_guard<std::mutex> lock(shared_state.state_mutex);
            std::cout << "[Main Thread] PLC Status: " << shared_state.last_plc_status << std::endl;
            std::cout << "[Main Thread] PLC Connected: " << (shared_state.plc_connected ? "YES" : "NO") << std::endl;
        }

        command_index++;
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    // std::thread lidar_thread(lidar_thread_func, std::ref(shared_state));
    // std::thread battery_thread(battery_thread_func, std::ref(shared_state));
    // std::thread webserver_thread(webserver_thread_func, std::ref(shared_state));

    int command_count = 0;
    // while (true) {
    //     std::string command = "READ_REG_" + std::to_string(command_count++);
    //     plc_command_queue.push(command);

    //     std::string result;
    //     plc_result_queue.pop(result);
    //     std::cout << "[Main Thread] Nhận được phản hồi từ PLC: " << result << std::endl;

    //     std::this_thread::sleep_for(std::chrono::seconds(2));
    // }

    plc_thread.join();
    // lidar_thread.join();
    // battery_thread.join();
    // webserver_thread.join();

    return 0;
}
