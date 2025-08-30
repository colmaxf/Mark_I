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

// SYSTEM STATE STRUCT
struct SystemState {
    std::mutex state_mutex;

     // Battery data
    double battery_level = 0.0;
    
    // PLC data
    std::string last_lidar_data = "Chưa có dữ liệu Lidar";
    bool plc_connected = false;

    // LiDAR data
    std::string last_lidar_data = "Chưa có dữ liệu Lidar";
    bool lidar_connected = false;
    std::vector<Point2D> latest_convex_hull;
    RobotPose current_robot_pose;
    int total_stable_hulls = 0;
    int total_scan_count = 0;
    
    // SLAM mapping data
    std::vector<std::vector<Point2D>> global_map_hulls;
    long last_hull_timestamp = 0;

};

// Queue để truyền convex hull giữa các thread
// Communication queues
ThreadSafeQueue<std::vector<Point2D>> convex_hull_queue;
ThreadSafeQueue<RobotPose> pose_queue;
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

// LIDAR THREAD WITH SLAM PROCESSING
void lidar_thread_func(SystemState& state) {
#ifdef ENABLE_LOG
    LOG_INFO << "[LiDAR Thread] Starting SLAM LiDAR processing...";
#else
    std::cout << "[LiDAR Thread] Starting SLAM LiDAR processing..." << std::endl;
#endif

    // Initialize LiDAR processor
    LidarProcessor lidar_processor("192.168.3.10", "2368", "192.168.3.101", "2368");
    
    if (!lidar_processor.initialize()) {
        std::cerr << "[LiDAR Thread] Failed to initialize LiDAR processor!" << std::endl;
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.lidar_connected = false;
            state.last_lidar_data = "LiDAR initialization failed";
        }
        return;
    }

    // Set up callbacks for hull and map updates
    lidar_processor.set_hull_callback([&state](const std::vector<Point2D>& hull, const RobotPose& pose) {
        // Update system state with new stable hull
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.latest_convex_hull = hull;
            state.current_robot_pose = pose;
            state.total_stable_hulls++;
            state.last_hull_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        }
        
        // Send hull data to queue for other threads
        convex_hull_queue.push(hull);
        pose_queue.push(pose);
        
#ifdef ENABLE_LOG
        LOG_INFO << "[LiDAR Thread] New stable hull with " << hull.size() << " points";
#else
        std::cout << "[LiDAR Thread] New stable hull with " << hull.size() << " points at pose (" 
                  << std::fixed << std::setprecision(2) << pose.x << ", " << pose.y << ")" << std::endl;
#endif
    });

    lidar_processor.set_map_callback([&state](const std::vector<std::vector<Point2D>>& map_hulls) {
        // Update global map
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.global_map_hulls = map_hulls;
        }
        
#ifdef ENABLE_LOG
        LOG_INFO << "[LiDAR Thread] Map updated with " << map_hulls.size() << " hull regions";
#else
        std::cout << "[LiDAR Thread] Map updated with " << map_hulls.size() << " hull regions" << std::endl;
#endif
    });

    // Start processing
    if (!lidar_processor.start_processing()) {
        std::cerr << "[LiDAR Thread] Failed to start LiDAR processing!" << std::endl;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(state.state_mutex);
        state.lidar_connected = true;
        state.last_lidar_data = "LiDAR SLAM processing active";
    }

    // Status update timer
    auto last_status_time = std::chrono::steady_clock::now();
    const auto status_interval = std::chrono::seconds(5);

    // Main processing loop
    std::thread lidar_processing_thread([&lidar_processor]() {
        lidar_processor.process_slam_lidar_data();
    });

    // Status monitoring loop
    while (global_running && lidar_processor.is_slam_active()) {
        auto current_time = std::chrono::steady_clock::now();
        
        // Update status periodically
        if (current_time - last_status_time >= status_interval) {
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.total_scan_count = lidar_processor.get_scan_count();
                state.current_robot_pose = lidar_processor.get_current_pose();
                
                std::string status = "Scans: " + std::to_string(state.total_scan_count) + 
                                   ", Stable hulls: " + std::to_string(state.total_stable_hulls) +
                                   ", Pose: (" + std::to_string(state.current_robot_pose.x) + 
                                   ", " + std::to_string(state.current_robot_pose.y) + ")";
                state.last_lidar_data = status;
            }
            
            // Send status update
            lidar_status_queue.push("STATUS_UPDATE");
            last_status_time = current_time;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Clean shutdown
    lidar_processor.stop_processing();
    
    if (lidar_processing_thread.joinable()) {
        lidar_processing_thread.join();
    }

    {
        std::lock_guard<std::mutex> lock(state.state_mutex);
        state.lidar_connected = false;
        state.last_lidar_data = "LiDAR processing stopped";
    }

#ifdef ENABLE_LOG
    LOG_INFO << "[LiDAR Thread] Shutting down...";
#else
    std::cout << "[LiDAR Thread] Shutting down..." << std::endl;
#endif
}

// DATA PROCESSING THREAD - Consumes convex hull data
void data_processing_thread_func(SystemState& state) {
#ifdef ENABLE_LOG
    LOG_INFO << "[Data Processing Thread] Starting...";
#else
    std::cout << "[Data Processing Thread] Starting..." << std::endl;
#endif

    while (global_running) {
        std::vector<Point2D> hull_data;
        RobotPose pose_data;
        
        // Wait for new convex hull data
        if (convex_hull_queue.pop(hull_data, 1000)) { // 1 second timeout
            // Try to get corresponding pose data
            if (pose_queue.pop(pose_data, 100)) {
#ifdef ENABLE_LOG
                LOG_INFO << "[Data Processing] Processing hull with " << hull_data.size() << " points";
#else
                std::cout << "[Data Processing] Processing hull with " << hull_data.size() 
                          << " points at pose (" << pose_data.x << ", " << pose_data.y << ")" << std::endl;
#endif

                // Example processing: Calculate hull area
                if (hull_data.size() >= 3) {
                    float area = 0.0f;
                    for (size_t i = 0; i < hull_data.size(); i++) {
                        size_t j = (i + 1) % hull_data.size();
                        area += hull_data[i].x * hull_data[j].y - hull_data[j].x * hull_data[i].y;
                    }
                    area = std::abs(area) / 2.0f;
                    
                    std::cout << "[Data Processing] Hull area: " << std::fixed << std::setprecision(3) 
                              << area << " m²" << std::endl;
                }

                // Example: Send processed data to other systems
                // You can add your custom processing logic here
                
                // Example: Detect if robot is in a specific region
                bool in_target_region = (pose_data.x > 1.0 && pose_data.x < 5.0 && 
                                       pose_data.y > 1.0 && pose_data.y < 5.0);
                
                if (in_target_region) {
                    std::cout << "[Data Processing] Robot is in target region!" << std::endl;
                    // Could trigger PLC command or other actions
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

#ifdef ENABLE_LOG
    LOG_INFO << "[Data Processing Thread] Shutting down...";
#else
    std::cout << "[Data Processing Thread] Shutting down..." << std::endl;
#endif
}

// MONITORING THREAD - Displays system status
void monitoring_thread_func(SystemState& state) {
#ifdef ENABLE_LOG
    LOG_INFO << "[Monitoring Thread] Starting...";
#else
    std::cout << "[Monitoring Thread] Starting..." << std::endl;
#endif

    auto last_display_time = std::chrono::steady_clock::now();
    const auto display_interval = std::chrono::seconds(10);

    while (global_running) {
        auto current_time = std::chrono::steady_clock::now();
        
        if (current_time - last_display_time >= display_interval) {
            std::cout << "\n========== SYSTEM STATUS ===========" << std::endl;
            
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                std::cout << "PLC Status: " << (state.plc_connected ? "CONNECTED" : "DISCONNECTED") << std::endl;
                std::cout << "LiDAR Status: " << (state.lidar_connected ? "CONNECTED" : "DISCONNECTED") << std::endl;
                std::cout << "Total Scans: " << state.total_scan_count << std::endl;
                std::cout << "Stable Hulls: " << state.total_stable_hulls << std::endl;
                std::cout << "Current Hull Points: " << state.latest_convex_hull.size() << std::endl;
                std::cout << "Robot Pose: (" << std::fixed << std::setprecision(2) 
                          << state.current_robot_pose.x << ", " 
                          << state.current_robot_pose.y << ", " 
                          << state.current_robot_pose.theta << ")" << std::endl;
                std::cout << "Queue Sizes - Hull: " << convex_hull_queue.size() 
                          << ", Pose: " << pose_queue.size() << std::endl;
            }
            
            std::cout << "==================================\n" << std::endl;
            last_display_time = current_time;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

#ifdef ENABLE_LOG
    LOG_INFO << "[Monitoring Thread] Shutting down...";
#else
    std::cout << "[Monitoring Thread] Shutting down..." << std::endl;
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
    std::thread plc_thread(plc_thread_func, std::ref(shared_state), 
                          std::ref(plc_command_queue), std::ref(plc_result_queue));
    
    std::thread lidar_thread(lidar_thread_func, std::ref(shared_state));
    
    std::thread data_processing_thread(data_processing_thread_func, std::ref(shared_state));
    
    std::thread monitoring_thread(monitoring_thread_func, std::ref(shared_state));

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
    auto last_command_time = std::chrono::steady_clock::now();
    const auto command_interval = std::chrono::seconds(5);

    while (global_running) {
        auto current_time = std::chrono::steady_clock::now();
        
        // Send test commands periodically
        if (current_time - last_command_time >= command_interval) {
            std::string command = test_commands[command_index % total_commands];
            std::cout << "\n[Main Thread] Sending PLC command: " << command << std::endl;
            plc_command_queue.push(command);

            // Wait for response
            std::string result;
            if (plc_result_queue.pop(result, 3000)) { // 3 second timeout
                std::cout << "[Main Thread] PLC Response: " << result << std::endl;
            } else {
                std::cout << "[Main Thread] Timeout waiting for PLC response!" << std::endl;
            }

            command_index++;
            last_command_time = current_time;
        }
        
        // Check for status updates from LiDAR
        std::string lidar_status;
        while (lidar_status_queue.pop(lidar_status, 10)) { // Non-blocking check
            std::cout << "[Main Thread] LiDAR Status Update: " << lidar_status << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

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
    
    if (data_processing_thread.joinable()) {
        data_processing_thread.join();
        std::cout << "[Main Thread] Data processing thread stopped." << std::endl;
    }
    
    if (monitoring_thread.joinable()) {
        monitoring_thread.join();
        std::cout << "[Main Thread] Monitoring thread stopped." << std::endl;
    }

    std::cout << "[Main Thread] System shutdown complete." << std::endl;
    return 0;
}