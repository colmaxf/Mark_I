#include "SystemManager.h"
#include <sched.h>
#include <cstring>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <sstream>

/**
 * @brief Hàm khởi tạo của lớp SystemManager.
 * @details Khởi tạo đối tượng giao tiếp với server `CommunicationServer` với địa chỉ IP và cổng được cung cấp.
 * @param server_ip Địa chỉ IP của server trung tâm.
 * @param server_port Cổng của server trung tâm.
 * @param last_processed_cmd_type_ Khởi tạo loại lệnh cuối cùng đã xử lý là STOP để tránh việc bỏ qua lệnh STOP ban đầu.
 */
SystemManager::SystemManager(const std::string& server_ip, int server_port) : 
    comm_server_(std::make_unique<ServerComm::CommunicationServer>(server_ip, server_port)),
    last_processed_cmd_type_(ServerComm::NavigationCommand::STOP) // Khởi tạo lệnh cuối cùng là STOP
{}

/**
 * @brief Hàm hủy của lớp SystemManager.
 * @details Đảm bảo rằng hàm `stop()` được gọi để dừng tất cả các luồng và giải phóng tài nguyên một cách an toàn.
 */
SystemManager::~SystemManager() {
    stop();
}

/**
 * @brief Ghim luồng hiện tại vào một lõi CPU cụ thể.
 * @details Việc này giúp giảm thiểu việc chuyển đổi ngữ cảnh (context switching) và có thể cải thiện hiệu năng
 * cho các tác vụ thời gian thực hoặc yêu cầu xử lý chuyên sâu.
 * @param core_id ID của lõi CPU để ghim luồng vào (ví dụ: 0, 1, 2, 3 trên Raspberry Pi 4).
 */
void SystemManager::pin_thread_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id % 4, &cpuset); // Pi 4 has 4 cores
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0) {
        LOG_WARNING << "[SystemManager] Failed to pin thread to core " << core_id << ": " << strerror(errno);
    }
}

/**
 * @brief Khởi tạo các thành phần cốt lõi của hệ thống.
 * @details
 * - Khởi tạo con trỏ tới đối tượng giao tiếp PLC.
 * - Cố gắng kết nối tới server trung tâm, thử lại nếu thất bại.
 * - Thiết lập các hàm callback cho `CommunicationServer`:
 *   - `setStatusCallback`: Định kỳ thu thập dữ liệu trạng thái từ các module (pin, PLC, LiDAR, an toàn) và đóng gói để gửi lên server.
 *   - `setConnectionCallback`: Cập nhật trạng thái kết nối server vào `SystemState`.
 * - Bắt đầu luồng giao tiếp của server.
 * @return `true` nếu tất cả các bước khởi tạo thành công, `false` nếu có lỗi xảy ra.
 */
bool SystemManager::initialize() {
    LOG_INFO << "[SystemManager] Initializing...";

    // Initialize PLC pointer
    plc_ptr_ = std::make_shared<MCProtocol>(PLC_IP, PLC_PORT);

        // Thiết lập callback TRƯỚC KHI kết nối.
    // Điều này đảm bảo trạng thái `state_.server_connected` luôn được cập nhật chính xác,
    // ngay cả khi kết nối lại tự động.
    comm_server_->setConnectionCallback([this](bool connected) {
        LOG_INFO << "[SystemManager] Server connection status changed to: " << (connected ? "Connected" : "Disconnected");
        std::lock_guard<std::mutex> lock(state_.state_mutex);
        state_.server_connected = connected;
    });
    // Vòng lặp kết nối đến server, thử lại sau mỗi 3 giây nếu thất bại.
    // Điều này đảm bảo hệ thống có thể khởi động ngay cả khi server chưa sẵn sàng.
    while (running_ && !comm_server_->connect()) {
        LOG_WARNING << "[SystemManager] Server connection failed, retrying in 3 seconds...";
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    if (!running_) return false;

    // Setup callbacks for the server communicator
    // Callback này được gọi bởi luồng của server để lấy dữ liệu trạng thái mới nhất của AGV.
    comm_server_->setStatusCallback([this]() -> ServerComm::AGVStatusPacket {
        ServerComm::AGVStatusPacket status;
        {
            // Khóa state_mutex để đọc dữ liệu từ state_ một cách an toàn.
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            // Điền các thông tin cơ bản vào gói tin trạng thái.
            status.id_agv = AGV_ID;
            status.battery_level = static_cast<float>(state_.battery_level);
            status.is_moving = state_.is_moving;
            status.is_safe = state_.is_safe_to_move;
            status.plc_connected = state_.plc_connected;
            status.lidar_connected = state_.lidar_connected;
            status.battery_connected = state_.battery_connected;
            status.current_speed = static_cast<float>(state_.current_speed);
        }

        {
            // Khóa plc_ptr_mutex_ để truy cập con trỏ PLC một cách an toàn.
            std::lock_guard<std::mutex> lock(plc_ptr_mutex_);
            if (plc_ptr_ && plc_ptr_->isConnected()) {
                try {
                    // Đọc nhiều thanh ghi PLC cùng lúc để tăng hiệu quả giao tiếp.
                    // Đọc một khối từ D100 đến D200 (101 thanh ghi).
                    auto regs = plc_ptr_->readWords("D", 100, 101); // Read D100 to D200
                    // Kiểm tra xem có đọc đủ số lượng thanh ghi không trước khi truy cập.
                    if (regs.size() >= 101) {
                        status.plc_registers["D100"] = regs[0];
                        status.plc_registers["D101"] = regs[1];
                        status.plc_registers["D102"] = regs[2];
                        status.plc_registers["D103"] = regs[3];
                        status.plc_registers["D110"] = regs[10];
                        status.plc_registers["D200"] = regs[100];
                    }
                    //--------Đọc từng thanh ghi riêng lẻ (không hiệu quả)--------
                    // status.plc_registers["D100"] = plc_ptr_->readWord("D", 100);
                    // status.plc_registers["D101"] = plc_ptr_->readWord("D", 101);
                    // status.plc_registers["D102"] = plc_ptr_->readWord("D", 102);
                    // status.plc_registers["D103"] = plc_ptr_->readWord("D", 103);
                    // status.plc_registers["D110"] = plc_ptr_->readWord("D", 110);
                    // status.plc_registers["D200"] = plc_ptr_->readWord("D", 200);
                    //-------------------------------------------------------------

                } catch (const std::exception& e) {
                    LOG_ERROR << "[SystemManager] Failed to read PLC registers: " << e.what();
                }
            }
        }

        // Lấy dữ liệu điểm LiDAR ổn định từ hàng đợi (nếu có).
        // Lấy dữ liệu LiDAR ổn định cho mapping (chất lượng cao, ít điểm)
        // std::vector<Point2D> stable_points;
        // if (stable_points_queue_.pop(stable_points)) {
        //     status.stable_lidar_points.clear();
        //     status.stable_lidar_points.reserve(stable_points.size());
        //     for (const auto& p : stable_points) {
        //         status.stable_lidar_points.push_back({p.x, p.y});
        //     }
        //     LOG_INFO << "[SystemManager] Sending " << status.stable_lidar_points.size() 
        //              << " stable points for mapping";
        // }

        // Lấy dữ liệu LiDAR realtime cho visualization (nhiều điểm, cập nhật liên tục)
        std::vector<ServerComm::Point2D> realtime_points;
        if (realtime_points_queue_.pop(realtime_points)) {
            // Giới hạn số điểm realtime để không quá tải
            // const size_t MAX_REALTIME_POINTS = 1000;
            status.realtime_lidar_points.clear();

            // Thay vì gửi đi trực tiếp, chúng ta gọi hàm sampling để giảm số lượng điểm
            std::vector<ServerComm::Point2D> sampled_points = sampleImportantPoints(realtime_points);

            // Gán các điểm đã được sampling vào gói tin
            status.realtime_lidar_points = std::move(sampled_points);
            // for (const auto& p : sampled_points) {
            //      status.realtime_lidar_points.push_back({p.x, p.y});
            // }
            // if (realtime_points.size() > MAX_REALTIME_POINTS) {
            //     // Lấy mẫu đều để giảm số điểm
            //     size_t step = realtime_points.size() / MAX_REALTIME_POINTS;
            //     for (size_t i = 0; i < realtime_points.size(); i += step) {
            //         status.realtime_lidar_points.push_back({
            //             realtime_points[i].x, 
            //             realtime_points[i].y
            //         });
            //     }
            // } else {
                // for (const auto& p : realtime_points) {
                //     status.realtime_lidar_points.push_back({p.x, p.y});
                // }
                LOG_INFO << "[SystemManager] Sending " << status.realtime_lidar_points.size() 
                         << " realtime points for mapping";
            // }
        }
        // Gán dấu thời gian hiện tại cho gói tin.
        status.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // --- LOG dữ liệu gửi lên server ---
        std::stringstream log_ss;
        log_ss << "[SystemManager] AGVStatusPacket to server: "
               << "id_agv=" << status.id_agv
               << ", battery_level=" << status.battery_level
               << ", is_moving=" << status.is_moving
               << ", is_safe=" << status.is_safe
               << ", plc_connected=" << status.plc_connected
               << ", lidar_connected=" << status.lidar_connected
               << ", battery_connected=" << status.battery_connected
               << ", current_speed=" << status.current_speed
               << ", plc_registers={";
        for (const auto& reg : status.plc_registers) {
            log_ss << reg.first << ":" << reg.second << ",";
        }
        log_ss << "}, lidar_points=" /*<< status.stable_lidar_points.size() << "/"*/
               << status.realtime_lidar_points.size()
               << ", timestamp=" << status.timestamp;
        LOG_INFO << log_ss.str();
        // --- END LOG ---

        return status;
    });

    comm_server_->setConnectionCallback([this](bool connected) {
        // Callback được gọi khi trạng thái kết nối server thay đổi.
        LOG_INFO << "[SystemManager] Server connection status: " << (connected ? "Connected" : "Disconnected");
        std::lock_guard<std::mutex> lock(state_.state_mutex);
        state_.server_connected = connected;
    });

    if (!comm_server_->start()) {
        LOG_ERROR << "[SystemManager] Failed to start server communication";
        return false;
    }

    LOG_INFO << "[SystemManager] Initialized successfully";
    return true;
}

/**
 * @brief Bắt đầu vòng lặp hoạt động chính của hệ thống.
 * @details
 * - Gọi `initialize()` để thiết lập hệ thống.
 * - Khởi chạy tất cả các luồng xử lý chính (PLC, LiDAR, an toàn, pin, v.v.).
 * - Duy trì một vòng lặp chính để in ra trạng thái hệ thống định kỳ cho đến khi nhận được tín hiệu dừng.
 */
void SystemManager::run() {
    if (!initialize()) {
        LOG_FATAL << "[SystemManager] Initialization failed. Shutting down.";
        return;
    }

    LOG_INFO << "[SystemManager] Starting all system threads...";
    // Tạo và khởi chạy các luồng.
    threads_.emplace_back(&SystemManager::plc_thread_func, this);
    threads_.emplace_back(&SystemManager::lidar_thread_func, this);
    threads_.emplace_back(&SystemManager::command_handler_thread, this);
    threads_.emplace_back(&SystemManager::safety_monitor_thread, this);
    threads_.emplace_back(&SystemManager::plc_periodic_tasks_thread, this);
    threads_.emplace_back(&SystemManager::battery_thread_func, this);
    // Luồng server_communication_thread được quản lý bên trong lớp CommunicationServer.

    // Vòng lặp chính chỉ để giám sát và báo cáo trạng thái.
    while (running_) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        {
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            LOG_INFO << "--- SYSTEM STATUS ---";
            LOG_INFO << "PLC Connected: " << (state_.plc_connected ? "Yes" : "No");
            LOG_INFO << "LiDAR Connected: " << (state_.lidar_connected ? "Yes" : "No");
            LOG_INFO << "Server Connected: " << (state_.server_connected ? "Yes" : "No");
            LOG_INFO << "Last LiDAR Data: " << state_.last_lidar_data;
            LOG_INFO << "---------------------";
        }
    }
}

/**
 * @brief Dừng toàn bộ hệ thống một cách an toàn.
 * @details
 * - Đặt cờ `running_` thành `false` để báo hiệu cho tất cả các luồng kết thúc.
 * - Dừng module giao tiếp server.
 * - Chờ (join) tất cả các luồng con kết thúc thực thi.
 * - Xóa danh sách các luồng.
 */
void SystemManager::stop() {
    // Sử dụng exchange để đảm bảo hàm chỉ chạy một lần.
    if (!running_.exchange(false)) {
        return;
    }
    LOG_INFO << "[SystemManager] Initiating shutdown...";
    if (comm_server_) {
        comm_server_->stop(); // Gửi tín hiệu dừng đến luồng của server.
    }
    for (auto& thread : threads_) {
        if (thread.joinable()) {
            thread.join();
            LOG_INFO << "[SystemManager] Thread joined";
        }
    }
    threads_.clear();
    LOG_INFO << "[SystemManager] System shutdown complete.";
}

// --- Helper Function Implementations ---

/**
 * @brief Gửi lệnh khởi tạo hệ thống đến PLC.
 * @details Gửi lệnh `WRITE_D110_1` để báo cho PLC biết rằng phần mềm đã sẵn sàng.
 * Sử dụng cờ `system_initialized_` để đảm bảo lệnh này chỉ được gửi một lần.
 */
void SystemManager::initializeSystem() {
    if (system_initialized_.exchange(true)) {
        LOG_INFO << "[System Init] Already initialized, skipping D110_1";
        return;
    }

    LOG_INFO << "[System Init] Sending D110_1";
    plc_command_queue_.push("WRITE_D110_1");
    // Chờ phản hồi từ PLC (pop có thể bị block).
    std::string result;
    if (plc_result_queue_.pop(result)) {
        LOG_INFO << "[System Init] D110_1 response: " << result;
    } else {
        LOG_ERROR << "[System Init] No response for D110_1";
    }
}

/**
 * @brief Tính toán tốc độ mục tiêu dựa trên khoảng cách đến vật cản.
 * @details Sử dụng hàm ánh xạ tuyến tính theo từng đoạn (piecewise linear mapping) để tạo ra một
 * đường cong tốc độ mượt mà. Tốc độ tăng dần khi khoảng cách tăng.
 * @param distance Khoảng cách đến vật cản (cm).
 * @return Tốc độ mục tiêu (giá trị số nguyên để gửi cho PLC).
 */
double SystemManager::calculateSpeed(double distance) {
    distance = std::max(50.0, std::min(400.0, distance));
    if (distance >= 50.0 && distance < 100.0) {
        return 100.0 + ((distance - 50.0) * (500.0 - 100.0)) / (100.0 - 50.0);
    } else if (distance >= 100.0 && distance < 200.0) {
        return 500.0 + ((distance - 100.0) * (1500.0 - 500.0)) / (200.0 - 100.0);
    } else if (distance >= 200.0 && distance < 300.0) {
        return 1500.0 + ((distance - 200.0) * (2500.0 - 1500.0)) / (300.0 - 200.0);
    } else { // distance >= 300.0
        return 2500.0 + ((distance - 300.0) * (3000.0 - 2500.0)) / (400.0 - 300.0);
    }
}

/**
 * @brief Tính toán tốc độ di chuyển mượt mà, có gia tốc và giảm tốc.
 * @details Hàm này điều chỉnh tốc độ hiện tại của AGV để tiến tới tốc độ mục tiêu một cách từ từ,
 * thay vì thay đổi đột ngột, tạo ra chuyển động mượt mà hơn.
 * @param distance_cm Khoảng cách đến vật cản phía trước (cm), dùng để tính `target_speed`.
 * @param movement_active Cờ cho biết có lệnh di chuyển (tiến/lùi) đang hoạt động hay không.
 * @return Tốc độ đã được làm mượt để gửi đến PLC.
 */
int SystemManager::calculateSmoothSpeed(float distance_cm, bool movement_active) {
    // Nếu không có lệnh di chuyển, dừng AGV.
    if (!movement_active) {
        state_.is_moving = false;
        state_.current_speed = 0;
        return 0;
    }

    // Tính tốc độ mục tiêu dựa trên khoảng cách.
    int target = static_cast<int>(calculateSpeed(distance_cm));

    // Nếu AGV đang đứng yên và nhận lệnh di chuyển.
    if (!state_.is_moving && target > 0) {
        LOG_INFO << "[Speed Control] Starting movement towards target speed: " << target;
        state_.is_moving = true;
        state_.movement_start_time = std::chrono::steady_clock::now();
        state_.current_speed = MIN_START_SPEED; // Bắt đầu với tốc độ tối thiểu để tránh giật.
    }

    // Nếu AGV đang trong quá trình di chuyển.
    if (state_.is_moving) {
        // Tính thời gian đã trôi qua kể từ khi bắt đầu di chuyển.
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - state_.movement_start_time).count();

        // Giai đoạn tăng tốc: tăng tốc tuyến tính trong khoảng thời gian ACCEL_TIME_MS.
        if (elapsed < ACCEL_TIME_MS) {
            float t = static_cast<float>(elapsed) / ACCEL_TIME_MS;
            state_.current_speed = MIN_START_SPEED + static_cast<int>((target - MIN_START_SPEED) * t);
        } else {
            // Sau giai đoạn tăng tốc, giữ tốc độ bằng tốc độ mục tiêu.
            state_.current_speed = target;
        }
    }

    // Nếu tốc độ mục tiêu giảm xuống dưới mức tối thiểu, dừng AGV.
    if (target < MIN_START_SPEED) {
        state_.is_moving = false;
        state_.current_speed = 0;
    }

    // Trả về tốc độ hiện tại đã được làm mượt.
    return state_.current_speed;
}

/**
 * @brief Phân tích và thực thi một chuỗi lệnh trên PLC.
 * @param command Chuỗi lệnh có định dạng "ACTION_DEVICE_VALUE" (ví dụ: "WRITE_D100_1", "READ_D200").
 * @param plc Tham chiếu đến đối tượng MCProtocol đã kết nối.
 * @return Chuỗi kết quả của hành động (ví dụ: "Write OK", "D200 = 123").
 */
std::string SystemManager::parseAndExecutePlcCommand(const std::string& command, MCProtocol& plc) {
    std::stringstream ss(command);
    std::string command_part, device_part, value_part;
    std::getline(ss, command_part, '_');
    std::getline(ss, device_part, '_');
    std::getline(ss, value_part);

    if (command_part.empty() || device_part.empty()) return "Invalid command format";

    // Tách loại thiết bị (D, M, Y,...) và địa chỉ.
    std::string device_type = device_part.substr(0, 1);
    uint32_t address;
    try {
        address = std::stoul(device_part.substr(1));
    } catch (...) {
        return "Invalid address";
    }

    // Xử lý lệnh READ.
    if (command_part == "READ") {
        if (device_type == "D" || device_type == "C") {
            return device_part + " = " + std::to_string(plc.readSingleWord(device_type, address));
        }
        return "Unsupported READ device";
    } // Xử lý lệnh WRITE.
    else if (command_part == "WRITE") {
        try {
            uint16_t value = std::stoul(value_part);
            bool success = plc.writeSingleWord(device_type, address, value);
            return success ? "Write OK" : "Write FAILED";
        } catch (...) {
            return "Invalid value for WRITE";
        }
    }
    return "Unknown command";
}

// --- Thread Function Implementations ---

/**
 * @brief Luồng xử lý giao tiếp với PLC.
 * @details
 * - Ghim luồng vào CPU core 0.
 * - Cố gắng kết nối đến PLC, thử lại 3 lần nếu thất bại.
 * - Sau khi kết nối, gọi `initializeSystem()` để gửi lệnh khởi tạo.
 * - Thiết lập một bộ giám sát (`monitorRegister`) trên thanh ghi D102 của PLC để xử lý trạng thái lỗi.
 * - Vòng lặp chính: Lấy lệnh từ `plc_command_queue_`, thực thi chúng, và đẩy kết quả vào `plc_result_queue_`.
 * - Dọn dẹp và ngắt kết nối khi luồng dừng.
 */
void SystemManager::plc_thread_func() {
    pin_thread_to_core(0);
    LOG_REGISTER_CONTEXT("PLC", "PLC Communication Thread");
    LOG_SET_CONTEXT("PLC");
    LOG_INFO << "[PLC Thread] Starting...";

    // Vòng lặp quản lý trạng thái chính: Luôn chạy để đảm bảo PLC được kết nối.
    while (running_) {
        // ----- TRẠNG THÁI: ĐÃ KẾT NỐI -> XỬ LÝ LỆNH VÀ GIÁM SÁT -----
        if (plc_ptr_->isConnected()) {
            std::string command;
            // Lấy lệnh từ queue (không block)
            if (plc_command_queue_.pop(command)) {
                try {
                    // Thực thi lệnh. Nếu mất kết nối, hàm này sẽ throw exception.
                    LOG_INFO << "[PLC Thread] Executing command: " << command;
                    std::string response = parseAndExecutePlcCommand(command, *plc_ptr_);
                    plc_result_queue_.push(response);
                    {
                        std::lock_guard<std::mutex> lock(state_.state_mutex);
                        state_.last_plc_status = response;
                    }
                } catch (const std::exception& e) {
                    LOG_ERROR << "[PLC Thread] Connection lost while executing command '" << command << "': " << e.what();
                    
                    // --- HÀNH ĐỘNG KHI MẤT KẾT NỐI ---
                    plc_ptr_->stopAllMonitors(); // Dừng tất cả các monitor cũ để tránh thread leak
                    plc_ptr_->disconnect();      // Đóng socket cũ
                    {
                        std::lock_guard<std::mutex> lock(state_.state_mutex);
                        state_.plc_connected = false; // Cập nhật trạng thái
                    }
                    // Vòng lặp chính sẽ chuyển sang trạng thái kết nối lại ở lần lặp sau.
                }
            }
            // Ngủ một chút để tránh chiếm dụng CPU khi không có lệnh
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } 
        // ----- TRẠNG THÁI: MẤT KẾT NỐI -> THỬ KẾT NỐI LẠI -----
        else {
            LOG_INFO << "[PLC Thread] PLC disconnected. Attempting to reconnect...";
            bool connection_established = false;
            
            // Thử kết nối lại 3 lần
            for (int attempt = 1; attempt <= 3 && running_; ++attempt) {
                if (plc_ptr_->connect()) {
                    connection_established = true;
                    LOG_INFO << "[PLC Thread] PLC reconnected successfully.";
                    break;
                }
                LOG_WARNING << "[PLC Thread] Reconnection attempt " << attempt << " failed.";
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }

            // Xử lý sau khi thử kết nối lại
            if (connection_established) {
                // --- KHI KẾT NỐI LẠI THÀNH CÔNG ---
                {
                    std::lock_guard<std::mutex> lock(state_.state_mutex);
                    state_.plc_connected = true; // Cập nhật trạng thái
                }
                // Khởi tạo lại hệ thống và các monitor cho kết nối mới
                initializeSystem();
                plc_ptr_->monitorRegister("D", 102,
                    [this](const std::string&, uint32_t, uint16_t, uint16_t new_val) {
                        if (new_val == 5) {
                            LOG_WARNING << "[PLC Monitor] D102 is 5 (Error State). Acknowledging.";
                            plc_in_error_state_ = true;
                            plc_command_queue_.push("WRITE_D110_2");
                        } else if (new_val == 0 && plc_in_error_state_) {
                            LOG_INFO << "[PLC Monitor] D102 is 0 after error. Resetting.";
                            plc_in_error_state_ = false;
                            plc_command_queue_.push("WRITE_D110_1");
                        }
                    }, 250);
            } else {
                // Nếu sau 3 lần vẫn thất bại, đợi một lúc rồi thử lại từ đầu
                LOG_ERROR << "[PLC Thread] Could not reconnect after 3 attempts. Waiting for 5 seconds.";
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
        }
    }

    // Dọn dẹp khi luồng kết thúc.
    plc_ptr_->stopAllMonitors();
    plc_ptr_->disconnect();
    LOG_INFO << "[PLC Thread] Stopped.";
}

/**
 * @brief Luồng xử lý dữ liệu từ cảm biến LiDAR.
 * @details
 * - Ghim luồng vào CPU core 1.
 * - Khởi tạo và bắt đầu `LidarProcessor`.
 * - Thiết lập `setRealtimeCallback`: được gọi cho mỗi frame dữ liệu LiDAR.
 *   - Tính toán khoảng cách an toàn phía trước.
 *   - Cập nhật `is_safe_to_move` và `current_front_distance` trong `state_`.
 *   - Tính toán tốc độ mượt mà và gửi lệnh `WRITE_D103_...` đến PLC.
 * - Thiết lập `setStablePointsCallback`: được gọi khi có một tập hợp điểm ổn định.
 *   - Đẩy các điểm này vào `stable_points_queue_` để luồng server gửi đi.
 * - Dừng `LidarProcessor` khi luồng kết thúc.
 */
void SystemManager::lidar_thread_func() {
    pin_thread_to_core(1);
    LOG_INFO << "[LiDAR Thread] Starting...";

    // Vòng lặp chính để quản lý kết nối và xử lý dữ liệu LiDAR.
    // Nếu kết nối bị mất, thử khởi động lại.
    while (running_) {
        auto lidar_processor = std::make_unique<LidarProcessor>();
        // Thử khởi tạo với retry
        int init_attempts = 0;
        bool initialized = false;

        // Vòng lặp khởi tạo với tối đa 3 lần thử.
        // Nếu thất bại, đợi 10 giây rồi thử lại từ đầu.
        // Cập nhật trạng thái kết nối LiDAR trong state_.
        while (init_attempts < 3 && running_) {
            if (lidar_processor->initialize() && lidar_processor->start()) {
                initialized = true;
                break;
            }

            LOG_WARNING << "[LiDAR Thread] Initialization attempt "
                        << (init_attempts + 1) << " failed. Retrying in 3s...";
            std::this_thread::sleep_for(std::chrono::seconds(3));
            init_attempts++;
        }

        // Nếu sau 3 lần thử vẫn không thành công, đợi 10 giây rồi thử lại từ đầu.
        if (!initialized) {
            LOG_ERROR << "[LiDAR Thread] Failed to initialize after 3 attempts";
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            state_.lidar_connected = false;

            // Đợi 10 giây rồi thử lại
            std::this_thread::sleep_for(std::chrono::seconds(10));
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            state_.lidar_connected = true;
        }

        // Callback xử lý dữ liệu thời gian thực để phát hiện vật cản và điều khiển tốc độ.
        lidar_processor->setRealtimeCallback([this](const std::vector<LidarPoint>& points) {
            LOG_INFO << "[LIDAR Thread] Processed frame. Points: " << points.size();
            {
                // Đóng gói dữ liệu điểm để gửi đi
                std::vector<ServerComm::Point2D> points_to_send;
                points_to_send.reserve(points.size());
                for (const auto& p : points) {
                    points_to_send.push_back({p.x, p.y});
                }
                // Đẩy vào hàng đợi thời gian thực
                realtime_points_queue_.push(std::move(points_to_send));
            }
            float min_front = 999.0f;
            // Tìm khoảng cách nhỏ nhất trong vùng 90 độ phía trước.
            for (const auto& p : points) {
                // Góc được tính bằng radian, 2.356 rad ≈ 135°, 3.927 rad ≈ 225°.
                if (p.angle > 2.356 && p.angle < 3.927) {
                    min_front = std::min(min_front, p.distance);
                }
            }

            // Nếu không có điểm nào ở phía trước, bỏ qua.
            if (min_front >= 999.0f) return;

            float min_dist_cm = min_front * 100.0f; // Chuyển từ mét sang cm.
            int smooth_speed = 0; // Tốc độ mượt mà để gửi đến PLC.
            bool movement_active = false; // Cờ lệnh di chuyển hiện tại.

            // Khóa mutex để cập nhật và đọc state_ một cách an toàn.
            {
                std::lock_guard<std::mutex> lock(state_.state_mutex); // Khóa state_mutex để truy cập state_.
                state_.current_front_distance = min_dist_cm; // Cập nhật khoảng cách phía trước.
                state_.is_safe_to_move = min_dist_cm > EMERGENCY_STOP_DISTANCE_CM; // Cập nhật trạng thái an toàn.
                state_.last_safety_update = std::chrono::steady_clock::now().time_since_epoch().count(); // Cập nhật thời gian.
                movement_active = state_.movement_command_active; // Lấy trạng thái lệnh di chuyển hiện tại.

                // Cập nhật chuỗi trạng thái LiDAR để ghi log
                std::stringstream ss;
                ss << "Points: " << points.size()
                   << ", FrontDist: " << std::fixed << std::setprecision(1)
                   << min_dist_cm << "cm, Safe: " << (state_.is_safe_to_move ? "Yes" : "No");
                state_.last_lidar_data = ss.str();
                LOG_INFO << "[LIDAR Thread][REALTIMECALLBACK] state_.last_lidar_data: " << state_.last_lidar_data;
                // Tính toán tốc độ mượt mà dựa trên trạng thái an toàn và lệnh di chuyển.
                if (state_.is_safe_to_move && movement_active) { // Chỉ tính tốc độ nếu an toàn và có lệnh di chuyển.
                    smooth_speed = calculateSmoothSpeed(min_dist_cm, true); // movement_active = true
                } else {
                    smooth_speed = calculateSmoothSpeed(min_dist_cm, false); // movement_active = false
                }
            }

            // Chỉ gửi lệnh tốc độ đến PLC nếu giá trị tốc độ thay đổi để giảm tải.
            static int last_sent_speed = -1;
            if (smooth_speed != last_sent_speed) { // Gửi lệnh mới nếu tốc độ thay đổi.
                LOG_INFO << "[LIDAR Thread] Sending speed command to PLC: " << smooth_speed;
                // Đẩy lệnh điều khiển tốc độ vào hàng đợi PLC.
                plc_command_queue_.push("WRITE_D103_" + std::to_string(smooth_speed));
                last_sent_speed = smooth_speed;
            }
        });

        // // Callback xử lý dữ liệu ổn định để gửi lên server.
        // lidar_processor->setStablePointsCallback([this](const std::vector<LidarPoint>& stable_points) {
        //     if (stable_points.empty()) {
        //         LOG_WARNING << "[LiDAR Thread][STABLEPOINTSCALLBACK] No stable points received.";
        //         return;
        //     }
        //     std::vector<Point2D> stable_2d;
        //     stable_2d.reserve(stable_points.size());
        //     LOG_INFO << "[LiDAR Thread][STABLEPOINTSCALLBACK] Stable points received. Size: " << stable_points.size();
        //     for (const auto& p : stable_points) {
        //         stable_2d.push_back({p.x, p.y});
        //     }
        //     // Đẩy vào hàng đợi không khóa để luồng server xử lý.
        //     stable_points_queue_.push(std::move(stable_2d));
        //     LOG_INFO << "[LiDAR Thread] Stable points pushed to queue. Size: " << stable_2d.size();
        // });

        // Monitor loop - kiểm tra health định kỳ
        while (running_ && lidar_processor->isConnectionHealthy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // Update connection status
            {
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                state_.lidar_connected = lidar_processor->isConnectionHealthy();
            }
        }

        // Nếu tới đây nghĩa là mất kết nối
        LOG_WARNING << "[LiDAR Thread] LiDAR connection lost. Will restart processor...";
        lidar_processor->stop();

        {
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            state_.lidar_connected = false;
            state_.is_safe_to_move = false;  // Set unsafe khi mất LiDAR
        }

        // Gửi lệnh dừng khẩn cấp nếu đang di chuyển
        if (state_.is_moving) {
            plc_command_queue_.push("WRITE_D100_0");  // Stop
            plc_command_queue_.push("WRITE_D103_0");  // Speed = 0
            LOG_WARNING << "[LiDAR Thread] Emergency stop due to LiDAR disconnection";
        }

        // Đợi trước khi thử khởi động lại
        LOG_INFO << "[LiDAR Thread] Entering main loop.";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));// Đợi 1 giây trước khi thử lại
    }
    // Dừng và dọn dẹp khi luồng kết thúc.
    LOG_INFO << "[LiDAR Thread] Stopped.";
}

/**
 * @brief Luồng xử lý các lệnh điều khiển nhận được từ server.
 * @details
 * - Ghim luồng vào CPU core 3.
 * - Vòng lặp chính: Lấy lệnh điều hướng từ hàng đợi của `comm_server_`.
 * - Kiểm tra trạng thái an toàn và lỗi PLC trước khi thực thi.
 * - Chuyển đổi các lệnh từ server (ví dụ: `MOVE_TO_POINT`) thành các lệnh cụ thể cho PLC (ví dụ: `WRITE_D100_1`).
 * - Đẩy các lệnh PLC vào `plc_command_queue_`.
 */
void SystemManager::command_handler_thread() {
    pin_thread_to_core(3);
    LOG_INFO << "[Command Handler] Thread started. Waiting for commands from server...";

    while (running_) {
        ServerComm::NavigationCommand cmd;
        // Lấy lệnh từ server (getNextCommand có thể bị block).
        if (comm_server_->getNextCommand(cmd)) {
            LOG_INFO << "[Command Handler] Processing command type: " << static_cast<int>(cmd.type);

            // KIỂM TRA LỆNH LẶP LẠI: Nếu lệnh mới giống lệnh cũ, bỏ qua.
            if (cmd.type == last_processed_cmd_type_) {
                LOG_INFO << "[Command Handler] Ignoring duplicate command: " << static_cast<int>(cmd.type);
                continue;
            }
            last_processed_cmd_type_ = cmd.type; // Cập nhật lệnh cuối cùng đã xử lý

            bool is_safe;
            // Lấy trạng thái an toàn hiện tại.
            {
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                is_safe = state_.is_safe_to_move;
            }

            LOG_ERROR << "[Command Handler] Command ignored! PLC is in error state: " << plc_in_error_state_ 
                      << ", PLC connected: " << (plc_ptr_ ? (plc_ptr_->isConnected() ? "Yes" : "No") : "No");
            if (plc_in_error_state_ || plc_ptr_ == nullptr || !plc_ptr_->isConnected()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            switch (cmd.type) {
                // Các lệnh di chuyển tiến.
                case ServerComm::NavigationCommand::MOVE_TO_POINT:
                case ServerComm::NavigationCommand::FOLLOW_PATH: {
                    // Tạo lệnh di chuyển.
                    std::string move_cmd =  cmd.type == ServerComm::NavigationCommand::MOVE_TO_POINT ? "WRITE_D100_1" : "WRITE_D100_2";
                    LOG_INFO << "[Command Handler] Processing movement command: " << move_cmd;
                    // Lưu lại lệnh di chuyển cuối cùng để có thể tự động tiếp tục.
                    {
                        std::lock_guard<std::mutex> lock(last_command_mutex_);
                        last_movement_command_ = move_cmd;
                    }
                    // Chỉ thực hiện nếu an toàn.
                    if (is_safe) {
                        LOG_INFO << "Executing movement command from server.";
                        std::lock_guard<std::mutex> lock(state_.state_mutex);
                        state_.movement_command_active = true;
                        state_.is_moving = false;
                        plc_command_queue_.push("WRITE_D101_0");
                        plc_command_queue_.push(move_cmd);
                    } else {
                        LOG_WARNING << "Cannot execute movement - obstacle detected!";
                    }
                    break;
                }
                // Lệnh xoay.
                case ServerComm::NavigationCommand::ROTATE_TO_ANGLE: {
                    LOG_INFO << "[Command Handler] Processing rotation command.";
                    // Chỉ thực hiện nếu an toàn.
                    if (is_safe) {
                        LOG_INFO << "Executing rotation command from server.";
                        std::lock_guard<std::mutex> lock(state_.state_mutex);
                        state_.movement_command_active = true;
                        state_.is_moving = false;
                        plc_command_queue_.push("WRITE_D100_1");
                        plc_command_queue_.push("WRITE_D101_1"); // Giả sử xoay trái.
                    } else {
                        LOG_WARNING << "Cannot execute rotation - obstacle detected!";
                    }
                    break;
                }
                // Lệnh dừng.
                case ServerComm::NavigationCommand::STOP:
                case ServerComm::NavigationCommand::EMERGENCY_STOP: {
                    LOG_INFO << "[Command Handler] Executing STOP command from server.";
                    plc_command_queue_.push("WRITE_D100_0");
                    plc_command_queue_.push("WRITE_D101_0");
                    // Xóa lệnh di chuyển cuối cùng để ngăn việc tự động tiếp tục.
                    {
                        std::lock_guard<std::mutex> lock(last_command_mutex_);
                        last_movement_command_.clear();
                    }
                    {
                        std::lock_guard<std::mutex> lock(state_.state_mutex);
                        state_.movement_command_active = false;
                        state_.is_moving = false;
                    }
                    break;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    LOG_INFO << "[Command Handler] Thread stopped.";
}

/**
 * @brief Luồng giám sát an toàn.
 * @details
 * - Ghim luồng vào CPU core 0.
 * - Vòng lặp chính: Liên tục kiểm tra cờ `is_safe_to_move` từ `state_`.
 * - **Logic chính**:
 *   - Nếu trạng thái chuyển từ an toàn (`was_safe=true`) sang không an toàn (`is_safe=false`), gửi lệnh dừng khẩn cấp đến PLC.
 *   - Nếu trạng thái chuyển từ không an toàn sang an toàn, kiểm tra `last_movement_command_` và gửi lại lệnh đó để AGV tự động tiếp tục di chuyển.
 */
void SystemManager::safety_monitor_thread() {
    pin_thread_to_core(0);
    LOG_INFO << "[Safety Monitor] Started.";
    bool was_safe = true;

    while (running_) {
        bool is_safe;
        // Lấy trạng thái an toàn hiện tại.
        {
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            is_safe = state_.is_safe_to_move;
        }

        // Chuyển từ an toàn -> không an toàn: Dừng khẩn cấp.
        if (was_safe && !is_safe) {
            LOG_WARNING << "[Safety Monitor] DANGER DETECTED! Sending stop command.";
            plc_command_queue_.push("WRITE_D100_0");
            plc_command_queue_.push("WRITE_D101_0");
        } // Chuyển từ không an toàn -> an toàn: Tự động tiếp tục.
        else if (!was_safe && is_safe) {
            LOG_INFO << "[Safety Monitor] Path clear. Checking for command to resume...";
            std::lock_guard<std::mutex> lock(last_command_mutex_);
            // Chỉ tiếp tục nếu có lệnh di chuyển cuối cùng và PLC không bị lỗi.
            if (!last_movement_command_.empty() && !plc_in_error_state_) {
                LOG_INFO << "[Safety Monitor] Resuming last command: " << last_movement_command_;
                plc_command_queue_.push(last_movement_command_);
            }
        }

        was_safe = is_safe;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    LOG_INFO << "[Safety Monitor] Stopped.";
}

/**
 * @brief Luồng gửi các tác vụ định kỳ đến PLC.
 * @details
 * - Ghim luồng vào CPU core 1.
 * - Gửi lệnh `WRITE_D200_1` đến PLC mỗi 500ms. Lệnh này có thể được dùng như một tín hiệu "heartbeat" để PLC biết rằng phần mềm vẫn đang hoạt động.
 */
void SystemManager::plc_periodic_tasks_thread() {
    pin_thread_to_core(1);
    LOG_INFO << "[PLC Periodic] Thread started.";
    std::this_thread::sleep_for(std::chrono::seconds(5));

    while (running_) {
        plc_command_queue_.push("WRITE_D200_1");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    LOG_INFO << "[PLC Periodic] Thread stopped.";
}

/**
 * @brief Luồng giám sát và đọc dữ liệu từ pin.
 * @details
 * - Ghim luồng vào CPU core 2.
 * - Sử dụng lớp `JBDBMSSingleton` để đảm bảo chỉ có một kết nối đến BMS.
 * - Khởi tạo kết nối đến BMS qua cổng serial.
 * - Vòng lặp chính: Cứ sau 20 giây, gọi `bms.updateBatteryData()` để đọc dữ liệu mới và cập nhật `battery_level` và `battery_connected` trong `state_`.
 */
void SystemManager::battery_thread_func() {
    pin_thread_to_core(2);
    LOG_INFO << "[Battery Thread] Starting...";

    JBDBMSSingleton& bms = JBDBMSSingleton::getInstance(BATTERY_BMS_SERIAL_PORT);
    bool is_bms_initialized = false; // Cờ theo dõi trạng thái kết nối cục bộ

    // Vòng lặp quản lý trạng thái chính
    while (running_) {
        if (is_bms_initialized) {
            // ----- TRẠNG THÁI: ĐÃ KẾT NỐI -> CẬP NHẬT DỮ LIỆU -----
            if (bms.updateBatteryData()) { // Cập nhật thành công
                BatteryData data = bms.getBatteryData();
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                state_.battery_connected = true;
                state_.battery_level = data.stateOfCharge;
            } else {
                // Nếu update thất bại, coi như mất kết nối và thử kết nối lại
                LOG_WARNING << "[Battery Thread] Failed to update BMS data. Connection likely lost.";
                is_bms_initialized = false; // Kích hoạt logic kết nối lại ở lần lặp sau
                {
                    std::lock_guard<std::mutex> lock(state_.state_mutex);
                    state_.battery_connected = false;
                }
            }
            // Đợi 20 giây cho lần cập nhật tiếp theo
            std::this_thread::sleep_for(std::chrono::seconds(20));

        } else {
            // ----- TRẠNG THÁI: MẤT KẾT NỐI -> THỬ KẾT NỐI LẠI -----
            LOG_INFO << "[Battery Thread] Attempting to initialize BMS connection...";
            if (bms.initialize()) { // Kết nối thành công
                LOG_INFO << "[Battery Thread] BMS connected successfully.";
                is_bms_initialized = true; // Đánh dấu đã kết nối thành công
            } else {
                LOG_ERROR << "[Battery Thread] Failed to initialize BMS. Retrying in 5 seconds...";
                {
                    std::lock_guard<std::mutex> lock(state_.state_mutex);
                    state_.battery_connected = false;
                }
                std::this_thread::sleep_for(std::chrono::seconds(5)); // Đợi 5 giây trước khi thử lại
            }
        }
    }
    LOG_INFO << "[Battery Thread] Stopped.";
}

/**
 * @brief Lấy mẫu các điểm quan trọng từ tập hợp điểm LiDAR.
 * @details Hàm này được sử dụng để giảm số lượng điểm LiDAR gửi lên server,
 * chỉ giữ lại những điểm quan trọng để giảm tải băng thông mà vẫn đảm bảo
 * đủ thông tin cho việc hiển thị hoặc xử lý.
 * @param points Vector chứa tất cả các điểm LiDAR.
 * @return Vector chứa các điểm LiDAR đã được lấy mẫu.
 */
std::vector<ServerComm::Point2D> SystemManager::sampleImportantPoints(const std::vector<ServerComm::Point2D>& points) {
    if (points.empty()) {
        return {};
    }

   std::vector<ServerComm::Point2D> result;
    // Dành trước bộ nhớ để tăng hiệu năng, tránh cấp phát lại nhiều lần
    result.reserve(points.size() / 2); // Ước tính kích thước sau khi sampling

    // Duyệt qua tất cả các điểm đầu vào
    for (const auto& p : points) {
        // Tính khoảng cách từ điểm đến tâm (vị trí của LiDAR)
        float dist = sqrt(p.x * p.x + p.y * p.y);

        if (dist < 2.0f) {
            // Giữ lại 100% các điểm ở khoảng cách dưới 2 mét
            result.push_back(p);
        } else if (dist < 5.0f && (rand() % 2 == 0)) {
            // Giữ lại ngẫu nhiên 50% các điểm ở khoảng cách từ 2 đến 5 mét
            result.push_back(p);
        } else if (rand() % 4 == 0) {
            // Giữ lại ngẫu nhiên 25% các điểm ở khoảng cách xa hơn 5 mét
            result.push_back(p);
        }
    }

    return result;
}
// Lưu ý: Luồng server_communication_thread không còn được quản lý trực tiếp ở đây.
// Nó được quản lý bên trong lớp ServerComm::CommunicationServer,
// vốn sử dụng các luồng riêng (`std::thread`) để xử lý I/O mạng một cách bất đồng bộ.
// Việc khởi tạo và bắt đầu nó được thực hiện trong hàm `initialize()`.
