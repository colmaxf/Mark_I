#include "SystemManager.h"
#include <sched.h>
#include <cstring>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <sstream>

#if TEST_KEYBOARD_MODE == 1
#include <linux/input.h> // Cho input_event
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <cstring>
#endif

/**
 * @brief Hàm khởi tạo của lớp SystemManager.
 * @details Khởi tạo đối tượng giao tiếp với server `CommunicationServer` với địa chỉ IP và cổng được cung cấp.
 * @param server_ip Địa chỉ IP của server trung tâm.
 * @param server_port Cổng của server trung tâm.
 * @param last_processed_cmd_type_ Khởi tạo loại lệnh cuối cùng đã xử lý là STOP để tránh việc bỏ qua lệnh STOP ban đầu.
 */
SystemManager::SystemManager(const std::string &server_ip, int server_port) : comm_server_(std::make_unique<ServerComm::CommunicationServer>(server_ip, server_port)),
                                                                              last_processed_cmd_type_(ServerComm::NavigationCommand::STOP), // Khởi tạo lệnh cuối cùng là STOP
                                                                              arc_direction_(ArcDirection::NONE)
{
}

/**
 * @brief Hàm hủy của lớp SystemManager.
 * @details Đảm bảo rằng hàm `stop()` được gọi để dừng tất cả các luồng và giải phóng tài nguyên một cách an toàn.
 */
SystemManager::~SystemManager()
{
    stop();
}

/**
 * @brief Ghim luồng hiện tại vào một lõi CPU cụ thể.
 * @details Việc này giúp giảm thiểu việc chuyển đổi ngữ cảnh (context switching) và có thể cải thiện hiệu năng
 * cho các tác vụ thời gian thực hoặc yêu cầu xử lý chuyên sâu.
 * @param core_id ID của lõi CPU để ghim luồng vào (ví dụ: 0, 1, 2, 3 trên Raspberry Pi 4).
 */
void SystemManager::pin_thread_to_core(int core_id)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id % 4, &cpuset); // Pi 4 has 4 cores
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0)
    {
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
bool SystemManager::initialize()
{
    LOG_INFO << "[SystemManager] Initializing...";

    // Initialize PLC pointer
    plc_ptr_ = std::make_shared<MCProtocol>(PLC_IP, PLC_PORT);

    // Thiết lập callback TRƯỚC KHI kết nối.
    // Điều này đảm bảo trạng thái `state_.server_connected` luôn được cập nhật chính xác,
    // ngay cả khi kết nối lại tự động.
    comm_server_->setConnectionCallback([this](bool connected)
                                        {
        LOG_INFO << "[SystemManager] Server connection status changed to: " << (connected ? "Connected" : "Disconnected");
        std::lock_guard<std::mutex> lock(state_.state_mutex);
        state_.server_connected = connected; });
    // Vòng lặp kết nối đến server, thử lại sau mỗi 3 giây nếu thất bại.
    // Điều này đảm bảo hệ thống có thể khởi động ngay cả khi server chưa sẵn sàng.
    while (running_ && !comm_server_->connect())
    {
        LOG_WARNING << "[SystemManager] Server connection failed, retrying in 3 seconds...";
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    if (!running_)
        return false;

    // Setup callbacks for the server communicator
    // Callback này được gọi bởi luồng của server để lấy dữ liệu trạng thái mới nhất của AGV.
    comm_server_->setStatusCallback([this]() -> ServerComm::AGVStatusPacket
                                    {
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

            // status.imu_connected = state_.imu_connected;
            // status.current_heading = state_.current_heading;
            // status.current_roll = state_.current_roll;
            // status.current_pitch = state_.current_pitch;
        }

        {
            // Khóa plc_ptr_mutex_ để truy cập con trỏ PLC một cách an toàn.
            std::lock_guard<std::mutex> lock(plc_ptr_mutex_);
            if (plc_ptr_ && plc_ptr_->isConnected()) {
                try {
                    // Đọc nhiều thanh ghi PLC cùng lúc để tăng hiệu quả giao tiếp.
                    // Đọc một khối từ D100 đến D200 (101 thanh ghi).
                    // auto regs = plc_ptr_->readWords("D", 100, 101); // Read D100 to D200
                    //// // Kiểm tra xem có đọc đủ số lượng thanh ghi không trước khi truy cập.
                    // if (regs.size() >= 101) {
                    //     status.plc_registers["D100"] = regs[0];
                    //     status.plc_registers["D101"] = regs[1];
                    //     status.plc_registers["D102"] = regs[2];
                    //     status.plc_registers["D103"] = regs[3];
                    //     status.plc_registers["D110"] = regs[10];
                    //     status.plc_registers["D200"] = regs[100];
                    // }
                    //--------Đọc từng thanh ghi riêng lẻ (không hiệu quả)--------
                    status.plc_registers["D100"] = plc_ptr_->readSingleWord("D", 100);
                    status.plc_registers["D101"] = plc_ptr_->readSingleWord("D", 101);
                    status.plc_registers["D102"] = plc_ptr_->readSingleWord("D", 102);
                    status.plc_registers["D103"] = plc_ptr_->readSingleWord("D", 103);
                    status.plc_registers["D110"] = plc_ptr_->readSingleWord("D", 110);
                    status.plc_registers["D200"] = plc_ptr_->readSingleWord("D", 200);
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
            LOG_INFO << "[SystemManager] [khaipv] realtime_points: " << realtime_points.size();
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
                LOG_INFO << "[SystemManager] [khaipv] Sending " << status.realtime_lidar_points.size() 
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

        return status; });

    comm_server_->setConnectionCallback([this](bool connected)
                                        {
        // Callback được gọi khi trạng thái kết nối server thay đổi.
        LOG_INFO << "[SystemManager] Server connection status: " << (connected ? "Connected" : "Disconnected");
        std::lock_guard<std::mutex> lock(state_.state_mutex);
        state_.server_connected = connected; });

    if (!comm_server_->start())
    {
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
void SystemManager::run()
{
    if (!initialize())
    {
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
    threads_.emplace_back(&SystemManager::imu_thread_func, this);
    // threads_.emplace_back(&SystemManager::heading_correction_thread, this); // Đã tích hợp vào lidar_thread
#if TEST_KEYBOARD_MODE == 1
    threads_.emplace_back(&SystemManager::keyboard_control_thread, this);

    LOG_INFO << "[SystemManager] ⌨️  USB Keyboard control enabled (W/A/S/D/B)";
#endif
    // Luồng server_communication_thread được quản lý bên trong lớp CommunicationServer.

    // Vòng lặp chính chỉ để giám sát và báo cáo trạng thái.
    while (running_)
    {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        {
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            LOG_INFO << "--- SYSTEM STATUS ---";
            LOG_INFO << "PLC Connected: " << (state_.plc_connected ? "Yes" : "No")
                     << " | LiDAR Connected: " << (state_.lidar_connected ? "Yes" : "No")
                     << " | Server Connected: " << (state_.server_connected ? "Yes" : "No")
                     << " | Last LiDAR Data: " << state_.last_lidar_data
                     << " | Last IMU Data: " << state_.last_imu_data;
            LOG_INFO << "Heading: " << std::fixed << std::setprecision(1)
                     << state_.current_heading << "° | Safe: "
                     << (state_.is_safe_to_move ? "Yes" : "No")
                     << " | Moving: " << (state_.is_moving ? "Yes" : "No");
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
void SystemManager::stop()
{
    // Sử dụng exchange để đảm bảo hàm chỉ chạy một lần.
    if (!running_.exchange(false))
    {
        return;
    }
    LOG_INFO << "[SystemManager] Initiating shutdown...";
    if (comm_server_)
    {
        comm_server_->stop(); // Gửi tín hiệu dừng đến luồng của server.
    }
    for (auto &thread : threads_)
    {
        if (thread.joinable())
        {
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
void SystemManager::initializeSystem()
{
    if (system_initialized_.exchange(true))
    {
        LOG_INFO << "[System Init] Already initialized, skipping D110_1";
        return;
    }

    LOG_INFO << "[System Init] Sending D110_1";
    plc_command_queue_.push("WRITE_D110_1");
    // Chờ phản hồi từ PLC (pop có thể bị block).
    std::string result;
    if (plc_result_queue_.pop(result))
    {
        LOG_INFO << "[System Init] D110_1 response: " << result;
    }
    else
    {
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
double SystemManager::calculateSpeed(double distance)
{
    distance = std::max(50.0, std::min(400.0, distance));
    if (distance >= 50.0 && distance < 100.0)
    {
        return 100.0 + ((distance - 50.0) * (500.0 - 100.0)) / (100.0 - 50.0);
    }
    else if (distance >= 100.0 && distance < 200.0)
    {
        return 500.0 + ((distance - 100.0) * (1500.0 - 500.0)) / (200.0 - 100.0);
    }
    else if (distance >= 200.0 && distance < 300.0)
    {
        return 1500.0 + ((distance - 200.0) * (2500.0 - 1500.0)) / (300.0 - 200.0);
    }
    else
    { // distance >= 300.0
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
int SystemManager::calculateSmoothSpeed(float distance_cm, bool movement_active)
{
    // // Yêu cầu mới: Nếu đang lùi, luôn dùng tốc độ lùi cố định.
    // // Chúng ta sẽ dùng `calculateSmoothSpeed` để khởi động mượt mà đến tốc độ này.
    // bool is_reversing;
    // {
    //     std::lock_guard<std::mutex> lock(movement_target_mutex_);
    //     is_reversing = current_movement_.is_active && !current_movement_.is_forward;
    // }
    int target_speed = /*is_reversing ? 200 :*/ static_cast<int>(calculateSpeed(distance_cm));

    // Nếu không có lệnh di chuyển và không có lệnh đang chờ, dừng AGV.
    if (!movement_active && !state_.movement_pending)
    {
        state_.is_moving = false;
        state_.current_speed = 0;
        return 0;
    }

    // Nếu AGV đang đứng yên và nhận lệnh di chuyển mới (hoặc có lệnh đang chờ)

    if ((!state_.is_moving && target_speed > 0) || state_.movement_pending)
    { // Bắt đầu di chuyển
        LOG_INFO << "[Speed Control] Starting movement towards target speed: " << target_speed;
        state_.is_moving = true;

        if (state_.movement_pending)
        { // Có lệnh di chuyển đang chờ
            LOG_INFO << "[Speed Control] Acknowledged pending movement command.";
            state_.movement_pending = false;
        }

        state_.movement_start_time = std::chrono::steady_clock::now();
        state_.current_speed = MIN_START_SPEED;
    }

    if (state_.current_speed == 0 && state_.is_moving) 
    {// Trường hợp đặc biệt: từ đứng yên sang di chuyển
        state_.current_speed = MIN_START_SPEED;
        return state_.current_speed;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - state_.movement_start_time)
                       .count();

    if (elapsed < ACCEL_TIME_MS) 
    {// Gia tốc
        float t = static_cast<float>(elapsed) / ACCEL_TIME_MS;
        state_.current_speed = MIN_START_SPEED + static_cast<int>((target_speed - MIN_START_SPEED) * t);
    } else
    {// Giảm tốc
        state_.current_speed = target_speed;
    }

    if (target_speed < MIN_START_SPEED && !state_.movement_pending)
    {// Nếu tốc độ mục tiêu quá thấp và không có lệnh chờ, dừng AGV.
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
std::string SystemManager::parseAndExecutePlcCommand(const std::string &command, MCProtocol &plc)
{
    std::stringstream ss(command);
    std::string command_part, device_part, value_part;
    std::getline(ss, command_part, '_');
    std::getline(ss, device_part, '_');
    std::getline(ss, value_part);

    if (command_part.empty() || device_part.empty())
        return "Invalid command format";

    // Tách loại thiết bị (D, M, Y,...) và địa chỉ.
    std::string device_type = device_part.substr(0, 1);
    uint32_t address;
    try
    {
        address = std::stoul(device_part.substr(1));
    }
    catch (...)
    {
        return "Invalid address";
    }

    // Xử lý lệnh READ.
    if (command_part == "READ")
    {
        if (device_type == "D" || device_type == "C")
        {
            return device_part + " = " + std::to_string(plc.readSingleWord(device_type, address));
        }
        return "Unsupported READ device";
    } // Xử lý lệnh WRITE.
    else if (command_part == "WRITE")
    {
        try
        {
            uint16_t value = std::stoul(value_part);
            bool success = plc.writeSingleWord(device_type, address, value);
            return success ? "Write OK" : "Write FAILED";
        }
        catch (...)
        {
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
void SystemManager::plc_thread_func()
{
    pin_thread_to_core(0);
    LOG_REGISTER_CONTEXT("PLC", "PLC Communication Thread");
    LOG_SET_CONTEXT("PLC");
    LOG_INFO << "[PLC Thread] Starting...";

    // Vòng lặp quản lý trạng thái chính: Luôn chạy để đảm bảo PLC được kết nối.
    while (running_)
    {
        // ----- TRẠNG THÁI: ĐÃ KẾT NỐI -> XỬ LÝ LỆNH VÀ GIÁM SÁT -----
        if (plc_ptr_->isConnected())
        {
            std::string command;
            // Lấy lệnh từ queue (không block)
            if (plc_command_queue_.pop(command))
            {
                try
                {
                    // Thực thi lệnh. Nếu mất kết nối, hàm này sẽ throw exception.
                    LOG_INFO << "[PLC Thread] Executing command: " << command;
                    std::string response = parseAndExecutePlcCommand(command, *plc_ptr_);
                    plc_result_queue_.push(response);
                    {
                        std::lock_guard<std::mutex> lock(state_.state_mutex);
                        state_.last_plc_status = response;
                    }
                }
                catch (const std::exception &e)
                {
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
        else
        {
            LOG_INFO << "[PLC Thread] PLC disconnected. Attempting to reconnect...";
            bool connection_established = false;

            // Thử kết nối lại 3 lần
            for (int attempt = 1; attempt <= 3 && running_; ++attempt)
            {
                if (plc_ptr_->connect())
                {
                    connection_established = true;
                    LOG_INFO << "[PLC Thread] PLC reconnected successfully.";
                    break;
                }
                LOG_WARNING << "[PLC Thread] Reconnection attempt " << attempt << " failed.";
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }

            // Xử lý sau khi thử kết nối lại
            if (connection_established)
            {
                // --- KHI KẾT NỐI LẠI THÀNH CÔNG ---
                {
                    std::lock_guard<std::mutex> lock(state_.state_mutex);
                    state_.plc_connected = true; // Cập nhật trạng thái
                }
                // Khởi tạo lại hệ thống và các monitor cho kết nối mới
                initializeSystem();
                plc_ptr_->monitorRegister("D", 102, [this](const std::string &, uint32_t, uint16_t, uint16_t new_val)
                                          {
                        if (new_val == 5) {
                            LOG_WARNING << "[PLC Monitor] D102 is 5 (Error State). Acknowledging.";
                            plc_in_error_state_ = true;
                            plc_command_queue_.push("WRITE_D110_2");
                        } else if (new_val == 0 && plc_in_error_state_) {
                            LOG_INFO << "[PLC Monitor] D102 is 0 after error. Resetting.";
                            plc_in_error_state_ = false;
                            plc_command_queue_.push("WRITE_D110_1");
                        } }, 250);
            }
            else
            {
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
 *   - Tính toán tốc độ mượt mà và lưu vào state_.current_speed
 *   - GHI D103 = 1 (an toàn) hoặc 0 (không an toàn)
 *   - Heading correction sẽ xử lý D104/D105 ở thread riêng
 * - Thiết lập `setStablePointsCallback`: được gọi khi có một tập hợp điểm ổn định.
 *   - Đẩy các điểm này vào `stable_points_queue_` để luồng server gửi đi.
 * - Dừng `LidarProcessor` khi luồng kết thúc.
 */
void SystemManager::lidar_thread_func()
{
    pin_thread_to_core(1);
    LOG_INFO << "[LiDAR Thread] Starting...";

    // Vòng lặp chính để quản lý kết nối và xử lý dữ liệu LiDAR.
    // Nếu kết nối bị mất, thử khởi động lại.
    while (running_)
    {
        auto lidar_processor = std::make_unique<LidarProcessor>();
        // Thử khởi tạo với retry
        int init_attempts = 0;
        bool initialized = false;

        // Vòng lặp khởi tạo với tối đa 3 lần thử.
        // Nếu thất bại, đợi 10 giây rồi thử lại từ đầu.
        // Cập nhật trạng thái kết nối LiDAR trong state_.
        while (init_attempts < 3 && running_)
        {
            if (lidar_processor->initialize() && lidar_processor->start())
            {
                initialized = true;
                break;
            }

            LOG_WARNING << "[LiDAR Thread] Initialization attempt "
                        << (init_attempts + 1) << " failed. Retrying in 3s...";
            std::this_thread::sleep_for(std::chrono::seconds(3));
            init_attempts++;
        }

        // Nếu sau 3 lần thử vẫn không thành công, đợi 10 giây rồi thử lại từ đầu.
        if (!initialized)
        {
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
        lidar_processor->setRealtimeCallback([this](const std::vector<LidarPoint> &points)
                                             {
                                                 LOG_INFO << "[LIDAR Thread][khaipv] Processed frame. Points: " << points.size();
                                                 {
                                                     // Đóng gói dữ liệu điểm để gửi đi
                                                     std::vector<ServerComm::Point2D> points_to_send;
                                                     points_to_send.reserve(points.size());
                                                     for (const auto &p : points)
                                                     {
                                                         points_to_send.push_back({p.x, p.y});
                                                     }
                                                     // Đẩy vào hàng đợi thời gian thực
                                                     realtime_points_queue_.push(std::move(points_to_send));
                                                     LOG_INFO << "[LIDAR Thread] [khaipv] Realtime points pushed to queue. Size: " << points_to_send.size();
                                                 }
                                                 float min_front = 999.0f;
                                                 // Tìm khoảng cách nhỏ nhất trong vùng 90 độ phía trước.
                                                 for (const auto &p : points)
                                                 {
                                                     // Góc được tính bằng radian, 2.356 rad ≈ 135°, 3.927 rad ≈ 225°.
                                                     if (p.angle > 2.356 && p.angle < 3.927)
                                                     {
                                                         min_front = std::min(min_front, p.distance);
                                                     }
                                                 }

                                                 // Nếu không có điểm nào ở phía trước, bỏ qua.
                                                 if (min_front >= 999.0f)
                                                     return;
                                                 float min_dist_cm = min_front * 100.0f; // Chuyển từ mét sang cm
                                                 int smooth_speed = 0;
                                                 bool movement_active = false;
                                                 bool is_safe = false;
                                                 bool is_reversing = false;
                                                 bool has_pending_movement = false;

                                                 // Khóa mutex để cập nhật và đọc state_ một cách an toàn
                                                 {
                                                     std::lock_guard<std::mutex> lock(state_.state_mutex);
                                                     state_.current_front_distance = min_dist_cm;
                                                     state_.is_safe_to_move = min_dist_cm > EMERGENCY_STOP_DISTANCE_CM;
                                                     state_.last_safety_update = std::chrono::steady_clock::now().time_since_epoch().count();
                                                     movement_active = state_.movement_command_active;
                                                     is_safe = state_.is_safe_to_move;
                                                     has_pending_movement = state_.movement_pending;


                                                     // Kiểm tra xem có đang lùi không
                                                     {
                                                         std::lock_guard<std::mutex> lock_movement(movement_target_mutex_);
                                                         is_reversing = current_movement_.is_active && !current_movement_.is_forward;
                                                     }

                                                     LOG_INFO << "[Lidar Thread] is_reversing: " << (is_reversing ? "Yes" : "No")
                                                              << ", is_safe: " << (is_safe ? "Yes" : "No")
                                                              << ", movement_active: " << (movement_active ? "Yes" : "No");

                                                     // Cập nhật chuỗi trạng thái LiDAR
                                                     std::stringstream ss;
                                                     ss << "Points: " << points.size()
                                                        << ", FrontDist: " << std::fixed << std::setprecision(1)
                                                        << min_dist_cm << "cm, Safe: " << (state_.is_safe_to_move ? "Yes" : "No");
                                                     state_.last_lidar_data = ss.str();
                                                     LOG_INFO << "[LIDAR Thread][REALTIMECALLBACK] state_.last_lidar_data: " << state_.last_lidar_data;

                                                     // QUAN TRỌNG: Logic tính toán tốc độ mượt mà
                                                     if (movement_active || has_pending_movement)
                                                    {
                                                        if (is_safe)
                                                        {
                                                            // Luôn luôn là di chuyển tiến nếu an toàn
                                                            smooth_speed = calculateSmoothSpeed(min_dist_cm, true);
                                                            if (smooth_speed == 0 && has_pending_movement)
                                                            {
                                                                smooth_speed = MIN_START_SPEED;
                                                                state_.movement_pending = false; 
                                                                LOG_INFO << "[Lidar Thread] Starting forward with initial speed: " << smooth_speed;
                                                            }
                                                            else
                                                            {
                                                                LOG_INFO << "[Lidar Thread] Moving forward with speed: " << smooth_speed;
                                                            }
                                                        }
                                                        else
                                                        {
                                                            // Khi tiến nhưng không an toàn: Dừng
                                                            smooth_speed = 0;
                                                            LOG_WARNING << "[Lidar Thread] Forward blocked! Speed set to 0";
                                                        }
                                                    }
                                                    else
                                                    {
                                                        // Không có lệnh di chuyển
                                                        smooth_speed = calculateSmoothSpeed(min_dist_cm, false);
                                                    }

                                                     // Lưu smooth_speed vào state để heading correction dùng
                                                     state_.current_speed = smooth_speed;
                                                 }

                                                 // Chỉ gửi lệnh tốc độ đến PLC nếu giá trị tốc độ thay đổi để giảm tải.
                                                 static int last_sent_safety = -1;
                                                 int safety_signal = is_safe ? 1 : 0; // D103 = 1 (an toàn), 0 (không an toàn)

                                                 if (safety_signal != last_sent_safety)
                                                 {
                                                     plc_command_queue_.push("WRITE_D103_" + std::to_string(safety_signal));
                                                     last_sent_safety = safety_signal;
                                                     LOG_INFO << "[LIDAR Thread] Safety signal D103 = " << safety_signal;
                                                 }

                                                 // =================================================================
                                                 // LOGIC HEADING CORRECTION ĐƯỢC TÍCH HỢP VÀO ĐÂY
                                                 // =================================================================
                                                 applyHeadingCorrection();
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
        while (running_ && lidar_processor->isConnectionHealthy())
        {
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
            state_.is_safe_to_move = false; // Set unsafe khi mất LiDAR
        }

        // Gửi lệnh dừng khẩn cấp nếu đang di chuyển
        if (state_.is_moving)
        {
            plc_command_queue_.push("WRITE_D100_0"); // Stop
            plc_command_queue_.push("WRITE_D103_0"); // Safety OFF
            plc_command_queue_.push("WRITE_D104_0"); // Left wheel stop
            plc_command_queue_.push("WRITE_D105_0"); // Right wheel stop
            LOG_WARNING << "[LiDAR Thread] Emergency stop due to LiDAR disconnection";
        }

        // Đợi trước khi thử khởi động lại
        LOG_INFO << "[LiDAR Thread] Entering main loop.";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Đợi 1 giây trước khi thử lại
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
void SystemManager::command_handler_thread()
{
    pin_thread_to_core(3);
    LOG_INFO << "[Command Handler] Thread started. Waiting for commands from server...";

    while (running_)
    {
        ServerComm::NavigationCommand cmd;

        if (comm_server_->getNextCommand(cmd))
        {
            LOG_INFO << "[Command Handler] Processing command type: " << static_cast<int>(cmd.type);

            // KIỂM TRA LỆNH LẶP LẠI
            if (cmd.type == last_processed_cmd_type_)
            {
                LOG_INFO << "[Command Handler] Ignoring duplicate command: " << static_cast<int>(cmd.type);
                continue;
            }
            last_processed_cmd_type_ = cmd.type;

            bool is_safe;
            {
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                is_safe = state_.is_safe_to_move;
            }

            if (plc_in_error_state_ || plc_ptr_ == nullptr || !plc_ptr_->isConnected())
            {
                LOG_ERROR << "[Command Handler] Command ignored! PLC not ready";
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            switch (cmd.type)
            {
            case ServerComm::NavigationCommand::MOVE_TO_POINT:
            {
                LOG_INFO << "[Command Handler] Processing MOVE_TO_POINT command";

                if (is_safe)
                {
                    // LƯU LẠI GÓC HIỆN TẠI LÀM MỤC TIÊU
                    float current_heading;
                    {
                        std::lock_guard<std::mutex> lock(state_.state_mutex);
                        current_heading = state_.current_heading;
                        state_.movement_command_active = true;
                        state_.is_moving = false;
                    }

                    // Cập nhật movement target
                    {
                        std::lock_guard<std::mutex> lock(movement_target_mutex_);
                        current_movement_.is_active = true;
                        current_movement_.target_heading = current_heading;
                        current_movement_.heading_tolerance = 2.0f; // ±2°
                        current_movement_.is_forward = true;
                        current_movement_.start_time = std::chrono::steady_clock::now();

                        // Reset PID controller
                        heading_pid_.reset();
                    }

                    LOG_INFO << "[Command Handler] Target heading locked at: "
                             << current_heading << "°";

                    // Lưu lại lệnh
                    {
                        std::lock_guard<std::mutex> lock(last_command_mutex_);
                        last_movement_command_ = "WRITE_D100_1";
                    }

                    // Gửi lệnh di chuyển
                    plc_command_queue_.push("WRITE_D101_0"); // Hướng: thẳng
                    plc_command_queue_.push("WRITE_D100_1"); // Bắt đầu di chuyển
                }
                else
                {
                    LOG_WARNING << "[Command Handler] Cannot execute - obstacle detected!";
                }
                break;
            }

            // case ServerComm::NavigationCommand::REVERSE_TO_POINT:
            // {
            //     LOG_INFO << "[Command Handler] Processing REVERSE_TO_POINT command";

            //     // LƯU LẠI GÓC HIỆN TẠI
            //     float current_heading;
            //     {
            //         std::lock_guard<std::mutex> lock(state_.state_mutex);
            //         current_heading = state_.current_heading;
            //         state_.movement_command_active = true;
            //         state_.is_moving = false;
            //     }

            //     // Cập nhật movement target
            //     {
            //         std::lock_guard<std::mutex> lock(movement_target_mutex_);
            //         current_movement_.is_active = true;
            //         current_movement_.target_heading = current_heading;
            //         current_movement_.heading_tolerance = 3.0f; // ±3° (rộng hơn khi lùi)
            //         current_movement_.is_forward = false;
            //         current_movement_.start_time = std::chrono::steady_clock::now();

            //         heading_pid_.reset();
            //     }

            //     LOG_INFO << "[Command Handler] Reverse target heading locked at: "
            //              << current_heading << "°";

            //     {
            //         std::lock_guard<std::mutex> lock(last_command_mutex_);
            //         last_movement_command_ = "WRITE_D100_2";
            //     }

            //     plc_command_queue_.push("WRITE_D101_0");
            //     plc_command_queue_.push("WRITE_D100_2"); // Lùi
            //     break;
            // }

            case ServerComm::NavigationCommand::ROTATE_TO_LEFT:
            case ServerComm::NavigationCommand::ROTATE_TO_RIGHT:
            {
                LOG_INFO << "[Command Handler] Processing 90-degree arc rotation";
                if (is_safe)
                {
                    float start_heading;
                    {
                        std::lock_guard<std::mutex> lock(state_.state_mutex);
                        start_heading = state_.current_heading;
                        state_.movement_command_active = true;
                        state_.is_moving = false; // Chờ gia tốc
                        state_.movement_pending = true;
                    }

                    // ~~~ SỬA: Cập nhật movement target cho việc rẽ ~~~
                    {
                        std::lock_guard<std::mutex> lock(movement_target_mutex_);
                        current_movement_.is_active = true;
                        current_movement_.is_turning = true; // Đánh dấu đang rẽ
                        current_movement_.turn_start_heading = start_heading; // Lưu góc bắt đầu
                        current_movement_.start_time = std::chrono::steady_clock::now();
                    }

                    {
                        std::lock_guard<std::mutex> lock(arc_direction_mutex_);
                        arc_direction_ = (cmd.type == ServerComm::NavigationCommand::ROTATE_TO_LEFT) ? ArcDirection::LEFT : ArcDirection::RIGHT;
                    }

                    LOG_INFO << "[Command Handler] Arc turn started from " << start_heading << " degrees.";
                     // Gửi lệnh di chuyển tiến để AGV có thể bắt đầu rẽ
                // Lệnh D101 chỉ có tác dụng khi D100=1
                std::string cmd_send = (cmd.type == ServerComm::NavigationCommand::ROTATE_TO_LEFT) ? "WRITE_D101_1" : "WRITE_D101_2";

                plc_command_queue_.push("WRITE_D100_1");
                plc_command_queue_.push(cmd_send);

                }
                else
                {
                    LOG_WARNING << "[Command Handler] Cannot execute rotation - obstacle detected!";
                }
                break;
            }

            case ServerComm::NavigationCommand::STOP:
            case ServerComm::NavigationCommand::EMERGENCY_STOP:
            {
                LOG_INFO << "[Command Handler] Executing STOP command";

                // VÔ HIỆU HÓA heading correction
                {
                    std::lock_guard<std::mutex> lock(movement_target_mutex_);
                    current_movement_.is_active = false;
                    heading_pid_.reset();
                }

                plc_command_queue_.push("WRITE_D100_0");
                plc_command_queue_.push("WRITE_D101_0");

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

            default:
                LOG_WARNING << "[Command Handler] Unknown command type";
                break;
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
void SystemManager::safety_monitor_thread()
{
    pin_thread_to_core(0);
    LOG_INFO << "[Safety Monitor] Started.";
    bool was_safe = true;

    while (running_)
    {
        bool is_safe;
        {
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            is_safe = state_.is_safe_to_move;
        }

        if (was_safe && !is_safe)
        {
            LOG_WARNING << "[Safety Monitor] DANGER! Stopping movement";

            // VÔ HIỆU HÓA heading correction
            {
                std::lock_guard<std::mutex> lock(movement_target_mutex_);
                current_movement_.is_active = false;
                heading_pid_.reset();
            }

            plc_command_queue_.push("WRITE_D100_0");
            plc_command_queue_.push("WRITE_D101_0");
        }
        else if (!was_safe && is_safe)
        {
            LOG_INFO << "[Safety Monitor] Path clear. Checking for command to resume...";
            std::lock_guard<std::mutex> lock(last_command_mutex_);

            if (!last_movement_command_.empty() && !plc_in_error_state_)
            {
                LOG_INFO << "[Safety Monitor] Resuming: " << last_movement_command_;

                // KHÔI PHỤC heading correction
                float current_heading;
                {
                    std::lock_guard<std::mutex> lock2(state_.state_mutex);
                    current_heading = state_.current_heading;
                }

                {
                    std::lock_guard<std::mutex> lock2(movement_target_mutex_);
                    current_movement_.is_active = true;
                    current_movement_.target_heading = current_heading;
                    heading_pid_.reset();
                }

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
void SystemManager::plc_periodic_tasks_thread()
{
    pin_thread_to_core(1);
    LOG_INFO << "[PLC Periodic] Thread started.";
    std::this_thread::sleep_for(std::chrono::seconds(5));

    while (running_)
    {
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
void SystemManager::battery_thread_func()
{
    pin_thread_to_core(2);
    LOG_INFO << "[Battery Thread] Starting...";

    JBDBMSSingleton &bms = JBDBMSSingleton::getInstance(BATTERY_BMS_SERIAL_PORT);
    bool is_bms_initialized = false; // Cờ theo dõi trạng thái kết nối cục bộ

    // Vòng lặp quản lý trạng thái chính
    while (running_)
    {
        if (is_bms_initialized)
        {
            // ----- TRẠNG THÁI: ĐÃ KẾT NỐI -> CẬP NHẬT DỮ LIỆU -----
            if (bms.updateBatteryData())
            { // Cập nhật thành công
                BatteryData data = bms.getBatteryData();
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                state_.battery_connected = true;
                state_.battery_level = data.stateOfCharge;
            }
            else
            {
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
        }
        else
        {
            // ----- TRẠNG THÁI: MẤT KẾT NỐI -> THỬ KẾT NỐI LẠI -----
            LOG_INFO << "[Battery Thread] Attempting to initialize BMS connection...";
            if (bms.initialize())
            { // Kết nối thành công
                LOG_INFO << "[Battery Thread] BMS connected successfully.";
                is_bms_initialized = true; // Đánh dấu đã kết nối thành công
            }
            else
            {
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
std::vector<ServerComm::Point2D> SystemManager::sampleImportantPoints(const std::vector<ServerComm::Point2D> &points)
{
    if (points.empty())
    {
        return {};
    }

    std::vector<ServerComm::Point2D> result;
    // Dành trước bộ nhớ để tăng hiệu năng, tránh cấp phát lại nhiều lần
    result.reserve(points.size() / 2); // Ước tính kích thước sau khi sampling

    // Duyệt qua tất cả các điểm đầu vào
    for (const auto &p : points)
    {
        // Tính khoảng cách từ điểm đến tâm (vị trí của LiDAR)
        float dist = sqrt(p.x * p.x + p.y * p.y);

        if (dist < 2.0f)
        {
            // Giữ lại 100% các điểm ở khoảng cách dưới 2 mét
            result.push_back(p);
        }
        else if (dist < 5.0f && (rand() % 2 == 0))
        {
            // Giữ lại ngẫu nhiên 50% các điểm ở khoảng cách từ 2 đến 5 mét
            result.push_back(p);
        }
        else if (rand() % 4 == 0)
        {
            // Giữ lại ngẫu nhiên 25% các điểm ở khoảng cách xa hơn 5 mét
            result.push_back(p);
        }
    }

    return result;
}

/**
 * @brief Luồng xử lý dữ liệu từ cảm biến IMU MPU9250.
 * @details
 * - Ghim luồng vào CPU core 2.
 * - Khởi tạo và bắt đầu AGVNavigation.
 * - Đọc dữ liệu real-time từ MPU9250 với tần số ~100Hz.
 * - Cập nhật orientation (heading, roll, pitch) cho điều hướng AGV.
 * - Kiểm tra an toàn (nghiêng, va chạm).
 * - Cập nhật state_ để các thread khác sử dụng.
 */
void SystemManager::imu_thread_func()
{
    pin_thread_to_core(2);
    LOG_INFO << "[IMU Thread] Starting MPU9250 navigation system...";

    // Vòng lặp chính để quản lý kết nối và xử lý dữ liệu IMU
    while (running_)
    {
        imu_ptr_ = std::make_unique<AGVNavigation>();

        // Thử khởi tạo với retry
        int init_attempts = 0;
        bool initialized = false;

        while (init_attempts < 3 && running_)
        {
            if (imu_ptr_->begin("/dev/i2c-1"))
            {
                initialized = true;
                LOG_INFO << "[IMU Thread] MPU9250 initialized successfully";
                break;
            }

            LOG_WARNING << "[IMU Thread] Initialization attempt "
                        << (init_attempts + 1) << " failed. Retrying in 3s...";
            std::this_thread::sleep_for(std::chrono::seconds(3));
            init_attempts++;
        }

        if (!initialized)
        {
            LOG_ERROR << "[IMU Thread] Failed to initialize after 3 attempts";
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            state_.imu_connected = false;
            state_.last_imu_data = "IMU initialization failed";

            // Đợi 10 giây rồi thử lại
            std::this_thread::sleep_for(std::chrono::seconds(10));
            continue;
        }

        // Hiệu chuẩn gyroscope
        LOG_INFO << "[IMU Thread] Calibrating gyroscope...";
        imu_ptr_->calibrateGyro(2000);

        // Cấu hình tham số an toàn
        imu_ptr_->setMaxTiltAngle(30.0f);      // 30° max tilt
        imu_ptr_->setCollisionThreshold(3.0f); // 3g collision threshold
        imu_ptr_->setFilterBeta(0.08f);        // Madgwick filter gain

        {
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            state_.imu_connected = true;
            state_.last_imu_data = "IMU connected and calibrated";
        }

        LOG_INFO << "[IMU Thread] Entering main loop at ~100Hz";

        // Main processing loop
        int consecutive_errors = 0;
        const int MAX_CONSECUTIVE_ERRORS = 50; // ~0.5 giây ở 100Hz

        while (running_ && consecutive_errors < MAX_CONSECUTIVE_ERRORS)
        {
            try
            {
                // Update sensor fusion (Madgwick filter)
                imu_ptr_->update();

                // Read processed data
                float heading, roll, pitch;
                imu_ptr_->getOrientation(heading, roll, pitch);

                // Read raw sensor data
                float ax, ay, az, gx, gy, gz, mx, my, mz;
                imu_ptr_->getRawAccel(ax, ay, az);
                imu_ptr_->getRawGyro(gx, gy, gz);
                imu_ptr_->getRawMag(mx, my, mz);
                float temp = imu_ptr_->getTemperature();

                // Safety checks
                bool is_safe = imu_ptr_->isSafe();
                bool is_tilted = imu_ptr_->isTilted();
                const char *compass_dir = imu_ptr_->getCompassDirection();

                // Update shared state
                {
                    std::lock_guard<std::mutex> lock(state_.state_mutex);
                    state_.imu_connected = true;
                    state_.current_heading = heading;
                    state_.current_roll = roll;
                    state_.current_pitch = pitch;
                    state_.imu_tilt_warning = is_tilted;

                    // Update status string
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(1);
                    ss << "Heading: " << heading << "° (" << compass_dir << "), "
                       << "Roll: " << roll << "°, "
                       << "Pitch: " << pitch << "°, "
                       << "Safe: " << (is_safe ? "Yes" : "No");
                    state_.last_imu_data = ss.str();
                }

                // Update local IMU data structure (for detailed access)
                {
                    std::lock_guard<std::mutex> lock(imu_data_mutex_);
                    latest_imu_data_.heading = heading;
                    latest_imu_data_.roll = roll;
                    latest_imu_data_.pitch = pitch;
                    latest_imu_data_.accel_x = ax;
                    latest_imu_data_.accel_y = ay;
                    latest_imu_data_.accel_z = az;
                    latest_imu_data_.gyro_x = gx;
                    latest_imu_data_.gyro_y = gy;
                    latest_imu_data_.gyro_z = gz;
                    latest_imu_data_.temperature = temp;
                    latest_imu_data_.is_tilted = is_tilted;
                    latest_imu_data_.is_safe = is_safe;
                    latest_imu_data_.compass_direction = compass_dir;
                    latest_imu_data_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                     std::chrono::system_clock::now().time_since_epoch())
                                                     .count();
                }

                // Log periodically (mỗi giây)
                static int log_counter = 0;
                if (++log_counter >= 50)
                { // Log mỗi 0.5 giây (50 samples @ 100Hz)
                    // Lấy giá trị yaw chưa chuẩn hóa để debug
                    float raw_yaw = imu_ptr_->getRawYaw();

                    std::stringstream imu_log_ss;
                    imu_log_ss << std::fixed << std::setprecision(2)
                               << "[IMU] Heading: " << heading << "° (" << compass_dir << ")"
                               << " | Raw Yaw: " << raw_yaw << "°"
                               << " | Roll: " << roll << "°"
                               << " | Pitch: " << pitch << "°"
                               << " | Gyro(Z): " << gz << " dps";
                    LOG_INFO << imu_log_ss.str();

                    // LOG_INFO << "[IMU Thread] Heading: " << heading << "° (" << compass_dir
                    //          << "), Roll: " << roll << "°, Pitch: " << pitch
                    //          << "°, Temp: " << temp << "°C";
                    log_counter = 0; // Reset counter
                }

                // Cảnh báo nếu nghiêng nguy hiểm
                if (is_tilted && !is_safe)
                {
                    LOG_WARNING << "[IMU Thread] DANGER: Excessive tilt detected! Roll: "
                                << roll << "°, Pitch: " << pitch << "°";
                }

                consecutive_errors = 0; // Reset error counter on success
            }
            catch (const std::exception &e)
            {
                LOG_ERROR << "[IMU Thread] Error reading IMU data: " << e.what();
                consecutive_errors++;
            }

            // Run at ~100Hz (10ms period)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Nếu tới đây nghĩa là mất kết nối hoặc quá nhiều lỗi
        LOG_WARNING << "[IMU Thread] IMU connection lost or too many errors. Restarting...";
        imu_ptr_.reset();

        {
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            state_.imu_connected = false;
            state_.last_imu_data = "IMU disconnected";
        }

        // Đợi trước khi thử khởi động lại
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    LOG_INFO << "[IMU Thread] Stopped.";
}

/**
 * @brief Chuẩn hóa góc về khoảng [0, 360)
 */
float SystemManager::normalizeAngle(float angle)
{
    while (angle < 0.0f)
        angle += 360.0f;
    while (angle >= 360.0f)
        angle -= 360.0f;
    return angle;
}

/**
 * @brief Tính sai số góc ngắn nhất giữa góc hiện tại và góc mục tiêu
 * @return Sai số trong khoảng [-180, 180]
 */
float SystemManager::calculateHeadingError(float current, float target)
{
    current = normalizeAngle(current);
    target = normalizeAngle(target);

    float error = target - current;

    // Tìm đường đi ngắn nhất
    if (error > 180.0f)
    {
        error -= 360.0f;
    }
    else if (error < -180.0f)
    {
        error += 360.0f;
    }

    return error;
}

/**
 * @brief Áp dụng logic giữ hướng và điều khiển tốc độ vi sai.
 * @details Hàm này được gọi liên tục từ `lidar_thread_func`. Nó chịu trách nhiệm:
 * - Lấy mục tiêu di chuyển và trạng thái hiện tại.
 * - Dừng bánh xe nếu không an toàn hoặc không có lệnh di chuyển.
 * - Tính toán sai số góc và áp dụng PID để điều chỉnh tốc độ 2 bánh.
 * - Xử lý logic rẽ vòng cung (arc turn).
 */
void SystemManager::applyHeadingCorrection()
{
    MovementTarget target; // Mục tiêu di chuyển hiện tại
    float current_heading; // Heading hiện tại
    int base_speed;        // Tốc độ cơ sở (đã được làm mượt)
    bool is_safe;          // Trạng thái an toàn

    // Lấy thông tin movement target và các trạng thái cần thiết
    {
        std::lock_guard<std::mutex> lock(movement_target_mutex_);
        target = current_movement_; // Sao chép mục tiêu hiện tại
    }
    {
        std::lock_guard<std::mutex> lock(state_.state_mutex);
        current_heading = state_.current_heading;
        base_speed = state_.current_speed;
        is_safe = state_.is_safe_to_move;
    }

    // Nếu không có lệnh di chuyển (tiến/lùi), không làm gì cả.
    if (!target.is_active)
    {
        return;
    }

    // QUAN TRỌNG: Kiểm tra pending trước khi check base_speed
    bool check_pending = false;
    {
        std::lock_guard<std::mutex> lock(state_.state_mutex);
        check_pending = state_.movement_pending;
    }

    // Điều kiện dừng bánh xe:
    // 1. Tốc độ cơ bản = 0 (đã dừng hoặc đang giảm tốc về 0).
    // 2. HOẶC đang di chuyển tiến VÀ không an toàn.
    //    (Lưu ý: Điều kiện an toàn không áp dụng khi lùi).
    if (base_speed == 0 || /*(target.is_forward && !is_safe)*/ check_pending)
    {
            if (target.is_forward && is_safe)
        {
            base_speed = MIN_START_SPEED;
            LOG_INFO << "[LIDAR/Heading] Starting with pending, speed: " << base_speed;
        }

        return;
    }

    // Nếu vẫn = 0 và không phải lùi, dừng
    if (base_speed == 0 )//&& !is_reversing)
    {
        plc_command_queue_.push("WRITE_D104_0");
        plc_command_queue_.push("WRITE_D105_0");
        return;
    }

    // Xử lý logic rẽ vòng cung (ưu tiên cao hơn giữ hướng)
    ArcDirection direction;
    {
        std::lock_guard<std::mutex> lock(arc_direction_mutex_);
        direction = arc_direction_;
    }

    // Nếu đang rẽ, kiểm tra xem đã đủ góc 90 độ chưa
    if (target.is_turning) {
        float angle_turned = fabs(calculateHeadingError(current_heading, target.turn_start_heading));
        if (angle_turned >= 90.0f) {
            LOG_INFO << "[Heading Correction] 90-degree turn complete. Stopping.";
            plc_command_queue_.push("WRITE_D100_0"); // Gửi lệnh dừng
            
            // Reset trạng thái di chuyển
            {
                std::lock_guard<std::mutex> lock(movement_target_mutex_);
                current_movement_.is_active = false;
                current_movement_.is_turning = false;
            }
            {
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                state_.movement_command_active = false;
                state_.is_moving = false;
            }
            return; // Dừng xử lý thêm
        }
    }
    if (direction != ArcDirection::NONE)
    {

            // Reset arc sau khi xử lý
        {
            std::lock_guard<std::mutex> lock(arc_direction_mutex_);
            arc_direction_ = ArcDirection::NONE;
        }

        int outer_speed = base_speed;
        int inner_speed = base_speed / 2; // Bánh trong chạy bằng nửa tốc độ

        if (direction == ArcDirection::LEFT)
        {
            LOG_INFO << "[LIDAR/ARC] Arc Left - BaseSpeed: " << base_speed << " -> L:" << outer_speed << ", R:" << inner_speed;
            plc_command_queue_.push("WRITE_D104_" + std::to_string(outer_speed)); // Bánh trái (ngoài)
            plc_command_queue_.push("WRITE_D105_" + std::to_string(inner_speed)); // Bánh phải (trong)
        }
        else // ArcDirection::RIGHT
        {
            LOG_INFO << "[LIDAR/ARC] Arc Right - BaseSpeed: " << base_speed << " -> L:" << inner_speed << ", R:" << outer_speed;
            plc_command_queue_.push("WRITE_D104_" + std::to_string(inner_speed)); // Bánh trái (trong)
            plc_command_queue_.push("WRITE_D105_" + std::to_string(outer_speed)); // Bánh phải (ngoài)
        }
        return; // Kết thúc sớm vì đang rẽ vòng cung
    }

    // Xử lý giữ hướng khi đi thẳng/lùi
    float heading_error = calculateHeadingError(current_heading, target.target_heading);

    // Nếu lệch hướng, tính toán lại tốc độ 2 bánh
    if (fabs(heading_error) > target.heading_tolerance)
    {
        LOG_WARNING << "[LIDAR/Heading] Deviation! Err: " << heading_error << "°. Correcting...";
        int left_speed, right_speed;

        // Giảm độ nhạy khi lùi bằng cách giảm sai số
        float effective_error = target.is_forward ? heading_error : heading_error * 0.5f;
        calculateDifferentialSpeed(effective_error, base_speed, left_speed, right_speed);

        plc_command_queue_.push("WRITE_D104_" + std::to_string(left_speed));
        plc_command_queue_.push("WRITE_D105_" + std::to_string(right_speed));
    }
    else
    {
        // Nếu đi đúng hướng, 2 bánh cùng tốc độ
        plc_command_queue_.push("WRITE_D104_" + std::to_string(base_speed));
        plc_command_queue_.push("WRITE_D105_" + std::to_string(base_speed));
    }
}
/**
 * @brief Tính toán tốc độ 2 bánh để điều chỉnh hướng
 * @param heading_error Sai số góc (độ), + nghĩa là cần quay phải
 * @param base_speed Tốc độ cơ bản (smooth_speed từ LiDAR)
 * @param left_speed Output: tốc độ bánh trái
 * @param right_speed Output: tốc độ bánh phải
 *
 * LOGIC:
 * - heading_error > 0 (cần quay PHẢI): GIẢM tốc bánh PHẢI
 * - heading_error < 0 (cần quay TRÁI): GIẢM tốc bánh TRÁI
 */
void SystemManager::calculateDifferentialSpeed(float heading_error, int base_speed,
                                               int &left_speed, int &right_speed)
{
    // Tính toán PID
    heading_pid_.integral += heading_error;

    // Anti-windup: giới hạn integral
    if (heading_pid_.integral > heading_pid_.max_integral)
    {
        heading_pid_.integral = heading_pid_.max_integral;
    }
    else if (heading_pid_.integral < -heading_pid_.max_integral)
    {
        heading_pid_.integral = -heading_pid_.max_integral;
    }

    float derivative = heading_error - heading_pid_.last_error;
    heading_pid_.last_error = heading_error;

    // PID output
    float correction = (heading_pid_.kp * heading_error) +
                       (heading_pid_.ki * heading_pid_.integral) +
                       (heading_pid_.kd * derivative);

    // Giới hạn correction (max 50% của base_speed)
    float max_correction = base_speed * 0.5f;
    correction = std::max(-max_correction, std::min(max_correction, correction));

    // =====================================================
    // TÍNH TỐC ĐỘ 2 BÁNH
    // =====================================================
    // CHÚ Ý: Với cấu hình Z-up, X-forward, Y-left:
    // heading_error > 0: Cần quay PHẢI → Giảm tốc bánh PHẢI
    // heading_error < 0: Cần quay TRÁI → Giảm tốc bánh TRÁI
    // =====================================================

    left_speed = base_speed + static_cast<int>(correction);
    right_speed = base_speed - static_cast<int>(correction);

    // Đảm bảo tốc độ trong giới hạn [0, 3000]
    left_speed = std::max(0, std::min(3000, left_speed));
    right_speed = std::max(0, std::min(3000, right_speed));

    LOG_INFO << "[PID] Base: " << base_speed
             << " | Err: " << std::fixed << std::setprecision(2) << heading_error << "°"
             << " | P: " << (heading_pid_.kp * heading_error)
             << " | I: " << (heading_pid_.ki * heading_pid_.integral)
             << " | D: " << (heading_pid_.kd * derivative)
             << " | Corr: " << correction
             << " -> L:" << left_speed << ", R:" << right_speed;
}


#if TEST_KEYBOARD_MODE == 1
/**
 * @brief Tìm device bàn phím USB trong /dev/input/
 * @return Đường dẫn đến device (vd: /dev/input/event0) hoặc rỗng nếu không tìm thấy
 */
std::string SystemManager::findKeyboardDevice()
{
    DIR *dir = opendir("/dev/input");
    if (!dir)
    {
        LOG_ERROR << "[Keyboard] Cannot open /dev/input";
        return "";
    }

    struct dirent *entry;
    std::string found_device;

    while ((entry = readdir(dir)) != nullptr)
    {
        // Chỉ kiểm tra các file event*
        if (strncmp(entry->d_name, "event", 5) != 0)
        {
            continue;
        }

        std::string device_path = std::string("/dev/input/") + entry->d_name;
        int fd = open(device_path.c_str(), O_RDONLY);

        if (fd < 0)
        {
            continue;
        }

        char name[256] = "Unknown";
        ioctl(fd, EVIOCGNAME(sizeof(name)), name);

        // Tìm bàn phím (chứa "keyboard" hoặc "Keyboard")
        std::string device_name(name);
        std::transform(device_name.begin(), device_name.end(), device_name.begin(), ::tolower);

        if (device_name.find("keyboard") != std::string::npos)
        {
            found_device = device_path;
            LOG_INFO << "[Keyboard] Found keyboard: " << name << " at " << device_path;
            close(fd);
            break;
        }

        close(fd);
    }

    closedir(dir);
    return found_device;
}

/**
 * @brief Luồng điều khiển AGV bằng bàn phím USB (WASD + B)
 * @details
 * - Đọc trực tiếp từ /dev/input/eventX (raw input events)
 * - Không cần terminal, hoạt động như service
 * - W: Tiến
 * - A: Xoay trái
 * - S: Lùi
 * - D: Xoay phải
 * - B: DỪNG (STOP)
 *
 * Ghim vào CPU core 3
 */
void SystemManager::keyboard_control_thread()
{
    pin_thread_to_core(3);
    LOG_INFO << "[Keyboard Control] Thread started";

    // Tìm bàn phím
    std::string keyboard_device = "/dev/input/event5"; // findKeyboardDevice();

    if (keyboard_device.empty())
    {
        LOG_WARNING << "[Keyboard Control] No keyboard found. Thread disabled.";
        LOG_WARNING << "[Keyboard Control] Please plug in a USB keyboard and restart service.";
        return;
    }

    // Mở device
    int fd = open(keyboard_device.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
    {
        LOG_ERROR << "[Keyboard Control] Cannot open " << keyboard_device
                  << ": " << strerror(errno);
        return;
    }

    // LOG_INFO << "[Keyboard Control] ===================================";
    // LOG_INFO << "[Keyboard Control]   ĐIỀU KHIỂN AGV BẰNG BÀN PHÍM   ";
    // LOG_INFO << "[Keyboard Control] ===================================";
    // LOG_INFO << "[Keyboard Control]   W - Tiến                        ";
    // LOG_INFO << "[Keyboard Control]   S - Lùi                         ";
    // LOG_INFO << "[Keyboard Control]   A - Xoay trái                   ";
    // LOG_INFO << "[Keyboard Control]   D - Xoay phải                   ";
    // LOG_INFO << "[Keyboard Control]   B - DỪNG                        ";
    // LOG_INFO << "[Keyboard Control] ===================================";

    struct input_event ev;
    char last_command = '\0';

    while (running_)
    {
        ssize_t bytes = read(fd, &ev, sizeof(ev));

        if (bytes < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // Không có dữ liệu, sleep ngắn
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            else
            {
                LOG_ERROR << "[Keyboard Control] Read error: " << strerror(errno);
                break;
            }
        }

        if (bytes != sizeof(ev))
        {
            continue;
        }

        // Chỉ xử lý key press events (EV_KEY, value=1)
        if (ev.type != EV_KEY || ev.value != 1)
        {
            continue;
        }

        char command = '\0';

        // Map key codes sang commands
        switch (ev.code)
        {
        case KEY_W:
            command = 'w';
            break;
        case KEY_A:
            command = 'a';
            break;
        case KEY_S:
            command = 's';
            break;
        case KEY_D:
            command = 'd';
            break;
        case KEY_B:
            command = 'b';
            break;
        default:
            continue; // Bỏ qua các phím khác
        }

        // Bỏ qua nếu trùng lệnh trước (trừ STOP)
        if (command == last_command && command != 'b')
        {
            continue;
        }

        // Kiểm tra PLC
        bool plc_ready = false;
        {
            std::lock_guard<std::mutex> lock(plc_ptr_mutex_);
            plc_ready = (plc_ptr_ != nullptr && plc_ptr_->isConnected());
        }

        if (!plc_ready)
        {
            LOG_WARNING << "[Keyboard Control] PLC not connected! Command ignored.";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Xử lý commands
        switch (command)
        {
        case 'w':
        {
            // TIẾN
            LOG_INFO << "[Keyboard Control]  FORWARD";
            LOG_INFO << "[khaipv] command forward received";

            bool is_safe;
            float current_heading;
            {
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                is_safe = state_.is_safe_to_move;
                current_heading = state_.current_heading;
                state_.movement_command_active = true;
                state_.is_moving = false;
                state_.movement_pending = true; // Báo hiệu có lệnh di chuyển mới
            }

            // Yêu cầu mới: Giữ lệnh nếu không an toàn
            if (!is_safe)
            {
                LOG_WARNING << "[Keyboard Control] Path blocked. Command 'FORWARD' is pending.";
                // Lệnh đã được lưu trong state_, luồng safety_monitor sẽ kích hoạt lại sau.
                continue;
            }

             {
                std::lock_guard<std::mutex> lock(arc_direction_mutex_);
                arc_direction_ = ArcDirection::NONE; // Tắt chế độ rẽ vòng cung khi tiến
            }

            if (is_safe)
            {
                {
                    std::lock_guard<std::mutex> lock(last_command_mutex_);
                    last_movement_command_ = "WRITE_D100_1";
                }

                plc_command_queue_.push("WRITE_D101_0");
                plc_command_queue_.push("WRITE_D100_1");

                // Lưu heading target
                {
                    std::lock_guard<std::mutex> lock(movement_target_mutex_);
                    current_movement_.is_active = true;
                    current_movement_.target_heading = current_heading;
                    current_movement_.heading_tolerance = 2.0f;
                    current_movement_.is_forward = true;
                    current_movement_.is_turning = false; // Không phải đang rẽ
                    current_movement_.start_time = std::chrono::steady_clock::now();
                    heading_pid_.reset();
                }
                LOG_INFO << "[Keyboard Control] Target heading locked: " << current_heading << "°";
            }
            else
            {
                LOG_WARNING << "[Keyboard Control] Không an toàn! Có vật cản phía trước.";
            }
            last_command = command;
            break;
        }

        case 's':
        {
            // LÙI
            LOG_INFO << "[Keyboard Control] REVERSE";
            LOG_INFO << "[khaipv] command reverse received";
            float current_heading;
            {
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                current_heading = state_.current_heading;
                state_.movement_command_active = true;
                state_.is_moving = false;
                state_.movement_pending = true; // Báo hiệu có lệnh di chuyển mới
            }

            {
                std::lock_guard<std::mutex> lock(arc_direction_mutex_);
                arc_direction_ = ArcDirection::NONE; // Tắt chế độ rẽ vòng cung khi lùi
            }

            plc_command_queue_.push("WRITE_D101_0");
            plc_command_queue_.push("WRITE_D100_2");

            {
                std::lock_guard<std::mutex> lock(movement_target_mutex_);
                current_movement_.is_active = true;
                current_movement_.target_heading = current_heading;
                current_movement_.heading_tolerance = 3.0f;
                current_movement_.is_turning = false; // Không phải đang rẽ
                current_movement_.is_forward = false;
                current_movement_.start_time = std::chrono::steady_clock::now();
                heading_pid_.reset();
            }

            {
                std::lock_guard<std::mutex> lock(last_command_mutex_);
                last_movement_command_ = "WRITE_D100_2";
            }

            LOG_INFO << "[Keyboard Control] Reverse target heading: " << current_heading << "°";
            last_command = command;
            break;
        }

        case 'a':
        { // D105 giảm
            // XOAY TRÁI THEO VÒNG CUNG
            LOG_INFO << "[Keyboard Control] ARC LEFT";
            LOG_INFO << "[khaipv] command arc left received";
            // Tắt heading correction
            {
                // Vẫn giữ active để tính tốc độ, nhưng is_turning sẽ được dùng để bỏ qua heading correction
                // std::lock_guard<std::mutex> lock(movement_target_mutex_);
                // current_movement_.is_active = false;
            }
            bool is_safe;
            float start_heading;
            {
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                is_safe = state_.is_safe_to_move;
                //current_heading = state_.current_heading;
                state_.movement_command_active = true;
                state_.is_moving = false;
                state_.movement_pending = true; // Báo hiệu có lệnh di chuyển mới
                start_heading = state_.current_heading;
            }

            // Đặt trạng thái rẽ vòng cung
            {
                std::lock_guard<std::mutex> lock(arc_direction_mutex_);
                arc_direction_ = ArcDirection::LEFT;
            }
            // Yêu cầu mới: Giữ lệnh nếu không an toàn
            if (!is_safe)
            {
                LOG_WARNING << "[Keyboard Control] Path blocked. Command 'ARC LEFT' is pending.";
                continue;
            }

            if (is_safe)
            {
                
                {
                    std::lock_guard<std::mutex> lock(last_command_mutex_);
                    last_movement_command_ = "WRITE_D101_2";
                }

                // Lưu trạng thái bắt đầu rẽ
                {
                    std::lock_guard<std::mutex> lock(movement_target_mutex_);
                    current_movement_.is_active = true;
                    current_movement_.is_turning = true; // Đánh dấu đang rẽ
                    current_movement_.turn_start_heading = start_heading;
                    current_movement_.is_forward = true;
                }

                // Gửi lệnh di chuyển tiến để AGV có thể bắt đầu rẽ
                // Lệnh D101 chỉ có tác dụng khi D100=1
                plc_command_queue_.push("WRITE_D100_1");
                plc_command_queue_.push("WRITE_D101_2");

                LOG_INFO << "[Keyboard Control] Arc Left started from heading: " << start_heading << "°";
            }
            else
            {
                LOG_WARNING << "[Keyboard Control] Không an toàn! Có vật cản phía trước.";
            }

            last_command = command;
            break;
        }

        case 'd':
        { // D104 giảm
            // XOAY PHẢI THEO VÒNG CUNG
            LOG_INFO << "[Keyboard Control] ARC RIGHT";
            LOG_INFO << "[khaipv] command arc right received";
            // Tắt heading correction
            {
                // std::lock_guard<std::mutex> lock(movement_target_mutex_);
                // current_movement_.is_active = false;
            }
            bool is_safe;
            // Đặt trạng thái rẽ vòng cung
            float start_heading;
            {
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                is_safe = state_.is_safe_to_move;
                //current_heading = state_.current_heading;
                state_.movement_command_active = true;
                state_.is_moving = false;
                state_.movement_pending = true; // Báo hiệu có lệnh di chuyển mới
                start_heading = state_.current_heading;
            }

            // Đặt trạng thái rẽ vòng cung
            {
                std::lock_guard<std::mutex> lock(arc_direction_mutex_);
                arc_direction_ = ArcDirection::RIGHT;
            }

            // Yêu cầu mới: Giữ lệnh nếu không an toàn
            if (!is_safe)
            {
                LOG_WARNING << "[Keyboard Control] Path blocked. Command 'ARC RIGHT' is pending.";
                continue;
            }

            if (is_safe)
            {
                
                {
                    std::lock_guard<std::mutex> lock(last_command_mutex_);
                    last_movement_command_ = "WRITE_D101_1";
                }
                // Lưu trạng thái bắt đầu rẽ
                {
                    std::lock_guard<std::mutex> lock(movement_target_mutex_);
                    current_movement_.is_active = true;
                    current_movement_.is_turning = true; // Đánh dấu đang rẽ
                    current_movement_.turn_start_heading = start_heading;
                    current_movement_.is_forward = true;
                }

                // Gửi lệnh di chuyển tiến để AGV có thể bắt đầu rẽ
                // Lệnh D101 chỉ có tác dụng khi D100=1
                plc_command_queue_.push("WRITE_D100_1");
                plc_command_queue_.push("WRITE_D101_1");

                LOG_INFO << "[Keyboard Control] Arc Right started from heading: " << start_heading << "°";
            }

            last_command = command;
            break;
        }

        case 'b':
        {
            // DỪNG
            LOG_INFO << "[Keyboard Control] STOP";
            LOG_INFO << "[khaipv] command stop received";
            // VÔ HIỆU HÓA heading correction
            {
                std::lock_guard<std::mutex> lock(movement_target_mutex_);
                current_movement_.is_active = false;
                current_movement_.is_turning = false;
                heading_pid_.reset();
            }
            
            // Tắt chế độ rẽ vòng cung
            {
                std::lock_guard<std::mutex> lock(arc_direction_mutex_);
                arc_direction_ = ArcDirection::NONE;
            }

            plc_command_queue_.push("WRITE_D100_0");
            plc_command_queue_.push("WRITE_D101_0");
            plc_command_queue_.push("WRITE_D104_0");
            plc_command_queue_.push("WRITE_D105_0");

            {
                std::lock_guard<std::mutex> lock(last_command_mutex_);
                last_movement_command_.clear();
            }

            {
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                state_.movement_command_active = false;
                state_.is_moving = false;
                state_.movement_pending = false; // Hủy mọi lệnh đang chờ
            }

            last_command = '\0'; // Reset để cho phép dừng liên tục
            break;
        }
        }
    }

    close(fd);
    LOG_INFO << "[Keyboard Control] Thread stopped";
}

#endif

// Lưu ý: Luồng server_communication_thread không còn được quản lý trực tiếp ở đây.
// Nó được quản lý bên trong lớp ServerComm::CommunicationServer,
// vốn sử dụng các luồng riêng (`std::thread`) để xử lý I/O mạng một cách bất đồng bộ.
// Việc khởi tạo và bắt đầu nó được thực hiện trong hàm `initialize()`.
