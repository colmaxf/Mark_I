// servercommunicator.h
#ifndef SERVER_COMMUNICATOR_H
#define SERVER_COMMUNICATOR_H
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <functional>
#include <queue>
#include <chrono>
#include <memory>
#include <map>
#include <condition_variable>
#include <zlib.h>  // For CRC32 and compression
#include <endian.h>  // For htobe64, etc.
#include <thread>

#include "../config.h"
#include "../logger/Logger.h"

/**
 * @namespace ServerComm
 * @brief Bao bọc tất cả các lớp và chức năng liên quan đến giao tiếp với server.
 */
namespace ServerComm {

/**
 * @struct Point2D
 * @brief Đại diện cho một điểm trong không gian 2D.
 * Thường được sử dụng để biểu diễn dữ liệu từ cảm biến LiDAR.
 */
struct Point2D {
    float x; ///< Tọa độ X của điểm.
    float y; ///< Tọa độ Y của điểm.
};

/**
 * @struct AGVStatusPacket
 * @brief Tập hợp tất cả dữ liệu trạng thái của AGV để gửi đến server trong một gói tin.
 */
struct AGVStatusPacket {
    int id_agv;                             ///< ID định danh của AGV.
    float battery_level;                    ///< Mức pin hiện tại (thường là phần trăm).
    bool is_moving;                         ///< Trạng thái cho biết AGV có đang di chuyển hay không.
    bool is_safe;                           ///< Trạng thái an toàn (ví dụ: không có vật cản).
    bool plc_connected;                     ///< Trạng thái kết nối với PLC.
    bool lidar_connected;                   ///< Trạng thái kết nối với LiDAR.
    bool battery_connected;                 ///< Trạng thái kết nối với hệ thống quản lý pin (BMS).
    float current_speed;                    ///< Tốc độ di chuyển hiện tại của AGV.
    //bool imu_connected = false;
    float current_heading;      // Hướng hiện tại (0-360°)
    //float current_roll;         // Góc nghiêng trái/phải
    //float current_pitch;        // Góc nghiêng lên/xuống
    //bool imu_tilt_warning;     // Cảnh báo nghiêng nguy hiểm
    //std::string last_imu_data = "Chưa có dữ liệu IMU";
    std::map<std::string, uint16_t> plc_registers; ///< Dữ liệu các thanh ghi đọc từ PLC.
    // std::vector<Point2D> stable_lidar_points;    // Dữ liệu chất lượng cao cho Mapping
    std::vector<Point2D> realtime_lidar_points;  // Dữ liệu liên tục cho Visualization
    long timestamp;                         ///< Dấu thời gian khi gói tin được tạo.
};

/**
 * @struct NavigationCommand
 * @brief Đại diện cho một lệnh điều hướng nhận được từ server.
 */
struct NavigationCommand {
    /**
     * @enum Type
     * @brief Liệt kê các loại lệnh điều hướng có thể có.
     */
    enum Type { 
        STOP = 0,               ///< Dừng AGV ngay lập tức.
        MOVE_TO_POINT,      ///< Di chuyển tiến.
        //REVERSE_TO_POINT,   ///< Di chuyển lùi.
        ROTATE_TO_LEFT,     ///< Quay trái.
        ROTATE_TO_RIGHT,    ///< Quay phải.
        //ROTATE_180_LEFT,     ///< Quay trái 90 độ.
        //ROTATE_180_RIGHT,    ///< Quay phải 90 độ.
        EMERGENCY_STOP      ///< Lệnh dừng khẩn cấp, có độ ưu tiên cao nhất.
    };
    Type type;            ///< Loại lệnh điều hướng.
    float target_x;       ///< Tọa độ X của điểm đến.
    float target_y;       ///< Tọa độ Y của điểm đến.
    float target_angle;   ///< Góc xoay mục tiêu (tính bằng độ hoặc radian).
    float speed;          ///< Tốc độ thực hiện lệnh.
};

/**
 * @namespace BinaryProtocol
 * @brief Chứa các hàm và cấu trúc để xử lý giao thức nhị phân tự định nghĩa.
 * Giao thức này được thiết kế để hiệu quả hơn JSON về mặt băng thông và tốc độ xử lý.
 */
namespace BinaryProtocol {
    
    /**
     * @enum PacketType
     * @brief Định nghĩa các loại gói tin trong giao thức nhị phân.
     */
    enum PacketType : uint8_t {
        REALTIME_STATUS   = 0x01, ///< Gói tin chứa trạng thái thời gian thực của AGV.
        NAVIGATION_COMMAND= 0x02, ///< Gói tin chứa lệnh điều hướng từ server.
        HEARTBEAT         = 0x05, ///< Gói tin heartbeat để duy trì và kiểm tra kết nối.
        HEARTBEAT_ACK     = 0x06,  ///< Gói tin xác nhận heartbeat từ server.
        PING              = 0x07, ///< Gói tin PING để đo độ trễ.
        PONG              = 0x08  ///< Gói tin PONG trả lời cho PING.
    };
    
    /**
     * @struct PacketHeader
     * @brief Cấu trúc của phần header trong mỗi gói tin nhị phân.
     * Header có kích thước cố định và chứa thông tin metadata về gói tin.
     */
    struct PacketHeader {
        uint16_t magic;     ///< Số "magic" (ví dụ: 0xAA55) để xác thực gói tin.
        uint8_t type;       ///< Loại gói tin, xem @ref PacketType.
        uint8_t flags;      ///< Các cờ (ví dụ: bit 0 cho biết payload có được nén hay không).
        uint32_t length;    ///< Độ dài của payload (phần dữ liệu) theo sau header.
        uint32_t checksum;  ///< Checksum (ví dụ: CRC32) của payload để kiểm tra tính toàn vẹn.
    };
    
    uint32_t calculateCRC32(const uint8_t* data, size_t length);
    
    std::string buildRealtimePacket(
        int id_agv,
        float battery_level,
        bool is_moving,
        bool is_safe,
        bool plc_connected,
        bool lidar_connected,
        bool battery_connected,
        float current_speed,
        float current_heading,
        const std::map<std::string, uint16_t>& plc_registers,
        //const std::vector<Point2D>& stable_lidar_points,    // Dữ liệu ổn định cho Mapping
        const std::vector<Point2D>& realtime_lidar_points,  // Dữ liệu realtime cho Visualization
        long timestamp
    );
    
    std::string buildNavigationCommand(
        uint8_t command_type,
        float target_x,
        float target_y,
        float target_angle,
        float speed
    );
    
    std::string buildHeartbeatPacket(int agv_id);
    
    std::string buildPacket(uint8_t type, uint8_t flags, const std::vector<uint8_t>& payload);

    bool parsePacketHeader(const uint8_t* data, PacketHeader& header);

    bool parseNavigationCommand(const std::string& data, 
                               uint8_t& command_type,
                               float& target_x,
                               float& target_y,
                               float& target_angle,
                               float& speed);
    
    bool parseHeartbeatAck(const std::string& data, int& server_timestamp);
}

/**
 * @class CommunicationServer
 * @brief Quản lý toàn bộ việc giao tiếp qua TCP với server trung tâm.
 * Lớp này xử lý việc kết nối, gửi/nhận dữ liệu, tự động kết nối lại,
 * và quản lý các luồng riêng biệt cho việc gửi và nhận.
 */
class CommunicationServer {
public:
    using StatusCallback = std::function<AGVStatusPacket()>;
    using CommandCallback = std::function<void(const NavigationCommand&)>;
    using ConnectionCallback = std::function<void(bool)>;

public:
    // --- Network configuration ---
    CommunicationServer(const std::string& server_ip,
                       int server_port,
                       int send_interval_ms = COMM_SEND_INTERVAL_MS);
    ~CommunicationServer();
    
    // --- Connection management ---
    bool connect();
    void disconnect();
    bool isConnected() const { return is_connected.load(); }
    
    // --- Start/Stop ---
    bool start();
    void stop();
    
    // --- Data Sending ---
    void sendStatus(const AGVStatusPacket& status);
    void sendEmergencyStop();
    void sendHeartbeat();
    void sendPing();

    // --- Callbacks Setup ---
    void setStatusCallback(StatusCallback callback);
    void setCommandCallback(CommandCallback callback);
    void setConnectionCallback(ConnectionCallback callback);
    
    // --- Command Queue ---
    bool getNextCommand(NavigationCommand& cmd);
    size_t getCommandQueueSize() const;
    void clearCommandQueue();
    
    // --- Statistics ---
    long getTotalPacketsSent() const { return total_packets_sent.load(); }
    long getTotalPacketsReceived() const { return total_packets_received.load(); }
    long getTotalBytesSent() const { return total_bytes_sent.load(); }
    long getTotalBytesReceived() const { return total_bytes_received.load(); }
    float getDataRate() const;
    long getLastCommunicationTime() const;
    
    // --- Configuration ---
    void setSendInterval(int ms) { send_interval_ms = ms; }
    void setReconnectInterval(int ms) { reconnect_interval_ms = ms; }
    void setHeartbeatInterval(int ms) { heartbeat_interval_ms = ms; }
    void setHeartbeatTimeout(int ms) { heartbeat_timeout_ms = ms; }
    
private:
    // --- Thread functions ---
    void receiveThread();
    void sendThread();
    void reconnectThread();
    
    // --- Socket operations ---
    bool createSocket();
    bool connectToServer();
    void closeSocket();
    bool sendData(const std::string& data);
    std::string receiveData();
    
    // --- Data processing ---
    void processReceivedData(const std::string& data);
    void handleCommand(const NavigationCommand& cmd);
    void handleHeartbeatAck(int server_timestamp);

    // --- Utilities ---
    long getCurrentTimestamp() const;
    void updateStatistics(long bytes_sent, long bytes_received);
    bool isSocketValid() const;
    void setSocketTimeout(int timeout_ms);
    bool checkHeartbeatTimeout();

    std::string server_ip;
    int server_port;
    int socket_fd;
    std::atomic<bool> is_connected;
    std::atomic<bool> is_running;
    std::unique_ptr<std::thread> receive_thread;
    std::unique_ptr<std::thread> send_thread;
    std::unique_ptr<std::thread> reconnect_thread;
    std::queue<std::string> send_queue;
    std::queue<NavigationCommand> command_queue;
    mutable std::mutex send_mutex;
    mutable std::mutex command_mutex;
    mutable std::mutex connection_mutex;
    std::condition_variable send_cv;
    std::condition_variable command_cv;
    StatusCallback status_callback;
    CommandCallback command_callback;
    ConnectionCallback connection_callback;
    std::atomic<long> total_packets_sent;
    std::atomic<long> total_packets_received;
    std::atomic<long> total_bytes_sent;
    std::atomic<long> total_bytes_received;
    std::chrono::steady_clock::time_point last_send_time;
    std::chrono::steady_clock::time_point last_receive_time;
    std::chrono::steady_clock::time_point last_heartbeat_ack_time;
    int send_interval_ms;
    int reconnect_interval_ms;
    int heartbeat_interval_ms;
    int socket_timeout_ms;
    int heartbeat_timeout_ms;
};

} // namespace ServerComm

#endif // SERVER_COMMUNICATOR_H