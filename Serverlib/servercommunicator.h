#ifndef SERVER_COMMUNICATOR_H
#define SERVER_COMMUNICATOR_H

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <queue>
#include <chrono>
#include <memory>
#include <map>
#include <condition_variable>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "../config.h"
#include "../logger/Logger.h"

// Forward declaration - Point2D sẽ được định nghĩa trong main.cpp
struct Point2D;

// Namespace để tránh xung đột tên
namespace ServerComm {

// JSON helper functions - tránh phụ thuộc vào external library
class JsonBuilder {
public:
    static std::string pointToJson(float x, float y);
    static std::string pointsToJson(const std::vector<Point2D>& points);
    static std::string mapToJson(const std::map<std::string, uint16_t>& data);
    static bool parseCommand(const std::string& json_str, std::map<std::string, std::string>& result);
};

/**
 * @struct NavigationCommand
 * @brief Cấu trúc dữ liệu cho một lệnh điều hướng nhận được từ server.
 * @details Bao gồm các loại lệnh khác nhau như di chuyển đến điểm, xoay, hoặc dừng.
 */
struct NavigationCommand {
    /**
     * @enum Type
     * @brief Liệt kê các loại lệnh điều hướng có thể có.
     */
    enum Type {
        MOVE_TO_POINT = 0,   ///< Di chuyển đến một điểm (x, y) cụ thể.
        ROTATE_TO_ANGLE = 1, ///< Xoay đến một góc cụ thể.
        FOLLOW_PATH = 2,     ///< Đi theo một danh sách các điểm (path).
        STOP = 3,            ///< Dừng di chuyển một cách bình thường.
        EMERGENCY_STOP = 4   ///< Dừng khẩn cấp ngay lập tức.
    };
    
    Type type; ///< Loại lệnh điều hướng.
    float target_x, target_y; ///< Tọa độ x, y của điểm đến.
    float target_angle; ///< Góc mục tiêu cần xoay đến (radian).
    std::vector<Point2D> path; ///< Danh sách các điểm tạo thành đường đi.
    float speed; ///< Tốc độ di chuyển mong muốn (m/s).
    
    std::string toString() const;
    static NavigationCommand fromString(const std::string& data);
};

// Cấu trúc dữ liệu gửi lên server
struct AGVStatusPacket {
    float current_x, current_y;
    float current_angle;
    std::map<std::string, uint16_t> plc_registers;
    std::vector<Point2D> lidar_points;
    bool is_moving;
    bool is_safe;
    float battery_level;
    float current_speed;
    // Connection statuses
    bool plc_connected;
    bool lidar_connected;
    bool battery_connected;
    bool server_connected;
    long timestamp;
    
    std::string toProtocolString() const;
};

// Message Protocol
class MessageProtocol {
public:
    enum MessageType {
        STATUS_UPDATE = 1,
        LIDAR_DATA = 2,
        PLC_DATA = 3,
        NAVIGATION_CMD = 4,
        HEARTBEAT = 5,
        EMERGENCY = 6,
        ACK = 7
    };
    
    static std::string createMessage(MessageType type, const std::string& payload);
    static bool parseMessage(const std::string& raw_data, MessageType& type, std::string& payload);
    static std::string addHeader(const std::string& data);
    static bool extractMessage(std::string& buffer, std::string& message);
    
private:
    static const std::string HEADER_START;
    static const std::string HEADER_END;
    static const size_t MAX_MESSAGE_SIZE;
};

// Main Communication Server Class
class CommunicationServer {
public:
    using StatusCallback = std::function<AGVStatusPacket()>;
    using CommandCallback = std::function<void(const NavigationCommand&)>;
    using ConnectionCallback = std::function<void(bool)>;

private:
    // Network configuration
    std::string server_ip;
    int server_port;
    int socket_fd;
    std::atomic<bool> is_connected;
    
    // Threading
    std::atomic<bool> is_running;
    std::unique_ptr<std::thread> receive_thread;
    std::unique_ptr<std::thread> send_thread;
    std::unique_ptr<std::thread> reconnect_thread;
    
    // Data queues - sử dụng string serialization thay vì JSON object
    std::queue<std::string> send_queue;
    std::queue<NavigationCommand> command_queue;
    mutable std::mutex send_mutex;
    mutable std::mutex command_mutex;
    std::condition_variable send_cv;
    std::condition_variable command_cv;
    
    // Callbacks
    StatusCallback status_callback;
    CommandCallback command_callback;
    ConnectionCallback connection_callback;
    
    // Statistics
    std::atomic<long> total_packets_sent;
    std::atomic<long> total_packets_received;
    std::atomic<long> total_bytes_sent;
    std::atomic<long> total_bytes_received;
    std::chrono::steady_clock::time_point last_send_time;
    std::chrono::steady_clock::time_point last_receive_time;
    
    // Configuration
    int send_interval_ms;
    int reconnect_interval_ms;
    int heartbeat_interval_ms;
    int socket_timeout_ms;
    
public:
    CommunicationServer(const std::string& server_ip,
                       int server_port,
                       int send_interval_ms = 200);
    ~CommunicationServer();
    
    // Connection management
    bool connect();
    void disconnect();
    bool isConnected() const { return is_connected.load(); }
    
    // Start/Stop
    bool start();
    void stop();
    
    // Send operations
    void sendStatus(const AGVStatusPacket& status);
    void sendLidarPoints(const std::vector<Point2D>& points);
    void sendPLCRegisters(const std::map<std::string, uint16_t>& registers);
    void sendEmergencyStop();
    
    // Callbacks
    void setStatusCallback(StatusCallback callback);
    void setCommandCallback(CommandCallback callback); 
    void setConnectionCallback(ConnectionCallback callback);
    
    // Command queue
    bool getNextCommand(NavigationCommand& cmd);
    size_t getCommandQueueSize() const;
    void clearCommandQueue();
    
    // Statistics
    long getTotalPacketsSent() const { return total_packets_sent.load(); }
    long getTotalPacketsReceived() const { return total_packets_received.load(); }
    long getTotalBytesSent() const { return total_bytes_sent.load(); }
    long getTotalBytesReceived() const { return total_bytes_received.load(); }
    float getDataRate() const;
    long getLastCommunicationTime() const;
    
    // Configuration
    void setSendInterval(int ms) { send_interval_ms = ms; }
    void setReconnectInterval(int ms) { reconnect_interval_ms = ms; }
    void setHeartbeatInterval(int ms) { heartbeat_interval_ms = ms; }
    
private:
    // Thread functions
    void receiveThread();
    void sendThread();
    void reconnectThread();
    
    // Socket operations
    bool createSocket();
    bool connectToServer();
    void closeSocket();
    bool sendData(const std::string& data);
    std::string receiveData();
    
    // Data processing
    std::string createStatusPacket();
    std::string createHeartbeatPacket();
    void processReceivedData(const std::string& data);
    void handleCommand(const std::string& cmd_str);
    
    // Utilities
    long getCurrentTimestamp() const;
    void updateStatistics(long bytes_sent, long bytes_received);
    bool isSocketValid() const;
    void setSocketTimeout(int timeout_ms);
};

} // namespace ServerComm

#endif // SERVER_COMMUNICATOR_H