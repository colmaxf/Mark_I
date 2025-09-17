#include "servercommunicator.h"
#include <sstream>
#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <iomanip>

// Include Point2D definition from main
struct Point2D {
    float x, y;
};

namespace ServerComm {

// MessageProtocol static members
const std::string MessageProtocol::HEADER_START = "$$START$$";
const std::string MessageProtocol::HEADER_END = "$$END$$";
const size_t MessageProtocol::MAX_MESSAGE_SIZE = 1024 * 1024;

// JsonBuilder implementation - Simple JSON string builder
std::string JsonBuilder::pointToJson(float x, float y) {
    std::stringstream ss;
    ss << "{\"x\":" << x << ",\"y\":" << y << "}";
    return ss.str();
}

std::string JsonBuilder::pointsToJson(const std::vector<Point2D>& points) {
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < points.size(); ++i) {
        ss << pointToJson(points[i].x, points[i].y);
        if (i < points.size() - 1) ss << ",";
    }
    ss << "]";
    return ss.str();
}

std::string JsonBuilder::mapToJson(const std::map<std::string, uint16_t>& data) {
    std::stringstream ss;
    ss << "{";
    auto it = data.begin();
    for (size_t i = 0; i < data.size(); ++i, ++it) {
        ss << "\"" << it->first << "\":" << it->second;
        if (i < data.size() - 1) ss << ",";
    }
    ss << "}";
    return ss.str();
}

bool JsonBuilder::parseCommand(const std::string& json_str, std::map<std::string, std::string>& result) {
    // Simple JSON parser for command format: {"type":"value", "param":"value"}
    result.clear();
    
    size_t pos = 0;
    while ((pos = json_str.find("\"", pos)) != std::string::npos) {
        size_t key_start = pos + 1;
        size_t key_end = json_str.find("\"", key_start);
        if (key_end == std::string::npos) break;
        
        std::string key = json_str.substr(key_start, key_end - key_start);
        
        size_t colon_pos = json_str.find(":", key_end);
        if (colon_pos == std::string::npos) break;
        
        size_t value_start = json_str.find_first_not_of(" ", colon_pos + 1);
        size_t value_end;
        
        if (json_str[value_start] == '\"') {
            value_start++;
            value_end = json_str.find("\"", value_start);
        } else {
            value_end = json_str.find_first_of(",}", value_start);
        }
        
        if (value_end != std::string::npos) {
            result[key] = json_str.substr(value_start, value_end - value_start);
        }
        
        pos = value_end + 1;
    }
    
    return !result.empty();
}

// NavigationCommand implementation
std::string NavigationCommand::toString() const {
    std::stringstream ss;
    ss << "{\"type\":" << type 
       << ",\"target_x\":" << target_x
       << ",\"target_y\":" << target_y
       << ",\"target_angle\":" << target_angle
       << ",\"speed\":" << speed << "}";
    return ss.str();
}

NavigationCommand NavigationCommand::fromString(const std::string& data) {
    NavigationCommand cmd;
    std::map<std::string, std::string> parsed;
    
    if (JsonBuilder::parseCommand(data, parsed)) {
        if (parsed.count("type")) cmd.type = static_cast<Type>(std::stoi(parsed["type"]));
        if (parsed.count("target_x")) cmd.target_x = std::stof(parsed["target_x"]);
        if (parsed.count("target_y")) cmd.target_y = std::stof(parsed["target_y"]);
        if (parsed.count("target_angle")) cmd.target_angle = std::stof(parsed["target_angle"]);
        if (parsed.count("speed")) cmd.speed = std::stof(parsed["speed"]);
    }
    
    return cmd;
}

// AGVStatusPacket implementation
std::string AGVStatusPacket::toProtocolString() const {
    std::stringstream ss;
    ss << "{"
       << "\"position\":{\"x\":" << current_x 
       << ",\"y\":" << current_y
       << ",\"angle\":" << current_angle << "},"
       << "\"plc_registers\":" << JsonBuilder::mapToJson(plc_registers) << ","
       << "\"lidar_points\":" << JsonBuilder::pointsToJson(lidar_points) << ","
       << "\"status\":{"
       << "\"is_moving\":" << (is_moving ? "true" : "false")
       << ",\"is_safe\":" << (is_safe ? "true" : "false")
       << ",\"battery_level\":" << battery_level
       << ",\"current_speed\":" << current_speed
       << ",\"timestamp\":" << timestamp
       << "}}";
    return ss.str();
}

// MessageProtocol implementation
std::string MessageProtocol::createMessage(MessageType type, const std::string& payload) {
    std::stringstream ss;
    ss << "{\"type\":" << type 
       << ",\"payload\":" << payload
       << ",\"timestamp\":" << std::chrono::system_clock::now().time_since_epoch().count()
       << "}";
    return addHeader(ss.str());
}

bool MessageProtocol::parseMessage(const std::string& raw_data, MessageType& type, std::string& payload) {
    // Find type field
    size_t type_pos = raw_data.find("\"type\":");
    if (type_pos == std::string::npos) return false;
    
    size_t type_start = type_pos + 7;
    size_t type_end = raw_data.find_first_of(",}", type_start);
    if (type_end == std::string::npos) return false;
    
    type = static_cast<MessageType>(std::stoi(raw_data.substr(type_start, type_end - type_start)));
    
    // Find payload field
    size_t payload_pos = raw_data.find("\"payload\":");
    if (payload_pos != std::string::npos) {
        size_t payload_start = payload_pos + 10;
        size_t payload_end = raw_data.find(",\"timestamp\"", payload_start);
        if (payload_end == std::string::npos) {
            payload_end = raw_data.find("}", payload_start);
        }
        payload = raw_data.substr(payload_start, payload_end - payload_start);
    }
    
    return true;
}

std::string MessageProtocol::addHeader(const std::string& data) {
    std::stringstream ss;
    ss << HEADER_START << data.length() << "|" << data << HEADER_END;
    return ss.str();
}

bool MessageProtocol::extractMessage(std::string& buffer, std::string& message) {
    size_t start_pos = buffer.find(HEADER_START);
    if (start_pos == std::string::npos) return false;
    
    size_t header_end = buffer.find("|", start_pos);
    if (header_end == std::string::npos) return false;
    
    std::string size_str = buffer.substr(start_pos + HEADER_START.length(),
                                         header_end - start_pos - HEADER_START.length());
    size_t message_size = std::stoul(size_str);
    
    if (message_size > MAX_MESSAGE_SIZE) {
        LOG_ERROR << "[MessageProtocol] Message too large: " << message_size;
        buffer.clear();
        return false;
    }
    
    size_t end_pos = buffer.find(HEADER_END, header_end);
    if (end_pos == std::string::npos) return false;
    
    message = buffer.substr(header_end + 1, message_size);
    buffer.erase(0, end_pos + HEADER_END.length());
    
    return true;
}

// CommunicationServer implementation
CommunicationServer::CommunicationServer(const std::string& server_ip,
                                       int server_port,
                                       int send_interval_ms)
    : server_ip(server_ip), server_port(server_port),
      socket_fd(-1), is_connected(false), is_running(false),
      send_interval_ms(send_interval_ms),
      reconnect_interval_ms(5000),
      heartbeat_interval_ms(1000),
      socket_timeout_ms(5000),
      total_packets_sent(0), total_packets_received(0),
      total_bytes_sent(0), total_bytes_received(0) {
    
    LOG_INFO << "[CommServer] Initialized for " << server_ip << ":" << server_port;
}

CommunicationServer::~CommunicationServer() {
    stop();
}

bool CommunicationServer::connect() {
    if (is_connected) {
        LOG_WARNING << "[CommServer] Already connected";
        return true;
    }
    
    if (!createSocket()) {
        return false;
    }
    
    if (!connectToServer()) {
        closeSocket();
        return false;
    }
    
    is_connected = true;
    
    if (connection_callback) {
        connection_callback(true);
    }
    
    LOG_INFO << "[CommServer] Connected successfully";
    return true;
}

void CommunicationServer::disconnect() {
    if (!is_connected) return;
    
    is_connected = false;
    closeSocket();
    
    if (connection_callback) {
        connection_callback(false);
    }
    
    LOG_INFO << "[CommServer] Disconnected";
}

bool CommunicationServer::start() {
    if (is_running) {
        LOG_WARNING << "[CommServer] Already running";
        return true;
    }
    
    if (!is_connected) {
        if (!connect()) {
            LOG_ERROR << "[CommServer] Failed to connect";
            return false;
        }
    }
    
    is_running = true;
    
    receive_thread = std::make_unique<std::thread>(&CommunicationServer::receiveThread, this);
    send_thread = std::make_unique<std::thread>(&CommunicationServer::sendThread, this);
    reconnect_thread = std::make_unique<std::thread>(&CommunicationServer::reconnectThread, this);
    
    LOG_INFO << "[CommServer] Started";
    return true;
}

void CommunicationServer::stop() {
    if (!is_running) return;
    
    LOG_INFO << "[CommServer] Stopping...";
    
    is_running = false;
    send_cv.notify_all();
    command_cv.notify_all();
    
    if (receive_thread && receive_thread->joinable()) {
        receive_thread->join();
    }
    if (send_thread && send_thread->joinable()) {
        send_thread->join();
    }
    if (reconnect_thread && reconnect_thread->joinable()) {
        reconnect_thread->join();
    }
    
    disconnect();
    
    LOG_INFO << "[CommServer] Stopped";
}

void CommunicationServer::sendStatus(const AGVStatusPacket& status) {
    std::string message = MessageProtocol::createMessage(
        MessageProtocol::STATUS_UPDATE, 
        status.toProtocolString()
    );
    
    std::lock_guard<std::mutex> lock(send_mutex);
    send_queue.push(message);
    send_cv.notify_one();
}

void CommunicationServer::sendLidarPoints(const std::vector<Point2D>& points) {
    std::stringstream ss;
    ss << "{\"points\":" << JsonBuilder::pointsToJson(points)
       << ",\"count\":" << points.size()
       << ",\"timestamp\":" << getCurrentTimestamp() << "}";
    
    std::string message = MessageProtocol::createMessage(
        MessageProtocol::LIDAR_DATA, ss.str()
    );
    
    std::lock_guard<std::mutex> lock(send_mutex);
    send_queue.push(message);
    send_cv.notify_one();
}

void CommunicationServer::sendPLCRegisters(const std::map<std::string, uint16_t>& registers) {
    std::stringstream ss;
    ss << "{\"registers\":" << JsonBuilder::mapToJson(registers)
       << ",\"timestamp\":" << getCurrentTimestamp() << "}";
    
    std::string message = MessageProtocol::createMessage(
        MessageProtocol::PLC_DATA, ss.str()
    );
    
    std::lock_guard<std::mutex> lock(send_mutex);
    send_queue.push(message);
    send_cv.notify_one();
}

void CommunicationServer::sendEmergencyStop() {
    std::stringstream ss;
    ss << "{\"reason\":\"Emergency stop triggered\""
       << ",\"timestamp\":" << getCurrentTimestamp() << "}";
    
    std::string message = MessageProtocol::createMessage(
        MessageProtocol::EMERGENCY, ss.str()
    );
    
    if (sendData(message)) {
        LOG_WARNING << "[CommServer] Emergency stop sent";
    }
}

void CommunicationServer::setStatusCallback(StatusCallback callback) {
    status_callback = callback;
}

void CommunicationServer::setCommandCallback(CommandCallback callback) {
    command_callback = callback;
}

void CommunicationServer::setConnectionCallback(ConnectionCallback callback) {
    connection_callback = callback;
}

bool CommunicationServer::getNextCommand(NavigationCommand& cmd) {
    std::lock_guard<std::mutex> lock(command_mutex);
    if (command_queue.empty()) return false;
    
    cmd = command_queue.front();
    command_queue.pop();
    return true;
}

size_t CommunicationServer::getCommandQueueSize() const {
    std::lock_guard<std::mutex> lock(command_mutex);
    return command_queue.size();
}

void CommunicationServer::clearCommandQueue() {
    std::lock_guard<std::mutex> lock(command_mutex);
    std::queue<NavigationCommand> empty;
    std::swap(command_queue, empty);
}

float CommunicationServer::getDataRate() const {
    auto now = std::chrono::steady_clock::now();
    float duration = std::chrono::duration<float>(now - last_send_time).count();
    if (duration > 0) {
        return (total_bytes_sent + total_bytes_received) / duration / 1024.0f;
    }
    return 0.0f;
}

long CommunicationServer::getLastCommunicationTime() const {
    auto latest = std::max(last_send_time, last_receive_time);
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        latest.time_since_epoch()).count();
}

// Thread functions
void CommunicationServer::receiveThread() {
    LOG_INFO << "[CommServer] Receive thread started";
    
    std::string buffer;
    
    while (is_running) {
        if (!is_connected) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        std::string data = receiveData();
        if (!data.empty()) {
            buffer += data;
            total_bytes_received += data.length();
            
            std::string message;
            while (MessageProtocol::extractMessage(buffer, message)) {
                processReceivedData(message);
                total_packets_received++;
            }
            
            last_receive_time = std::chrono::steady_clock::now();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    LOG_INFO << "[CommServer] Receive thread stopped";
}

void CommunicationServer::sendThread() {
    LOG_INFO << "[CommServer] Send thread started";
    
    auto last_status_time = std::chrono::steady_clock::now();
    auto last_heartbeat_time = std::chrono::steady_clock::now();
    
    while (is_running) {
        if (!is_connected) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        auto now = std::chrono::steady_clock::now();
        
        // Send periodic status
        if (status_callback && 
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_status_time).count() >= send_interval_ms) {
            
            AGVStatusPacket status = status_callback();
            sendStatus(status);
            last_status_time = now;
        }
        
        // Send heartbeat
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_heartbeat_time).count() >= heartbeat_interval_ms) {
            
            std::string heartbeat = createHeartbeatPacket();
            std::string message = MessageProtocol::createMessage(
                MessageProtocol::HEARTBEAT, heartbeat);
            sendData(message);
            last_heartbeat_time = now;
        }
        
        // Process send queue
        {
            std::unique_lock<std::mutex> lock(send_mutex);
            if (send_cv.wait_for(lock, std::chrono::milliseconds(10),
                                [this] { return !send_queue.empty() || !is_running; })) {
                
                while (!send_queue.empty() && is_connected) {
                    std::string msg = send_queue.front();
                    send_queue.pop();
                    lock.unlock();
                    
                    if (sendData(msg)) {
                        total_packets_sent++;
                        total_bytes_sent += msg.length();
                    }
                    
                    lock.lock();
                }
            }
        }
        
        last_send_time = now;
    }
    
    LOG_INFO << "[CommServer] Send thread stopped";
}

void CommunicationServer::reconnectThread() {
    LOG_INFO << "[CommServer] Reconnect thread started";
    
    while (is_running) {
        if (!is_connected) {
            LOG_INFO << "[CommServer] Attempting reconnection...";
            
            if (connect()) {
                LOG_INFO << "[CommServer] Reconnected successfully";
            } else {
                LOG_WARNING << "[CommServer] Reconnection failed";
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms));
    }
    
    LOG_INFO << "[CommServer] Reconnect thread stopped";
}

// Socket operations
bool CommunicationServer::createSocket() {
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        LOG_ERROR << "[CommServer] Failed to create socket";
        return false;
    }
    
    int opt = 1;
    setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // Set non-blocking
    int flags = fcntl(socket_fd, F_GETFL, 0);
    fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);
    
    setSocketTimeout(socket_timeout_ms);
    
    return true;
}

bool CommunicationServer::connectToServer() {
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);
    
    if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0) {
        LOG_ERROR << "[CommServer] Invalid IP: " << server_ip;
        return false;
    }
    
    if (::connect(socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        if (errno != EINPROGRESS) {
            LOG_ERROR << "[CommServer] Connect failed: " << strerror(errno);
            return false;
        }
        
        fd_set write_set;
        FD_ZERO(&write_set);
        FD_SET(socket_fd, &write_set);
        
        struct timeval timeout;
        timeout.tv_sec = socket_timeout_ms / 1000;
        timeout.tv_usec = (socket_timeout_ms % 1000) * 1000;
        
        int result = select(socket_fd + 1, nullptr, &write_set, nullptr, &timeout);
        if (result <= 0) {
            LOG_ERROR << "[CommServer] Connection timeout";
            return false;
        }

        // After select indicates writability, we need to check SO_ERROR
        // to see if the connection was actually successful.
        int so_error = 0;
        socklen_t len = sizeof(so_error);
        getsockopt(socket_fd, SOL_SOCKET, SO_ERROR, &so_error, &len);
        if (so_error != 0) {
            LOG_ERROR << "[CommServer] Connect failed with error: " << strerror(so_error);
            return false;
        }
    }
    
    LOG_INFO << "[CommServer] Connected to " << server_ip << ":" << server_port;
    return true;
}

void CommunicationServer::closeSocket() {
    if (socket_fd >= 0) {
        close(socket_fd);
        socket_fd = -1;
    }
}

bool CommunicationServer::sendData(const std::string& data) {
    if (!is_connected || socket_fd < 0) return false;
    
    ssize_t bytes_sent = send(socket_fd, data.c_str(), data.length(), MSG_NOSIGNAL);
    
    if (bytes_sent < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            LOG_ERROR << "[CommServer] Send failed: " << strerror(errno);
            is_connected = false;
            return false;
        }
    }
    
    return bytes_sent > 0;
}

std::string CommunicationServer::receiveData() {
    if (!is_connected || socket_fd < 0) return "";
    
    char buffer[4096];
    ssize_t bytes_received = recv(socket_fd, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
    
    if (bytes_received > 0) {
        buffer[bytes_received] = '\0';
        return std::string(buffer);
    } else if (bytes_received == 0) {
        LOG_WARNING << "[CommServer] Connection closed by server";
        is_connected = false;
    } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
        LOG_ERROR << "[CommServer] Receive failed: " << strerror(errno);
        is_connected = false;
    }
    
    return "";
}

// Data processing
std::string CommunicationServer::createStatusPacket() {
    if (status_callback) {
        AGVStatusPacket status = status_callback();
        return status.toProtocolString();
    }
    return "{}";
}

std::string CommunicationServer::createHeartbeatPacket() {
    std::stringstream ss;
    ss << "{\"timestamp\":" << getCurrentTimestamp()
       << ",\"packets_sent\":" << total_packets_sent.load()
       << ",\"packets_received\":" << total_packets_received.load()
       << "}";
    return ss.str();
}

void CommunicationServer::processReceivedData(const std::string& data) {
    MessageProtocol::MessageType type;
    std::string payload;
    
    if (MessageProtocol::parseMessage(data, type, payload)) {
        LOG_INFO << "[CommServer] Received type: " << type;
        
        switch (type) {
            case MessageProtocol::NAVIGATION_CMD:
                handleCommand(payload);
                break;
                
            case MessageProtocol::ACK:
                LOG_DEBUG << "[CommServer] ACK received";
                break;
                
            default:
                LOG_WARNING << "[CommServer] Unknown type: " << type;
                break;
        }
    }
}

void CommunicationServer::handleCommand(const std::string& cmd_str) {
    NavigationCommand cmd = NavigationCommand::fromString(cmd_str);
    
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        command_queue.push(cmd);
    }
    command_cv.notify_one();
    
    if (command_callback) {
        command_callback(cmd);
    }
    
    LOG_INFO << "[CommServer] Command received: Type=" << cmd.type;
}

// Utilities
long CommunicationServer::getCurrentTimestamp() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void CommunicationServer::updateStatistics(long bytes_sent, long bytes_received) {
    total_bytes_sent += bytes_sent;
    total_bytes_received += bytes_received;
}

bool CommunicationServer::isSocketValid() const {
    return socket_fd >= 0 && is_connected;
}

void CommunicationServer::setSocketTimeout(int timeout_ms) {
    if (socket_fd < 0) return;
    
    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
}

} // namespace ServerComm