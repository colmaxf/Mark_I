// servercommunicator.cpp
#include "servercommunicator.h"
#include <sstream>
#include <iostream>
#include <cstring> // Dùng cho memcpy
#include <fcntl.h> // Dùng cho fcntl để điều khiển socket
#include <iomanip>
#include <zlib.h> // Thư viện nén zlib
#include <algorithm> // Dùng cho std::max
#include <ctime> // Dùng cho std::time

#include <arpa/inet.h> // Thêm header cho inet_pton
// Point2D is now included from the header

namespace ServerComm {

/**
 * @namespace BinaryProtocol
 * @brief Chứa các hàm và hằng số để xây dựng và phân tích các gói tin nhị phân.
 * Giao thức này được thiết kế để tối ưu hóa băng thông và hiệu suất so với JSON.
 */
namespace BinaryProtocol {

// Hằng số định danh cho gói tin, dùng để xác thực.
static constexpr uint16_t MAGIC_NUMBER = 0xAA55;
// Cờ báo hiệu payload của gói tin được nén.
static constexpr uint8_t FLAG_COMPRESSED = 0x01;

/**
 * @brief Chuyển đổi một số float sang định dạng 32-bit integer theo thứ tự byte mạng.
 * @details Sao chép bit-pattern của float vào một uint32_t, sau đó dùng htonl.
 * @param f Số float cần chuyển đổi.
 */
uint32_t float_to_net(float f) {
    uint32_t i;
    memcpy(&i, &f, sizeof(float));
    return htonl(i);
}

/**
 * @brief Tính toán checksum CRC32 cho một khối dữ liệu.
 * @param data Con trỏ tới dữ liệu cần tính checksum.
 * @param length Độ dài của dữ liệu (tính bằng byte).
 * @return Giá trị checksum CRC32.
 */
uint32_t calculateCRC32(const uint8_t* data, size_t length) {
    return crc32(0, data, length);
}

/**
 * @brief Xây dựng một gói tin nhị phân chung từ các thành phần.
 * @param type Loại gói tin (xem PacketType).
 * @param flags Các cờ (ví dụ: FLAG_COMPRESSED).
 * @param payload Dữ liệu của gói tin.
 * @return Chuỗi string chứa gói tin nhị phân hoàn chỉnh.
 */
std::string buildPacket(uint8_t type, uint8_t flags, const std::vector<uint8_t>& payload) {
    PacketHeader header;
    header.magic = htons(MAGIC_NUMBER);
    header.type = type;
    header.flags = flags;
    header.length = htonl(payload.size());
    header.checksum = htonl(calculateCRC32(payload.data(), payload.size()));

    std::string packet(sizeof(PacketHeader) + payload.size(), 0);
    memcpy(&packet[0], &header, sizeof(PacketHeader));
    memcpy(&packet[sizeof(PacketHeader)], payload.data(), payload.size());
    return packet;
}

/**
 * @brief Xây dựng gói tin trạng thái thời gian thực (Realtime Status).
 * Gói tin này chứa toàn bộ thông tin trạng thái của AGV để gửi lên server.
 * @details Gói tin được xây dựng theo giao thức nhị phân tự định nghĩa để tối ưu băng thông.
 * Dữ liệu LiDAR có thể được nén bằng zlib nếu số lượng điểm lớn.
 */
std::string buildRealtimePacket(
    int id_agv,
    float battery_level,
    bool is_moving,
    bool is_safe,
    bool plc_connected,
    bool lidar_connected,
    bool battery_connected,
    float current_speed,
    const std::map<std::string, uint16_t>& plc_registers,
    // const std::vector<Point2D>& stable_lidar_points,   
    const std::vector<Point2D>& realtime_lidar_points,
    float current_heading,
    long timestamp) {
    
    std::vector<uint8_t> payload;
    payload.reserve(4096); // Dự trữ bộ nhớ để tránh cấp phát lại nhiều lần

    // --- Đóng gói các trường dữ liệu cơ bản ---
    uint32_t net_id = htonl(id_agv);
    payload.insert(payload.end(), (uint8_t*)&net_id, (uint8_t*)&net_id + 4);
    
    // Đóng gói mức pin
    uint32_t net_battery = float_to_net(battery_level);
    payload.insert(payload.end(), (uint8_t*)&net_battery, (uint8_t*)&net_battery + 4);
    
    // Đóng gói các cờ trạng thái boolean vào một byte duy nhất
    //
    uint8_t status_flags = 0;
    if (is_moving) status_flags |= 0x01; // Bit 0
    if (is_safe) status_flags |= 0x02;   // Bit 1
    if (plc_connected) status_flags |= 0x04;    // Bit 2
    if (lidar_connected) status_flags |= 0x08;  // Bit 3
    if (battery_connected) status_flags |= 0x10;        // Bit 4
    payload.push_back(status_flags);
    
    // Đóng gói tốc độ
    uint32_t net_speed = float_to_net(current_speed);
    payload.insert(payload.end(), (uint8_t*)&net_speed, (uint8_t*)&net_speed + 4);

        // Đóng gói Góc
    uint32_t net_current_heading = float_to_net(current_heading);
    payload.insert(payload.end(), (uint8_t*)&net_current_heading, (uint8_t*)&net_current_heading + 4);
    
    // Đóng gói timestamp
    uint64_t net_ts = htobe64(timestamp);
    payload.insert(payload.end(), (uint8_t*)&net_ts, (uint8_t*)&net_ts + 8);
    
    // --- Đóng gói dữ liệu thanh ghi PLC ---
    uint32_t plc_count = plc_registers.size();
    uint32_t net_count = htonl(plc_count);
    payload.insert(payload.end(), (uint8_t*)&net_count, (uint8_t*)&net_count + 4);
    
    for (const auto& [name, value] : plc_registers) {
        // Kiểm tra định dạng tên thanh ghi (ví dụ: "D100")
        if (name.empty() || name[0] != 'D' || name.size() < 2 || !std::isdigit(name[1])) {
            LOG_WARNING << "Invalid PLC register name: " << name << ", skipping.";
            continue;
        }
        uint8_t reg_type = name[0];
        uint16_t address;
        try {
            address = std::stoi(name.substr(1));
        } catch (...) {
            LOG_WARNING << "Invalid PLC register address in: " << name << ", skipping.";
            continue;
        }
        
        payload.push_back(reg_type);
        
        uint16_t net_addr = htons(address);
        payload.insert(payload.end(), (uint8_t*)&net_addr, (uint8_t*)&net_addr + 2);
        
        uint16_t net_val = htons(value);
        payload.insert(payload.end(), (uint8_t*)&net_val, (uint8_t*)&net_val + 2);
    }
    
    // // --- ĐÓNG GÓI STABLE POINTS (CÓ NÉN) ---
    // uint32_t stable_point_count = stable_lidar_points.size();
    // uint32_t net_count_1 = htonl(stable_point_count);
    // payload.insert(payload.end(), (uint8_t*)&net_count_1, (uint8_t*)&net_count_1 + 4);
    
    // // Quyết định có nén dữ liệu LiDAR hay không (nếu có nhiều hơn 100 điểm)
    // bool compress_points = stable_point_count > 100; 
    
    // if (compress_points) {
    //     LOG_INFO << "Compressing " << stable_point_count << " points";
    //     std::vector<uint8_t> point_data(stable_point_count * 8);
    //     size_t offset = 0;
    //     for (const auto& p : stable_lidar_points) {
    //         uint32_t net_x = float_to_net(p.x);
    //         uint32_t net_y = float_to_net(p.y);
    //         memcpy(point_data.data() + offset, &net_x, 4);
    //         memcpy(point_data.data() + offset + 4, &net_y, 4);
    //         offset += 8;
    //     }
        
    //     // Tính toán kích thước buffer cần thiết cho dữ liệu nén
    //     uLongf compressed_size = compressBound(point_data.size());
    //     std::vector<uint8_t> compressed(compressed_size);
        
    //     // Thực hiện nén với zlib, ưu tiên tốc độ (Z_BEST_SPEED)
    //     if (compress2(compressed.data(), &compressed_size, 
    //                  point_data.data(), point_data.size(), 
    //                  Z_BEST_SPEED) == Z_OK) {
    //         LOG_INFO << "[BuildPacket] Compressed " << point_data.size() 
    //              << " bytes to " << compressed_size << " bytes";
    //         // Thêm kích thước gốc của dữ liệu vào trước để phía server biết giải nén ra bao nhiêu
    //         uint32_t orig_size = htonl(point_data.size());
    //         // Ghi kích thước gốc của dữ liệu chưa nén vào payload
    //         payload.insert(payload.end(), (uint8_t*)&orig_size, (uint8_t*)&orig_size + 4);
            
    //         compressed.resize(compressed_size);
    //         payload.insert(payload.end(), compressed.begin(), compressed.end());
    //     } else {
    //         LOG_ERROR << "[BuildPacket] Compression failed!";
    //         compress_points = false;
    //     }
    // }
    
    // // Nếu không nén, ghi trực tiếp dữ liệu điểm vào payload
    // if (!compress_points) {
    //     for (const auto& p : stable_lidar_points) {
    //         uint32_t net_x = float_to_net(p.x);
    //         uint32_t net_y = float_to_net(p.y);
    //         payload.insert(payload.end(), (uint8_t*)&net_x, (uint8_t*)&net_x + 4);
    //         payload.insert(payload.end(), (uint8_t*)&net_y, (uint8_t*)&net_y + 4);
    //     }
    // }
    
    //----- Đóng gói REALTIME POINTS (Luôn là dữ liệu thô) ---
    uint32_t realtime_point_count = realtime_lidar_points.size();
    net_count = htonl(realtime_point_count);
    payload.insert(payload.end(), (uint8_t*)&net_count, (uint8_t*)&net_count + 4);

    for (const auto& p : realtime_lidar_points) {
        uint32_t net_x = float_to_net(p.x);
        uint32_t net_y = float_to_net(p.y);
        payload.insert(payload.end(), (uint8_t*)&net_x, (uint8_t*)&net_x + 4);
        payload.insert(payload.end(), (uint8_t*)&net_y, (uint8_t*)&net_y + 4);
    }


    // --- Xây dựng header của gói tin ---
    std::string packet = buildPacket(REALTIME_STATUS, 0, payload);
    
    return packet;
}


/**
 * @brief Xây dựng gói tin lệnh điều khiển (Navigation Command).
 * @param command_type Loại lệnh (STOP, MOVE_TO_POINT, etc.).
 * @param target_x Tọa độ X mục tiêu.
 * @param target_y Tọa độ Y mục tiêu.
 * @param target_angle Góc mục tiêu.
 * @param speed Tốc độ thực hiện.
 * @return Chuỗi string chứa gói tin nhị phân hoàn chỉnh.
 */
std::string buildNavigationCommand(
    uint8_t command_type,
    float target_x,
    float target_y,
    float target_angle,
    float speed) {
    
    // Cấu trúc payload đơn giản cho lệnh điều khiển
    std::vector<uint8_t> payload(17);  // 1 + 4*4
    
    payload[0] = command_type;
    uint32_t net_x = float_to_net(target_x);
    memcpy(&payload[1], &net_x, 4);
    uint32_t net_y = float_to_net(target_y);
    memcpy(&payload[5], &net_y, 4);
    uint32_t net_angle = float_to_net(target_angle);
    memcpy(&payload[9], &net_angle, 4);
    uint32_t net_speed = float_to_net(speed);
    memcpy(&payload[13], &net_speed, 4);
    
    //Tạo header
    std::string packet = buildPacket(NAVIGATION_COMMAND, 0, payload);
    
    return packet;
}

/**
 * @brief Xây dựng gói tin heartbeat.
 * @param agv_id ID của AGV.
 * @return Chuỗi string chứa gói tin nhị phân hoàn chỉnh.
 */
std::string buildHeartbeatPacket(int agv_id) {
    std::vector<uint8_t> payload(8);  // agv_id (4) + client_timestamp (4)
    uint32_t net_id = htonl(static_cast<uint32_t>(agv_id));
    memcpy(payload.data(), &net_id, 4);
    uint32_t timestamp = htonl(static_cast<uint32_t>(std::time(nullptr)));  // Simple timestamp
    memcpy(payload.data() + 4, &timestamp, 4);

    return buildPacket(HEARTBEAT, 0, payload);
}
/**
 * @brief Phân tích header của một gói tin từ dữ liệu nhận được.
 * @param data Con trỏ tới buffer chứa dữ liệu gói tin.
 * @param header Tham chiếu tới cấu trúc PacketHeader để lưu kết quả.
 * @return true nếu header hợp lệ (đúng magic number), ngược lại false.
 */
bool parsePacketHeader(const uint8_t* data, PacketHeader& header) {
    header.magic = ntohs(*(uint16_t*)data);
    if (header.magic != MAGIC_NUMBER) { return false; }
    
    header.type = data[2];
    header.flags = data[3];
    memcpy(&header.length, data + 4, sizeof(uint32_t));
    memcpy(&header.checksum, data + 8, sizeof(uint32_t));
    header.length = ntohl(header.length);
    header.checksum = ntohl(header.checksum);
    
    return true;
}

/**
 * @brief Phân tích payload của một gói tin lệnh điều khiển.
 * @param data Chuỗi string chứa payload của gói tin.
 * @param command_type Tham chiếu để lưu loại lệnh.
 * @param target_x Tham chiếu để lưu tọa độ X.
 * @param target_y Tham chiếu để lưu tọa độ Y.
 * @param target_angle Tham chiếu để lưu góc.
 * @param speed Tham chiếu để lưu tốc độ.
 * @return true nếu phân tích thành công, ngược lại false.
 */
bool parseNavigationCommand(const std::string& data,
                               uint8_t& command_type,
                               float& target_x,
                               float& target_y,
                               float& target_angle,
                               float& speed) {
        // Check if buffer is large enough (1 byte for type + 4 floats * 4 bytes each)   
        if (data.size() < 1 + 4 * sizeof(uint32_t)) {
            LOG_WARNING << "[BinaryProtocol] Buffer too small: " << data.size()
                        << " bytes, expected at least " << (1 + 4 * sizeof(uint32_t)) << " bytes";
            return false;
        }
        // Extract type
        command_type = static_cast<uint8_t>(data[0]);
        // Extract floats (assuming they are stored as uint32_t in network byte order)
        uint32_t net_x, net_y, net_angle, net_speed;
        memcpy(&net_x, data.data() + 1, sizeof(uint32_t));
        memcpy(&net_y, data.data() + 1 + sizeof(uint32_t), sizeof(uint32_t));
        memcpy(&net_angle, data.data() + 1 + 2 * sizeof(uint32_t), sizeof(uint32_t));
        memcpy(&net_speed, data.data() + 1 + 3 * sizeof(uint32_t), sizeof(uint32_t));

        // Convert from network to host byte order
        net_x = ntohl(net_x);
        net_y = ntohl(net_y);
        net_angle = ntohl(net_angle);
        net_speed = ntohl(net_speed);

        // Reinterpret uint32_t as float
        memcpy(&target_x, &net_x, sizeof(float));
        memcpy(&target_y, &net_y, sizeof(float));
        memcpy(&target_angle, &net_angle, sizeof(float));
        memcpy(&speed, &net_speed, sizeof(float));

        // Log parsed values for debugging
        LOG_DEBUG << "[BinaryProtocol] Parsed command: type=" << static_cast<int>(command_type)
                  << ", x=" << target_x << ", y=" << target_y
                  << ", angle=" << target_angle << ", speed=" << speed;

        return true;
 }

/**
 * @brief Phân tích payload của gói tin heartbeat ACK.
 * @param data Chuỗi string chứa payload.
 * @param server_timestamp Tham chiếu để lưu timestamp từ server.
 * @return true nếu thành công.
 */
bool parseHeartbeatAck(const std::string& data, int& server_timestamp) {
    if (data.size() < 4) return false;
    uint32_t net_ts;
    memcpy(&net_ts, data.data(), 4);
    server_timestamp = ntohl(net_ts);
    return true;
}

} // namespace BinaryProtocol

/**
 * @brief Gửi gói tin trạng thái AGV lên server.
 * @param status Cấu trúc AGVStatusPacket chứa thông tin trạng thái cần gửi.
 */
void CommunicationServer::sendStatus(const AGVStatusPacket& status) {
    std::string packet = BinaryProtocol::buildRealtimePacket(
        status.id_agv,
        status.battery_level,
        status.is_moving,
        status.is_safe,
        status.plc_connected,
        status.lidar_connected,
        status.battery_connected,
        status.current_speed,
        status.plc_registers,
        // status.stable_lidar_points,    // Dữ liệu stable
        status.realtime_lidar_points,  // Dữ liệu realtime 
        status.current_heading,      
        status.timestamp
    );
    
    // Gửi gói tin với tiền tố là độ dài gói tin (4 bytes)
    uint32_t size = htonl(packet.size());
    LOG_INFO << "[SERSEND] [khaipv] Sending packet size: " << packet.size(); 
            //  << ", Stable points: " << status.stable_lidar_points.size()
    LOG_INFO << "[SERSEND] [khaipv] Realtime points: " << status.realtime_lidar_points.size();
    LOG_INFO << "[SERSEND] [khaipv]: id: " << std::dec << status.id_agv
             << ", battery: " << status.battery_level
             << ", moving: " << status.is_moving
             << ", safe: " << status.is_safe
             << ", plc_conn: " << status.plc_connected
             << ", lidar_conn: " << status.lidar_connected
             << ", battery_conn: " << status.battery_connected
             << ", speed: " << status.current_speed
             << ", current heading: "<< status.current_heading
             << ", timestamp: " << status.timestamp;
    std::lock_guard<std::mutex> lock(connection_mutex);
    if (send(socket_fd, &size, 4, MSG_NOSIGNAL) == 4) {
        if (send(socket_fd, packet.data(), packet.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(packet.size())) {
            // Gửi PING ngay sau khi gửi gói trạng thái thành công để đo độ trễ
            //sendPing();
            total_packets_sent++;
            total_bytes_sent += packet.size() + 4;
            LOG_INFO << "[SERSEND] Successfully sent packet #" << total_packets_sent;
        } else {
            LOG_ERROR << "[SERSEND] Failed to send packet data";
        }
    } else {
        LOG_ERROR << "[SERSEND] Failed to send size prefix";
    }
}

/**
 * @brief Gửi heartbeat.
 */
void CommunicationServer::sendHeartbeat() {
    std::string packet = BinaryProtocol::buildHeartbeatPacket(AGV_ID);
    uint32_t size = htonl(packet.size());
    
    std::lock_guard<std::mutex> lock(connection_mutex);
    if (send(socket_fd, &size, 4, MSG_NOSIGNAL) == 4) {
        if (send(socket_fd, packet.data(), packet.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(packet.size())) {
            total_packets_sent++;
            total_bytes_sent += packet.size() + 4;
            LOG_DEBUG << "[CommServer] Heartbeat sent";
        }
    }
}

/**
 * @brief Gửi gói tin PING để đo độ trễ.
 */
void CommunicationServer::sendPing() {
    // Lấy thời gian hiện tại với độ chính xác cao (microseconds)
    auto t1 = std::chrono::high_resolution_clock::now();
    uint64_t timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(t1.time_since_epoch()).count();

    // Chuẩn bị payload chứa timestamp
    std::vector<uint8_t> payload(8);
    uint64_t net_ts = htobe64(timestamp_us); // Chuyển sang network byte order
    memcpy(payload.data(), &net_ts, 8);

    // Xây dựng gói tin hoàn chỉnh
    std::string packet = BinaryProtocol::buildPacket(BinaryProtocol::PING, 0, payload);
    
    // Gửi gói tin (hàm sendData đã bao gồm size prefix)
    sendData(packet);

    LOG_INFO << "[Latency Check] Sent PING with timestamp " << timestamp_us;
}

/**
 * @brief Xử lý heartbeat ACK.
 * @param server_timestamp Timestamp từ server.
 */
void CommunicationServer::handleHeartbeatAck(int server_timestamp) {
    last_heartbeat_ack_time = std::chrono::steady_clock::now();
    LOG_INFO << "[CommServer] Heartbeat ACK received with server timestamp: " 
             << server_timestamp;  // This should appear in logs
}

/**
 * @brief Kiểm tra heartbeat timeout.
 * @return true nếu timeout.
 */
bool CommunicationServer::checkHeartbeatTimeout() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_heartbeat_ack_time).count();
    
    // Only consider timeout if we've been connected for a while
    auto connection_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_send_time).count();
    
    bool timeout = (duration > heartbeat_timeout_ms) && (connection_duration > heartbeat_timeout_ms);
    
    // Chỉ ghi log cảnh báo nếu thực sự timeout
    if (timeout) {
        LOG_WARNING << "[CommServer] Heartbeat timeout: " << duration 
                   << "ms since last ACK (threshold: " << heartbeat_timeout_ms << "ms)";
    }
    
    return timeout;
}

/**
 * @brief Xử lý dữ liệu nhị phân nhận được từ server.
 * Hàm này phân tích header, kiểm tra checksum và xử lý payload tương ứng.
 * @param data Chuỗi string chứa toàn bộ gói tin (header + payload).
 */
void CommunicationServer::processReceivedData(const std::string& data) {
    LOG_INFO << "[CommServe]";
    if (data.size() < 12) return;
    
    BinaryProtocol::PacketHeader header;
    if (!BinaryProtocol::parsePacketHeader((uint8_t*)data.data(), header)) {
        LOG_WARNING << "[CommServer]Invalid packet header";
        return;
    }
    LOG_DEBUG << "[CommServer] Received packet type: " << (int)header.type;
    if (header.length > data.size() - 12) {
        LOG_WARNING << "[CommServer]Incomplete packet: expected " << header.length << ", got " << data.size() - 12;
        return;
    }
    
    // Xác thực checksum của payload
    uint32_t calc_crc = BinaryProtocol::calculateCRC32(
        (uint8_t*)data.data() + 12, header.length);
    if (calc_crc != header.checksum) {
        LOG_WARNING << "Checksum mismatch";
        return;
    }
    
    std::string payload = data.substr(12, header.length);
    
    if (header.type == BinaryProtocol::NAVIGATION_COMMAND) {
        // Nếu là gói tin lệnh điều khiển, phân tích và đẩy vào hàng đợi lệnh
        uint8_t cmd_type;
        float x, y, angle, speed;
        if (BinaryProtocol::parseNavigationCommand(payload, cmd_type, x, y, angle, speed)) {
            NavigationCommand cmd;
            cmd.type = static_cast<NavigationCommand::Type>(cmd_type);
            cmd.target_x = x;
            cmd.target_y = y;
            cmd.target_angle = angle;
            cmd.speed = speed;
            
            handleCommand(cmd);
        }
    } else if (header.type == BinaryProtocol::HEARTBEAT_ACK) {
        int server_ts;
        if (BinaryProtocol::parseHeartbeatAck(payload, server_ts)) {
            handleHeartbeatAck(server_ts);
            LOG_INFO << "[CommServer] Processed heartbeat ACK successfully"; // Add this
        } else {
            LOG_WARNING << "[CommServer] Failed to parse heartbeat ACK"; // Add this
        }
    } else if (header.type == BinaryProtocol::PONG) {
        if (payload.size() >= 8) {
            // Lấy lại timestamp t1 đã gửi đi
            uint64_t t1_us;
            memcpy(&t1_us, payload.data(), 8);
            t1_us = be64toh(t1_us);

            // Lấy thời gian hiện tại t2
            auto t2 = std::chrono::high_resolution_clock::now();
            uint64_t t2_us = std::chrono::duration_cast<std::chrono::microseconds>(t2.time_since_epoch()).count();

            // Tính toán và in ra độ trễ
            double latency_ms = (t2_us - t1_us) / 1000.0;
            LOG_INFO << "[Latency Check] Received PONG. Round-Trip Time: " << latency_ms << " ms";
        }
    }
}

/**
 * @brief Constructor của lớp CommunicationServer.
 * @param server_ip Địa chỉ IP của server.
 * @param server_port Cổng của server.
 * @param send_interval_ms Khoảng thời gian (ms) giữa các lần gửi trạng thái.
 */
CommunicationServer::CommunicationServer(const std::string& server_ip,
                                       int server_port,
                                       int send_interval_ms)
    : server_ip(server_ip), server_port(server_port), socket_fd(-1),
      is_connected(false), is_running(false),
      total_packets_sent(0), total_packets_received(0),
      total_bytes_sent(0), total_bytes_received(0),
      send_interval_ms(send_interval_ms),
      reconnect_interval_ms(5000),
      heartbeat_interval_ms(1000),
      socket_timeout_ms(5000),
      heartbeat_timeout_ms(10000)
{
    last_heartbeat_ack_time = std::chrono::steady_clock::now();
    LOG_INFO << "[CommServer] Initialized for " << server_ip << ":" << server_port;
}

/**
 * @brief Destructor, đảm bảo dừng các luồng và ngắt kết nối.
 */
CommunicationServer::~CommunicationServer() {
    stop();
}

/**
 * @brief Thực hiện kết nối tới server.
 */
bool CommunicationServer::connect() {
    std::lock_guard<std::mutex> lock(connection_mutex);
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

/**
 * @brief Ngắt kết nối khỏi server.
 */
void CommunicationServer::disconnect() {
    std::lock_guard<std::mutex> lock(connection_mutex);
    if (!is_connected) return;
    
    is_connected = false;
    closeSocket();
    
    if (connection_callback) {
        connection_callback(false);
    }
    
    LOG_INFO << "[CommServer] Disconnected";
}

/**
 * @brief Bắt đầu các luồng giao tiếp (gửi, nhận, kết nối lại).
 */
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
    
    // Khởi tạo và chạy các luồng cho việc nhận, gửi và kết nối lại.
    receive_thread = std::make_unique<std::thread>(&CommunicationServer::receiveThread, this);
    send_thread = std::make_unique<std::thread>(&CommunicationServer::sendThread, this);
    reconnect_thread = std::make_unique<std::thread>(&CommunicationServer::reconnectThread, this);
    
    LOG_INFO << "[CommServer] Started";
    return true;
}

/**
 * @brief Dừng các luồng giao tiếp và dọn dẹp.
 */
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

/**
 * @brief Gửi lệnh dừng khẩn cấp ngay lập tức.
 * Lệnh này được ưu tiên gửi đi mà không qua hàng đợi.
 */
void CommunicationServer::sendEmergencyStop() {
    std::string packet = BinaryProtocol::buildNavigationCommand(
        static_cast<uint8_t>(NavigationCommand::EMERGENCY_STOP), 0, 0, 0, 0);
    uint32_t size = htonl(packet.size());
    
    std::lock_guard<std::mutex> lock(connection_mutex);
    if (send(socket_fd, &size, 4, MSG_NOSIGNAL) == 4) {
        send(socket_fd, packet.data(), packet.size(), MSG_NOSIGNAL);
        LOG_WARNING << "[CommServer] Emergency stop sent";
    }
}

/** @brief Đăng ký hàm callback để lấy dữ liệu trạng thái AGV. */
void CommunicationServer::setStatusCallback(StatusCallback callback) {
    status_callback = callback;
}
/** @brief Đăng ký hàm callback để xử lý lệnh điều hướng. */
void CommunicationServer::setCommandCallback(CommandCallback callback) {
    command_callback = callback;
}
/** @brief Đăng ký hàm callback để xử lý sự kiện kết nối/mất kết nối. */
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

/**
 * @brief Tính toán tốc độ truyền dữ liệu trung bình (KB/s).
 */
float CommunicationServer::getDataRate() const {
    auto now = std::chrono::steady_clock::now();
    auto latest = std::max(last_send_time, last_receive_time);
    float duration = std::chrono::duration<float>(now - latest).count();
    if (duration > 0) {
        return (total_bytes_sent + total_bytes_received) / duration / 1024.0f;
    }
    return 0.0f;
}

/**
 * @brief Lấy thời điểm giao tiếp cuối cùng (gửi hoặc nhận).
 */
long CommunicationServer::getLastCommunicationTime() const {
    auto latest = std::max(last_send_time, last_receive_time);
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        latest.time_since_epoch()).count();
}

/**
 * @brief Luồng chuyên trách nhận dữ liệu từ server.
 * Luồng sẽ đọc tiền tố 4-byte để biết kích thước gói tin, sau đó đọc toàn bộ gói tin.
 */
void CommunicationServer::receiveThread() {
    LOG_INFO << "[CommServer] Receive thread started";
    
    // Vòng lặp chính của luồng, chạy tant que is_running là true.
    while (is_running) {
        int current_socket_fd = -1;

        // === BẮT ĐẦU VÙNG KHÓA MUTEX NGẮN ===
        // Chỉ khóa mutex trong thời gian cực ngắn để lấy socket descriptor
        // và kiểm tra trạng thái kết nối. Điều này tránh giữ lock trong lúc
        // gọi hàm recv() có thể bị block, là nguyên nhân chính gây deadlock.
        {
            std::lock_guard<std::mutex> lock(connection_mutex);
            if (!is_connected) {
                // Nếu không có kết nối, luồng sẽ ngủ một chút để tránh
                // lãng phí CPU, sau đó lặp lại để kiểm tra lại.
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            current_socket_fd = socket_fd;
        }
        // === KẾT THÚC VÙNG KHÓA MUTEX NGẮN ===

        // === BẮT ĐẦU VÙNG KHÔNG KHÓA MUTEX ===
        // Thực hiện các thao tác mạng có thể bị block ở đây
        char size_buf[4];
        // Đọc 4 byte đầu tiên để biết kích thước của gói tin tiếp theo.
        // MSG_WAITALL đảm bảo hàm recv sẽ block cho đến khi đọc đủ 4 byte hoặc có lỗi.
        ssize_t bytes_received = recv(current_socket_fd, size_buf, 4, MSG_WAITALL);

        if (bytes_received == 4) {
            // Chuyển đổi 4 byte kích thước từ thứ tự byte mạng sang host.
            uint32_t packet_size = ntohl(*(uint32_t*)size_buf);

            if (packet_size > 0 && packet_size < 65536) { // Sanity check
                std::string packet_data(packet_size, 0);
                // Đọc toàn bộ gói tin với kích thước đã biết.
                bytes_received = recv(current_socket_fd, &packet_data[0], packet_size, MSG_WAITALL);

                if (bytes_received == static_cast<ssize_t>(packet_size)) {
                    // Nếu đọc thành công, xử lý dữ liệu và cập nhật thống kê.
                    processReceivedData(packet_data);
                    total_packets_received++;
                    total_bytes_received += packet_size + 4;
                    last_receive_time = std::chrono::steady_clock::now();
                } else {
                     // Nếu không đọc đủ dữ liệu, coi như kết nối lỗi và ngắt kết nối.
                     LOG_WARNING << "[CommServer] Failed to receive full packet data. Disconnecting.";
                     disconnect(); // Lỗi khi nhận, ngắt kết nối
                }
            }
        } else if (bytes_received == 0) {
            // recv trả về 0 nghĩa là server đã chủ động đóng kết nối.
            LOG_WARNING << "[CommServer] Connection closed by server. Disconnecting.";
            disconnect(); // Server đã đóng kết nối
        } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
            // Chỉ ngắt kết nối nếu đó là lỗi thực sự, không phải timeout tạm thời
            LOG_ERROR << "[CommServer] Receive failed: " << strerror(errno) << ". Disconnecting.";
            disconnect();
        }
        // === KẾT THÚC VÙNG KHÔNG KHÓA MUTEX ===
    }
    
    LOG_INFO << "[CommServer] Receive thread stopped";
}


/**
 * @brief Luồng chuyên trách gửi dữ liệu lên server. (ĐÃ SỬA LỖI)
 * Luồng sẽ không giữ mutex khi gọi hàm callback hoặc `send` (có thể bị block).
 */
void CommunicationServer::sendThread() {
    LOG_INFO << "[CommServer] Send thread started";
    
    auto last_status_time = std::chrono::steady_clock::now();
    auto last_heartbeat_time = std::chrono::steady_clock::now();
    auto last_ping_time = std::chrono::steady_clock::now();

    while (is_running) {
        try { // <<< BẮT ĐẦU KHỐI TRY ĐỂ BẮT LỖI
            
            // Luồng ngủ một khoảng ngắn để tránh lãng phí CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if (!is_connected) {
                continue;
            }
            
            auto now = std::chrono::steady_clock::now();
            
            // Gửi trạng thái định kỳ
            if (status_callback && std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_time).count() >= send_interval_ms) {
                AGVStatusPacket status = status_callback();
                sendStatus(status);
                last_status_time = now;
            }
            
            // Gửi heartbeat định kỳ
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_heartbeat_time).count() >= heartbeat_interval_ms) {
                sendHeartbeat();
                last_heartbeat_time = now;
            }

            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ping_time).count() >= 1000) {
                sendPing(); // Gửi độc lập
                last_ping_time = now;
            }

            last_send_time = now;

        } catch (const std::exception& e) {
            // <<< NẾU CÓ LỖI, NÓ SẼ ĐƯỢC BẮT VÀ GHI LOG TẠI ĐÂY
            LOG_ERROR << "[CommServer] Exception in sendThread: " << e.what() << ". Thread will continue.";
            // Ngủ một chút để tránh lặp lại lỗi quá nhanh
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } catch (...) {
            // <<< Bắt cả các lỗi không xác định
            LOG_ERROR << "[CommServer] Unknown exception in sendThread. Thread will continue.";
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    LOG_INFO << "[CommServer] Send thread stopped.";
}

/**
 * @brief Luồng chuyên trách kết nối lại server khi bị mất kết nối.
 * Luồng sẽ thử kết nối lại sau mỗi `reconnect_interval_ms`.
 */
void CommunicationServer::reconnectThread() {
    LOG_INFO << "[CommServer] Reconnect thread started";
    
    while (is_running) {
        if (!is_connected || checkHeartbeatTimeout()) {
            LOG_INFO << "[CommServer] Heartbeat timeout or disconnected. Reconnecting...";
            disconnect();  // Close old socket
            connect();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms));
    }
    
    LOG_INFO << "[CommServer] Reconnect thread stopped";
}

/**
 * @brief Tạo một socket mới và cấu hình các tùy chọn cần thiết.
 */
bool CommunicationServer::createSocket() {
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        LOG_ERROR << "[CommServer] Failed to create socket";
        return false;
    }
    
    int opt = 1;
    setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // Thiết lập socket ở chế độ không chặn (non-blocking) cho hàm connect
    int flags = fcntl(socket_fd, F_GETFL, 0);
    fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);
    
    
    return true;
}

/**
 * @brief Thực hiện kết nối tới địa chỉ server đã được cấu hình.
 */
bool CommunicationServer::connectToServer() {
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);
    setSocketTimeout(socket_timeout_ms);
    if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0) {
        LOG_ERROR << "[CommServer] Invalid IP: " << server_ip;
        return false;
    }

    // Vì socket là non-blocking, hàm connect sẽ trả về ngay lập tức
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

        // Dùng select để chờ socket sẵn sàng để ghi (kết nối thành công)
        int result = select(socket_fd + 1, nullptr, &write_set, nullptr, &timeout);
        if (result <= 0) {
            LOG_ERROR << "[CommServer] Connection timeout or error: " << (result == 0 ? "timeout" : strerror(errno));
            return false;
        }

        // Kiểm tra SO_ERROR để chắc chắn kết nối thành công
        int so_error = 0;
        socklen_t len = sizeof(so_error);
        if (getsockopt(socket_fd, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0) {
            LOG_ERROR << "[CommServer] getsockopt failed: " << strerror(errno);
            return false;
        }
        if (so_error != 0) {
            LOG_ERROR << "[CommServer] Connect failed with error: " << strerror(so_error);
            return false;
        }
    }

    // Unset non-blocking mode sau connect
    int flags = fcntl(socket_fd, F_GETFL, 0);
    if (flags == -1) {
        LOG_ERROR << "[CommServer] Failed to get socket flags: " << strerror(errno);
        return false;
    }
    if (fcntl(socket_fd, F_SETFL, flags & ~O_NONBLOCK) == -1) {
        LOG_ERROR << "[CommServer] Failed to unset non-blocking mode: " << strerror(errno);
        return false;
    }

    LOG_INFO << "[CommServer] Connected to " << server_ip << ":" << server_port;
    return true;
}

/**
 * @brief Đóng socket hiện tại.
 */
void CommunicationServer::closeSocket() {
    if (socket_fd >= 0) {
        close(socket_fd);
        socket_fd = -1;
    }
}

/**
 * @brief Gửi dữ liệu qua socket (hàm này chủ yếu cho giao thức cũ, hiện ít dùng).
 */
bool CommunicationServer::sendData(const std::string& data) {
    if (!is_connected || socket_fd < 0) return false;
    
    // Gửi tiền tố kích thước trước
    uint32_t size = htonl(data.size());
    if (send(socket_fd, &size, 4, MSG_NOSIGNAL) != 4) {
        LOG_ERROR << "[CommServer] Failed to send size prefix: " << strerror(errno);
        is_connected = false;
        return false;
    }

    ssize_t bytes_sent = send(socket_fd, data.data(), data.size(), MSG_NOSIGNAL);
    
    if (bytes_sent < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            LOG_ERROR << "[CommServer] Send failed: " << strerror(errno);
            is_connected = false;
            return false;
        }
    }
    
    return bytes_sent > 0;
}

/**
 * @brief Nhận dữ liệu từ socket (hàm này chủ yếu cho giao thức cũ, hiện ít dùng).
 */
std::string CommunicationServer::receiveData() {
    if (!is_connected || socket_fd < 0) return "";
    
    char buffer[4096];
    ssize_t bytes_received = recv(socket_fd, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
    
    if (bytes_received > 0) {
        buffer[bytes_received] = '\0';
        return std::string(buffer, bytes_received);
    }
    
    return "";
}

/**
 * @brief Xử lý một lệnh điều khiển nhận được.
 * Đẩy lệnh vào hàng đợi và thông báo cho luồng chính.
 */
void CommunicationServer::handleCommand(const NavigationCommand& cmd) {
    std::lock_guard<std::mutex> lock(command_mutex);
    command_queue.push(cmd);
    command_cv.notify_one();
    
    if (command_callback) {
        command_callback(cmd);
    }
    
    LOG_INFO << "[CommServer] Command received: Type=" << cmd.type;
}

/**
 * @brief Lấy timestamp hiện tại (milliseconds since epoch).
 */
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

/**
 * @brief Thiết lập timeout cho các thao tác gửi/nhận trên socket.
 */
void CommunicationServer::setSocketTimeout(int timeout_ms) {
    if (socket_fd < 0) return;
    
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

} // namespace ServerComm