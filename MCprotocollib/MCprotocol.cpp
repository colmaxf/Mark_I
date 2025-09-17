#include "MCprotocol.h"
#include <cstring>
#include <iomanip>

/// @name MC Protocol Command Codes
/// @{
static const uint8_t CMD_BATCH_READ_WORD = 0x01;   ///< Batch read in word units
static const uint8_t CMD_BATCH_WRITE_WORD = 0x03;  ///< Batch write in word units
static const uint8_t CMD_BATCH_READ_BIT = 0x00;    ///< Batch read in bit units
static const uint8_t CMD_BATCH_WRITE_BIT = 0x02;   ///< Batch write in bit units
/// @}

/// @name MC Protocol Response Codes
/// @{
static const uint8_t RESP_BATCH_READ_WORD = 0x81;  ///< Response for batch read word
static const uint8_t RESP_BATCH_WRITE_WORD = 0x83; ///< Response for batch write word
static const uint8_t RESP_BATCH_READ_BIT = 0x80;   ///< Response for batch read bit
static const uint8_t RESP_BATCH_WRITE_BIT = 0x82;  ///< Response for batch write bit
/// @}

MCProtocol::MCProtocol(const std::string& ip, int port, uint16_t timer_250ms)
    : m_plc_ip(ip), m_plc_port(port), m_is_connected(false), m_monitoring_timer(timer_250ms) {

    LOG_INFO << "[MCProtocol] Instance created for PLC at " << ip << ":" << port;
    m_socket_fd = INVALID_SOCKET;

}

MCProtocol::~MCProtocol() {
    disconnect();
}

std::string MCProtocol::vectorToHexString(const std::vector<unsigned char>& data) {
    std::stringstream ss;
    for (size_t i = 0; i < data.size(); ++i) {
        if (i > 0) ss << " ";
        ss << "0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
           << static_cast<int>(data[i]);
    }
    return ss.str();
}

std::string MCProtocol::vectorToHexString(const std::vector<unsigned short>& data) {
    std::stringstream ss;
    for (size_t i = 0; i < data.size(); ++i) {
        if (i > 0) ss << " ";
        ss << "0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(4)
           << static_cast<int>(data[i]);
    }
    return ss.str();
}

bool MCProtocol::connect() {
    // Already connected, return success
    if (m_is_connected) return true;

    // Create TCP socket
    m_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_socket_fd == INVALID_SOCKET) {

        LOG_ERROR << "[MCProtocol] Failed to create socket";
        return false;

    }

    // Prepare server address structure
    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(m_plc_port);
    
    // Convert IP address from string to binary
    if (inet_pton(AF_INET, m_plc_ip.c_str(), &server_addr.sin_addr) <= 0) {

        LOG_ERROR << "[MCProtocol] Invalid IP address: " << m_plc_ip;
        closesocket(m_socket_fd);
        m_socket_fd = INVALID_SOCKET;
        return false;

    }

    // Establish connection to PLC
    if (::connect(m_socket_fd, (sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {

        LOG_ERROR << "[MCProtocol] Failed to connect to PLC at " << m_plc_ip << ":" << m_plc_port;
        closesocket(m_socket_fd);
        m_socket_fd = INVALID_SOCKET;
        return false;

    }

    m_is_connected = true;

    LOG_INFO << "[MCProtocol] Connected to PLC at " << m_plc_ip << ":" << m_plc_port;
    return true;

}

void MCProtocol::disconnect() {
    if (m_socket_fd != INVALID_SOCKET) {
        closesocket(m_socket_fd);
        m_socket_fd = INVALID_SOCKET;
    }
    m_is_connected = false;
}

std::vector<uint16_t> MCProtocol::readWords(const std::string& device, uint32_t start_addr, uint16_t count) {
    
    std::lock_guard<std::mutex> lock(m_socket_mutex);
    
    // Check connection status
    if (!m_is_connected) {
        LOG_ERROR << "[MCProtocol] Read operation failed: Not connected to PLC";
        throw std::runtime_error("[MCProtocol]Not connected to PLC");
    }

    // Get device code for the specified device type
    uint16_t device_code = getDeviceCode(device);
    
    // Build MC Protocol 1E frame for batch read word command
    // Example: D100
    /*{
    0x01,  // Batch read in word units

    0xFF,  // PC no.

    0x0A,  //Monitoring timer: 000AH = 10 => 10 * 250ms = 2.5 s.
    0x00,

    0x64, // Head device: D100
    0x00,
    0x00,
    0x00,
    0x20,
    0x44,

    0x01, // Number of device points: 1 word -== 16bit
    0x00
    }
    */

    std::vector<uint8_t> request;
    request.push_back(CMD_BATCH_READ_WORD);      // Command: Batch read in word units
    request.push_back(0xFF);                     // PC No: Fixed value 0xFF
    
    // Monitoring timer (2 bytes, little endian)
    request.push_back(m_monitoring_timer & 0xFF);           // Low byte
    request.push_back((m_monitoring_timer >> 8) & 0xFF);    // High byte
    
    // Head device number (4 bytes, little endian)
    request.push_back(start_addr & 0xFF);                 // Byte 0
    request.push_back((start_addr >> 8) & 0xFF);          // Byte 1
    request.push_back((start_addr >> 16) & 0xFF);         // Byte 2
    request.push_back((start_addr >> 24) & 0xFF);         // Byte 3
    
    // Device code (2 bytes, little endian)
    request.push_back(device_code & 0xFF);                // Low byte
    request.push_back((device_code >> 8) & 0xFF);         // High byte
    
    // Number of device points (2 bytes, little endian)
    request.push_back(count & 0xFF);                      // Low byte
    request.push_back((count >> 8) & 0xFF);               // High byte

    LOG_INFO << "[MCProtocol] Sending request: " << vectorToHexString(request);

    // Send request frame to PLC
    if (send(m_socket_fd, (char*)request.data(), request.size(), 0) == SOCKET_ERROR) {

        LOG_ERROR << "[MCProtocol] Failed to send read bits request";
        throw std::runtime_error("[MCProtocol] Failed to send request");

    }

    // Receive response frame from PLC
    std::vector<uint8_t> response(2 + count * 2); // Response header (2 bytes) + data
    int bytes_received = recv(m_socket_fd, (char*)response.data(), response.size(), 0);

    LOG_INFO << "[MCProtocol] Received response: " << vectorToHexString(response);

    if (bytes_received <= 0) {
        LOG_ERROR << "[MCProtocol] Failed to receive response";
        throw std::runtime_error("[MCProtocol] Failed to receive response");
    }

    // Validate response frame
    if (response[0] != RESP_BATCH_READ_WORD) {
        LOG_ERROR << "[MCProtocol] Invalid response header";
        throw std::runtime_error("[MCProtocol] Invalid response header for read operation");
    }
    
    // Check completion code (0x00 = success)
    if (response[1] != 0x00) {
        LOG_ERROR << "[MCProtocol] PLC returned error code: " << std::to_string(response[1]);
        throw std::runtime_error("[MCProtocol] PLC returned error code: " + std::to_string(response[1]));
    }

    // Convert received data from little endian to host format
    std::vector<uint16_t> result;
    for (int i = 0; i < count; i++) {
        // Combine low and high bytes (little endian format)
        uint16_t value = response[2 + i * 2] | (response[2 + i * 2 + 1] << 8);
        result.push_back(value);
    }
    LOG_INFO << "[MCProtocol] Received data: " << vectorToHexString(result);

    return result;
}

bool MCProtocol::writeWords(const std::string& device, uint32_t start_addr, const std::vector<uint16_t>& values) {
    
    std::lock_guard<std::mutex> lock(m_socket_mutex);
    
    // Check connection status
    if (!m_is_connected) {
        LOG_ERROR << "[MCProtocol] Write operation failed: Not connected to PLC";
        throw std::runtime_error("[MCProtocol] Not connected to PLC");
    }

    uint16_t device_code = getDeviceCode(device);
    uint16_t count = values.size();
    
    // Build MC Protocol 1E frame for batch write word command
    std::vector<uint8_t> request;
    request.push_back(CMD_BATCH_WRITE_WORD);     // Command: Batch write in word units
    request.push_back(0xFF);                     // PC No: Fixed value 0xFF
    
    // Monitoring timer (2 bytes, little endian)
    request.push_back(m_monitoring_timer & 0xFF);           // Low byte
    request.push_back((m_monitoring_timer >> 8) & 0xFF);    // High byte
    
    // Head device number (4 bytes, little endian)
    request.push_back(start_addr & 0xFF);                 // Byte 0
    request.push_back((start_addr >> 8) & 0xFF);          // Byte 1
    request.push_back((start_addr >> 16) & 0xFF);         // Byte 2
    request.push_back((start_addr >> 24) & 0xFF);         // Byte 3
    
    // Device code (2 bytes, little endian)
    request.push_back(device_code & 0xFF);                // Low byte
    request.push_back((device_code >> 8) & 0xFF);         // High byte
    
    // Number of device points (2 bytes, little endian)
    request.push_back(count & 0xFF);                      // Low byte
    request.push_back((count >> 8) & 0xFF);               // High byte
    
    // Data to write (convert to little endian format)
    for (uint16_t value : values) {
        request.push_back(value & 0xFF);                  // Low byte first
        request.push_back((value >> 8) & 0xFF);           // High byte second
    }
    LOG_INFO << "[MCProtocol] Sending request: " << vectorToHexString(request);

    // Send request frame to PLC
    if (send(m_socket_fd, (char*)request.data(), request.size(), 0) == SOCKET_ERROR) {
        LOG_ERROR << "[MCProtocol] Failed to send write request";
        throw std::runtime_error("[MCProtocol] Failed to send request");
    }

    // Receive response frame from PLC (write operations return only header)
    std::vector<uint8_t> response(2); // Response header only
    int bytes_received = recv(m_socket_fd, (char*)response.data(), response.size(), 0);

        LOG_INFO << "[MCProtocol] Received response: " << vectorToHexString(response);

    if (bytes_received <= 0) {
        LOG_ERROR << "[MCProtocol] Failed to receive response";
        throw std::runtime_error("[MCProtocol] Failed to receive response");
    }

    // Validate response frame
    if (response[0] != RESP_BATCH_WRITE_WORD) {
        LOG_ERROR << "[MCProtocol] Invalid response header";
        throw std::runtime_error("[MCProtocol] Invalid response header for write operation");
    }
    
    // Check completion code (0x00 = success)
    if (response[1] != 0x00) {
        LOG_ERROR << "[MCProtocol] PLC returned error code: " << std::to_string(response[1]);
        throw std::runtime_error("[MCProtocol] PLC returned error code: " + std::to_string(response[1]));
    }

    return true;
}

std::vector<uint16_t> MCProtocol::readBits(const std::string& device, uint32_t start_addr, uint16_t word_count) {
    
    std::lock_guard<std::mutex> lock(m_socket_mutex);
    
    // Check connection status
    if (!m_is_connected) {
        LOG_ERROR << "[MCProtocol] Not connected to PLC";
        throw std::runtime_error("[MCProtocol] Not connected to PLC");
    }

    uint16_t device_code = getDeviceCode(device);
    
    // Build MC Protocol 1E frame for reading bit devices as words
    // Note: Use word read command even for bit devices to get word-formatted result
    std::vector<uint8_t> request;
    request.push_back(CMD_BATCH_READ_WORD);      // Command: Batch read in word units
    request.push_back(0xFF);                     // PC No: Fixed value 0xFF
    
    // Monitoring timer (2 bytes, little endian)
    request.push_back(m_monitoring_timer & 0xFF);           // Low byte
    request.push_back((m_monitoring_timer >> 8) & 0xFF);    // High byte
    
    // Head device number (4 bytes, little endian)
    request.push_back(start_addr & 0xFF);                 // Byte 0
    request.push_back((start_addr >> 8) & 0xFF);          // Byte 1
    request.push_back((start_addr >> 16) & 0xFF);         // Byte 2
    request.push_back((start_addr >> 24) & 0xFF);         // Byte 3
    
    // Device code (2 bytes, little endian)
    request.push_back(device_code & 0xFF);                // Low byte
    request.push_back((device_code >> 8) & 0xFF);         // High byte
    
    // Number of words (2 bytes, little endian)
    // Each word contains 16 consecutive bits
    request.push_back(word_count & 0xFF);                 // Low byte
    request.push_back((word_count >> 8) & 0xFF);          // High byte

    LOG_INFO << "[MCProtocol] Sending request: " << vectorToHexString(request);

    // Send request frame to PLC
    if (send(m_socket_fd, (char*)request.data(), request.size(), 0) == SOCKET_ERROR) {
        LOG_ERROR << "[MCProtocol] Failed to send read request";
        throw std::runtime_error("[MCProtocol] Failed to send request");
    }

    // Receive response frame from PLC
    std::vector<uint8_t> response(2 + word_count * 2);
    int bytes_received = recv(m_socket_fd, (char*)response.data(), response.size(), 0);

    LOG_INFO << "[MCProtocol] Received response: " << vectorToHexString(response);

    if (bytes_received <= 0) {
        LOG_ERROR << "[MCProtocol] Failed to receive response";
        throw std::runtime_error("[MCProtocol] Failed to receive response");
    }

    // Validate response frame
    if (response[0] != RESP_BATCH_READ_WORD) {
        LOG_ERROR << "[MCProtocol] Invalid response header";
        throw std::runtime_error("[MCProtocol] Invalid response header");
    }
    
    // Check completion code (0x00 = success)
    if (response[1] != 0x00) {
        LOG_ERROR << "[MCProtocol] PLC returned error code: " << std::to_string(response[1]);
        throw std::runtime_error("[MCProtocol] PLC returned error code: " + std::to_string(response[1]));
    }

    // Convert received bit data from little endian to host format
    std::vector<uint16_t> result;
    for (int i = 0; i < word_count; i++) {
        // Combine low and high bytes (little endian format)
        uint16_t value = response[2 + i * 2] | (response[2 + i * 2 + 1] << 8);
        result.push_back(value);
    }

    LOG_INFO << "[MCProtocol] Received data: " << vectorToHexString(result);

    return result;
}

uint16_t MCProtocol::readSingleWord(const std::string& device, uint32_t addr) {
    auto result = readWords(device, addr, 1);
    return result[0];
}

bool MCProtocol::writeSingleWord(const std::string& device, uint32_t addr, uint16_t value) {
    return writeWords(device, addr, {value});
}

void MCProtocol::setMonitoringTimer(uint16_t timer_250ms) {
    m_monitoring_timer = timer_250ms;
}

bool MCProtocol::isConnected() const {
    return m_is_connected;
}

uint16_t MCProtocol::getDeviceCode(const std::string& device) {
    /**
     * Convert device name to MC Protocol device code
     * Device codes are based on ASCII values in little endian format
     * 
     * Format: [ASCII_CODE][0x20]
     * Examples:
     * - "D" -> 'D'(0x44) + 0x20 -> 0x4420
     * - "Y" -> 'Y'(0x59) + 0x20 -> 0x5920
     */
    
    if (device == "D") return 0x4420;  // D register (Data register)
    if (device == "Y") return 0x5920;  // Y output (Output relay)
    if (device == "X") return 0x5820;  // X input (Input relay)
    if (device == "M") return 0x4D20;  // M relay (Internal relay)
    if (device == "R") return 0x5220;  // R register (File register)
    if (device == "Z") return 0x5A20;  // Z index register
    if (device == "T") return 0x5420;  // Timer
    if (device == "C") return 0x4320;  // Counter
    
    // Unsupported device type
    LOG_ERROR << "[MCProtocol] Unsupported device type: " << device;
    throw std::runtime_error("[MCProtocol] Unsupported device type: " + device);
}
/**
 * @brief Read individual bits from PLC
 * @param device Device type ("Y", "X", "M", "S", "T", "C")
 * @param start_addr Starting bit address (e.g., 100 for M100)
 * @param bit_count Number of bits to read (max 256 for bit units)
 * @return Vector of boolean values representing bit states
 * @throw std::runtime_error if not connected or operation fails
 * 
 * @example
 * @code
 * Example 1: Read individual bits
 * std::cout << "\n=== Reading M100-M109 (10 bits) ===" << std::endl;
 * auto bits = plc.readBitUnits("M", 100, 10);
 * for (int i = 0; i < bits.size(); i++) {
 *     std::cout << "M" << (100 + i) << " = " << (bits[i] ? "ON" : "OFF") << std::endl;
 * }
 * @endcode
 */
std::vector<bool> MCProtocol::readBitUnits(const std::string& device, uint32_t start_addr, uint16_t bit_count) {
    
    std::lock_guard<std::mutex> lock(m_socket_mutex);
    
    if (!m_is_connected) {
        LOG_ERROR << "[MCProtocol] Read bits failed: Not connected to PLC";
        throw std::runtime_error("[MCProtocol] Not connected to PLC");
    }

    if (bit_count > 256) {
        LOG_ERROR << "[MCProtocol] Bit count exceeds maximum (256)";
        throw std::runtime_error("[MCProtocol] Bit count exceeds maximum of 256");
    }

    uint16_t device_code = getDeviceCode(device);
    
    // Build MC Protocol 1E frame for reading bits in 1-point units (Command: 00H)
    std::vector<uint8_t> request;
    request.push_back(CMD_BATCH_READ_BIT);    // Command: 00H for bit units
    request.push_back(0xFF);                         // PC No: Fixed value 0xFF
    
    // Monitoring timer (2 bytes, little endian)
    request.push_back(m_monitoring_timer & 0xFF);
    request.push_back((m_monitoring_timer >> 8) & 0xFF);
    
    // Head device number (4 bytes, little endian)
    request.push_back(start_addr & 0xFF);
    request.push_back((start_addr >> 8) & 0xFF);
    request.push_back((start_addr >> 16) & 0xFF);
    request.push_back((start_addr >> 24) & 0xFF);
    
    // Device code (2 bytes, little endian)
    request.push_back(device_code & 0xFF);
    request.push_back((device_code >> 8) & 0xFF);
    
    // Number of device points - Use "00H" for 256 points as per documentation
    if (bit_count == 256) {
        request.push_back(0x00);  // Special case: 00H means 256 points
        request.push_back(0x00);
    } else {
        request.push_back(bit_count & 0xFF);
        request.push_back((bit_count >> 8) & 0xFF);
    }

    LOG_INFO << "[MCProtocol] Sending bit read request: " << vectorToHexString(request);

    // Send request
    if (send(m_socket_fd, (char*)request.data(), request.size(), 0) == SOCKET_ERROR) {
        LOG_ERROR << "[MCProtocol] Failed to send read bits request";
        throw std::runtime_error("[MCProtocol] Failed to send request");
    }

    // Calculate expected response size
    // Response: Subheader(1) + Complete code(1) + Data
    // For odd number of bits, dummy data (0H) is added
    int data_bytes = (bit_count + 1) / 2;  // Round up for odd numbers
    std::vector<uint8_t> response(2 + data_bytes);
    
    int bytes_received = recv(m_socket_fd, (char*)response.data(), response.size(), 0);

    LOG_INFO << "[MCProtocol] Received bit response: " << vectorToHexString(response);

    if (bytes_received <= 0) {
        LOG_ERROR << "[MCProtocol] Failed to receive response";
        throw std::runtime_error("[MCProtocol] Failed to receive response");
    }

    // Validate response
    if (response[0] != RESP_BATCH_READ_BIT) {
        LOG_ERROR << "[MCProtocol] Invalid response header: 0x" << std::hex << (int)response[0];
        throw std::runtime_error("[MCProtocol] Invalid response header");
    }
    
    if (response[1] != 0x00) {
        LOG_ERROR << "[MCProtocol] PLC returned error code: 0x" << std::hex << (int)response[1];
        throw std::runtime_error("[MCProtocol] PLC error code: " + std::to_string(response[1]));
    }

    // Parse bit data - each byte contains 2 x 4-bit values
    std::vector<bool> result;
    for (int i = 0; i < bit_count; i++) {
        int byte_idx = 2 + (i / 2);  // Skip header
        int nibble = (i % 2 == 0) ? (response[byte_idx] & 0x0F) : ((response[byte_idx] >> 4) & 0x0F);
        result.push_back(nibble == 1);
    }

    return result;
}

/**
 * @brief Write individual bits to PLC
 * @param device Device type ("Y", "M", "S", "T", "C")
 * @param start_addr Starting bit address
 * @param values Vector of boolean values to write
 * @return true if write successful
 * @throw std::runtime_error if not connected or operation fails
 * 
 * @example
 * @code
 * // Set M50=ON, M51=OFF, M52=ON
 * std::vector<bool> bits = {true, false, true};
 * plc.writeBitUnits("M", 50, bits);
 * @endcode
 */
bool MCProtocol::writeBitUnits(const std::string& device, uint32_t start_addr, const std::vector<bool>& values) {
    
    std::lock_guard<std::mutex> lock(m_socket_mutex);
    
    if (!m_is_connected) {
        LOG_ERROR << "[MCProtocol] Write bits failed: Not connected to PLC";
        throw std::runtime_error("[MCProtocol] Not connected to PLC");
    }

    uint16_t bit_count = values.size();
    if (bit_count > 160) {
        LOG_ERROR << "[MCProtocol] Bit count exceeds maximum for write (160)";
        throw std::runtime_error("[MCProtocol] Bit count exceeds maximum of 160");
    }

    uint16_t device_code = getDeviceCode(device);
    
    // Build MC Protocol 1E frame for writing bits in 1-point units (Command: 02H)
    std::vector<uint8_t> request;
    request.push_back(CMD_BATCH_WRITE_BIT);   // Command: 02H for bit write
    request.push_back(0xFF);                         // PC No: Fixed value 0xFF
    
    // Monitoring timer (2 bytes, little endian)
    request.push_back(m_monitoring_timer & 0xFF);
    request.push_back((m_monitoring_timer >> 8) & 0xFF);
    
    // Head device number (4 bytes, little endian)
    request.push_back(start_addr & 0xFF);
    request.push_back((start_addr >> 8) & 0xFF);
    request.push_back((start_addr >> 16) & 0xFF);
    request.push_back((start_addr >> 24) & 0xFF);
    
    // Device code (2 bytes, little endian)
    request.push_back(device_code & 0xFF);
    request.push_back((device_code >> 8) & 0xFF);
    
    // Number of device points
    request.push_back(bit_count & 0xFF);
    request.push_back((bit_count >> 8) & 0xFF);
    
    // Pack bit data - 2 bits per byte (4-bit nibbles)
    for (size_t i = 0; i < values.size(); i += 2) {
        uint8_t byte_val = 0;
        
        // First nibble (low 4 bits)
        if (i < values.size()) {
            byte_val |= (values[i] ? 0x01 : 0x00);
        }
        
        // Second nibble (high 4 bits)
        if (i + 1 < values.size()) {
            byte_val |= (values[i + 1] ? 0x10 : 0x00);
        }
        
        request.push_back(byte_val);
    }
    
    // Add dummy data (0H) if odd number of bits
    if (bit_count % 2 == 1) {
        // Already handled in the loop above
    }

    LOG_INFO << "[MCProtocol] Sending bit write request: " << vectorToHexString(request);

    // Send request
    if (send(m_socket_fd, (char*)request.data(), request.size(), 0) == SOCKET_ERROR) {
        LOG_ERROR << "[MCProtocol] Failed to send write bits request";
        throw std::runtime_error("[MCProtocol] Failed to send request");
    }

    // Receive response (only header for write operations)
    std::vector<uint8_t> response(2);
    int bytes_received = recv(m_socket_fd, (char*)response.data(), response.size(), 0);

    LOG_INFO << "[MCProtocol] Received write response: " << vectorToHexString(response);

    if (bytes_received <= 0) {
        LOG_ERROR << "[MCProtocol] Failed to receive response";
        throw std::runtime_error("[MCProtocol] Failed to receive response");
    }

    // Validate response
    if (response[0] != RESP_BATCH_WRITE_BIT) {
        LOG_ERROR << "[MCProtocol] Invalid response header";
        throw std::runtime_error("[MCProtocol] Invalid response header");
    }
    
    if (response[1] != 0x00) {
        LOG_ERROR << "[MCProtocol] PLC returned error code: " << std::to_string(response[1]);
        throw std::runtime_error("[MCProtocol] PLC error code: " + std::to_string(response[1]));
    }

    return true;
}

  /**
 * @brief Start monitoring a word register for changes
 * @param device Device type ("D", "R", "Z", "T", "C")
 * @param addr Register address to monitor
 * @param callback Function to call when value changes
 * @param poll_interval_ms Polling interval in milliseconds (default: 100ms)
 * @return Monitor ID for stopping monitoring later
 * 
 * @example
 * @code
 * auto id = plc.monitorRegister("D", 100, 
 *     [](const std::string& dev, uint32_t addr, uint16_t old_val, uint16_t new_val) {
 *         std::cout << dev << addr << " changed from " << old_val << " to " << new_val << std::endl;
 *     }, 100);
 * @endcode
 */
int MCProtocol::monitorRegister(const std::string& device, uint32_t addr, 
                                RegisterCallback callback, int poll_interval_ms) {
    std::lock_guard<std::mutex> lock(m_monitor_mutex);
    
    int monitor_id = m_next_monitor_id++;
    auto& stop_flag = m_monitor_flags[monitor_id];
    stop_flag = false;
    
    // Create monitoring thread
    m_monitor_threads[monitor_id] = std::make_unique<std::thread>(
        [this, device, addr, callback, poll_interval_ms, &stop_flag]() {
            LOG_INFO << "[MCProtocol][Monitor-" << device << addr << "] Started monitoring";
            
            uint16_t last_value = 0;
            bool first_read = true;
            
            while (!stop_flag) {
                try {
                    uint16_t current_value = readSingleWord(device, addr);
                    
                    if (!first_read && current_value != last_value) {
                        // Value changed - invoke callback
                        callback(device, addr, last_value, current_value);
                        LOG_INFO << "[MCProtocol][Monitor-" << device << addr << "] Value changed: " 
                                << last_value << " -> " << current_value;
                    }
                    
                    last_value = current_value;
                    first_read = false;
                    
                } catch (const std::exception& e) {
                    LOG_ERROR << "[MCProtocol][Monitor-" << device << addr << "] Error: " << e.what();
                }
                
                // Sleep for polling interval
                std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
            }
            
            LOG_INFO << "[MCProtocol][Monitor-" << device << addr << "] Stopped monitoring";
        }
    );
    
    return monitor_id;
}

/**
 * @brief Start monitoring a bit for changes
 * @param device Device type ("Y", "X", "M", "S", "T", "C")
 * @param addr Bit address to monitor
 * @param callback Function to call when bit state changes
 * @param poll_interval_ms Polling interval in milliseconds (default: 100ms)
 * @return Monitor ID for stopping monitoring later
 * Example 4: Monitor bit M60 for state changes
 * std::cout << "\n=== Monitoring M60 ===" << std::endl;
 * auto bit_monitor_id = plc.monitorBit("M", 60,
 *     [](const std::string& dev, uint32_t addr, bool old_state, bool new_state) {
 *         std::cout << "[BIT CALLBACK] " << dev << addr 
 *                  << " changed from " << (old_state ? "ON" : "OFF")
 *                  << " to " << (new_state ? "ON" : "OFF") << std::endl;
 *     }, 50);  // Poll every 50ms
 */
int MCProtocol::monitorBit(const std::string& device, uint32_t addr, 
                           BitCallback callback, int poll_interval_ms) {
    std::lock_guard<std::mutex> lock(m_monitor_mutex);
    
    int monitor_id = m_next_monitor_id++;
    auto& stop_flag = m_monitor_flags[monitor_id];
    stop_flag = false;
    
    // Create monitoring thread for bit
    m_monitor_threads[monitor_id] = std::make_unique<std::thread>(
        [this, device, addr, callback, poll_interval_ms, &stop_flag]() {
            LOG_INFO << "[MCProtocol][BitMonitor-" << device << addr << "] Started monitoring";
            
            bool last_value = false;
            bool first_read = true;
            
            while (!stop_flag) {
                try {
                    // Read single bit
                    auto bits = readBitUnits(device, addr, 1);
                    bool current_value = bits[0];
                    
                    if (!first_read && current_value != last_value) {
                        // Bit state changed - invoke callback
                        callback(device, addr, last_value, current_value);
                        LOG_INFO << "[MCProtocol][BitMonitor-" << device << addr << "] State changed: " 
                                << (last_value ? "ON" : "OFF") << " -> " 
                                << (current_value ? "ON" : "OFF");
                    }
                    
                    last_value = current_value;
                    first_read = false;
                    
                } catch (const std::exception& e) {
                    LOG_ERROR << "[MCProtocol][BitMonitor-" << device << addr << "] Error: " << e.what();
                }
                
                // Sleep for polling interval
                std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
            }
            
            LOG_INFO << "[MCProtocol][BitMonitor-" << device << addr << "] Stopped monitoring";
        }
    );
    
    return monitor_id;
}

void MCProtocol::stopMonitor(int monitor_id) {
    std::lock_guard<std::mutex> lock(m_monitor_mutex);
    
    auto flag_it = m_monitor_flags.find(monitor_id);
    if (flag_it != m_monitor_flags.end()) {
        flag_it->second = true;  // Set stop flag
    }
    
    auto thread_it = m_monitor_threads.find(monitor_id);
    if (thread_it != m_monitor_threads.end()) {
        if (thread_it->second->joinable()) {
            thread_it->second->join();
        }
        m_monitor_threads.erase(thread_it);
    }
    
    m_monitor_flags.erase(monitor_id);
    LOG_INFO << "[MCProtocol][Monitor] Stopped monitor ID: " << monitor_id;
}

void MCProtocol::stopAllMonitors() {
    std::lock_guard<std::mutex> lock(m_monitor_mutex);
    
    // Set all stop flags
    for (auto& [id, flag] : m_monitor_flags) {
        flag = true;
    }
    
    // Join all threads
    for (auto& [id, thread] : m_monitor_threads) {
        if (thread->joinable()) {
            thread->join();
        }
    }
    
    m_monitor_threads.clear();
    m_monitor_flags.clear();
    LOG_INFO << "[MCProtocol][Monitor] Stopped all monitors";
}