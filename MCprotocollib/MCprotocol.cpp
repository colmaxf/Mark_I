#include "MCprotocol.h"
#include <cstring>
#include <iomanip>

/// @name MC Protocol Command Codes
/// @{
static const uint8_t CMD_BATCH_READ_WORD = 0x01;   ///< Batch read in word units
static const uint8_t CMD_BATCH_WRITE_WORD = 0x03;  ///< Batch write in word units
static const uint8_t CMD_BATCH_READ_BIT = 0x02;    ///< Batch read in bit units
static const uint8_t CMD_BATCH_WRITE_BIT = 0x04;   ///< Batch write in bit units
/// @}

/// @name MC Protocol Response Codes
/// @{
static const uint8_t RESP_BATCH_READ_WORD = 0x81;  ///< Response for batch read word
static const uint8_t RESP_BATCH_WRITE_WORD = 0x83; ///< Response for batch write word
static const uint8_t RESP_BATCH_READ_BIT = 0x82;   ///< Response for batch read bit
static const uint8_t RESP_BATCH_WRITE_BIT = 0x84;  ///< Response for batch write bit
/// @}

MCProtocol::MCProtocol(const std::string& ip, int port, uint16_t timer_250ms)
    : m_plc_ip(ip), m_plc_port(port), m_is_connected(false), m_monitoring_timer(timer_250ms) {
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
#ifdef ENABLE_LOG
        LOG_ERROR << "Failed to create socket";
#else
        std::cerr << "Failed to create socket" << std::endl;
#endif
        return false;
    }

    // Prepare server address structure
    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(m_plc_port);
    
    // Convert IP address from string to binary
    if (inet_pton(AF_INET, m_plc_ip.c_str(), &server_addr.sin_addr) <= 0) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Invalid IP address: " << m_plc_ip;
#else
        std::cerr << "Invalid IP address: " << m_plc_ip << std::endl;
#endif
        closesocket(m_socket_fd);
        m_socket_fd = INVALID_SOCKET;
        return false;
    }

    // Establish connection to PLC
    if (::connect(m_socket_fd, (sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Failed to connect to PLC at " << m_plc_ip << ":" << m_plc_port;
#else
        std::cerr << "Failed to connect to PLC at " << m_plc_ip << ":" << m_plc_port << std::endl;
#endif  
        closesocket(m_socket_fd);
        m_socket_fd = INVALID_SOCKET;
        return false;
    }

    m_is_connected = true;
#ifdef ENABLE_LOG
    LOG_INFO << "Connected to PLC at " << m_plc_ip << ":" << m_plc_port;
#else
    std::cout << "Connected to PLC at " << m_plc_ip << ":" << m_plc_port << std::endl;
#endif
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
    // Check connection status
    if (!m_is_connected) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Not connected to PLC";
#else
        throw std::runtime_error("Not connected to PLC");
#endif
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
#ifdef ENABLE_LOG
    LOG_INFO << "[MCProtocol] Sending request: " << vectorToHexString(request);
#endif

    // Send request frame to PLC
    if (send(m_socket_fd, (char*)request.data(), request.size(), 0) == SOCKET_ERROR) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Failed to send request";
#else
        throw std::runtime_error("Failed to send request");
#endif
    }

    // Receive response frame from PLC
    std::vector<uint8_t> response(2 + count * 2); // Response header (2 bytes) + data
    int bytes_received = recv(m_socket_fd, (char*)response.data(), response.size(), 0);

#ifdef ENABLE_LOG
        LOG_INFO << "[MCProtocol] Received response: " << vectorToHexString(response);
#endif

    if (bytes_received <= 0) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Failed to receive response";
#else
        throw std::runtime_error("Failed to receive response");
#endif
    }

    // Validate response frame
    if (response[0] != RESP_BATCH_READ_WORD) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Invalid response header";
#else
        throw std::runtime_error("Invalid response header");
#endif
    }
    
    // Check completion code (0x00 = success)
    if (response[1] != 0x00) {
#ifdef ENABLE_LOG
        LOG_ERROR << "PLC returned error code: " << std::to_string(response[1]);
#else
        throw std::runtime_error("PLC returned error code: " + std::to_string(response[1]));
#endif
    }

    // Convert received data from little endian to host format
    std::vector<uint16_t> result;
    for (int i = 0; i < count; i++) {
        // Combine low and high bytes (little endian format)
        uint16_t value = response[2 + i * 2] | (response[2 + i * 2 + 1] << 8);
        result.push_back(value);
    }
#ifdef ENABLE_LOG
    LOG_INFO << "[MCProtocol] Received data: " << vectorToHexString(result);
#endif

    return result;
}

bool MCProtocol::writeWords(const std::string& device, uint32_t start_addr, const std::vector<uint16_t>& values) {
    // Check connection status
    if (!m_is_connected) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Not connected to PLC";
#else
        throw std::runtime_error("Not connected to PLC");
#endif
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
#ifdef ENABLE_LOG
    LOG_INFO << "[MCProtocol] Sending request: " << vectorToHexString(request);
#endif

    // Send request frame to PLC
    if (send(m_socket_fd, (char*)request.data(), request.size(), 0) == SOCKET_ERROR) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Failed to send request";
#else
        throw std::runtime_error("Failed to send request");
#endif
    }

    // Receive response frame from PLC (write operations return only header)
    std::vector<uint8_t> response(2); // Response header only
    int bytes_received = recv(m_socket_fd, (char*)response.data(), response.size(), 0);

#ifdef ENABLE_LOG
        LOG_INFO << "[MCProtocol] Received response: " << vectorToHexString(response);
#endif

    if (bytes_received <= 0) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Failed to receive response";
#else
        throw std::runtime_error("Failed to receive response");
#endif
    }

    // Validate response frame
    if (response[0] != RESP_BATCH_WRITE_WORD) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Invalid response header";
#else
        throw std::runtime_error("Invalid response header");
#endif
    }
    
    // Check completion code (0x00 = success)
    if (response[1] != 0x00) {
#ifdef ENABLE_LOG
        LOG_ERROR << "PLC returned error code: " << std::to_string(response[1]);
#else
        throw std::runtime_error("PLC returned error code: " + std::to_string(response[1]));
#endif
    }


    return true;
}

std::vector<uint16_t> MCProtocol::readBits(const std::string& device, uint32_t start_addr, uint16_t word_count) {
    // Check connection status
    if (!m_is_connected) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Not connected to PLC";
#else
        throw std::runtime_error("Not connected to PLC");
#endif
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

#ifdef ENABLE_LOG
    LOG_INFO << "[MCProtocol] Sending request: " << vectorToHexString(request);
#endif

    // Send request frame to PLC
    if (send(m_socket_fd, (char*)request.data(), request.size(), 0) == SOCKET_ERROR) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Failed to send request";
#else
        throw std::runtime_error("Failed to send request");
#endif
    }

    // Receive response frame from PLC
    std::vector<uint8_t> response(2 + word_count * 2);
    int bytes_received = recv(m_socket_fd, (char*)response.data(), response.size(), 0);

#ifdef ENABLE_LOG
    LOG_INFO << "[MCProtocol] Received response: " << vectorToHexString(response);
#endif

    if (bytes_received <= 0) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Failed to receive response";
#else
        throw std::runtime_error("Failed to receive response");
#endif
    }

    // Validate response frame
    if (response[0] != RESP_BATCH_READ_WORD) {
#ifdef ENABLE_LOG
        LOG_ERROR << "Invalid response header";
#else
        throw std::runtime_error("Invalid response header");
#endif
    }
    
    // Check completion code (0x00 = success)
    if (response[1] != 0x00) {
#ifdef ENABLE_LOG
        LOG_ERROR << "PLC returned error code: " << std::to_string(response[1]);
#else
        throw std::runtime_error("PLC returned error code: " + std::to_string(response[1]));
#endif
    }

    // Convert received bit data from little endian to host format
    std::vector<uint16_t> result;
    for (int i = 0; i < word_count; i++) {
        // Combine low and high bytes (little endian format)
        uint16_t value = response[2 + i * 2] | (response[2 + i * 2 + 1] << 8);
        result.push_back(value);
    }

#ifdef ENABLE_LOG
    LOG_INFO << "[MCProtocol] Received data: " << vectorToHexString(result);
#endif

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
#ifdef ENABLE_LOG
    LOG_ERROR << "Unsupported device type: " << device;
#else
    throw std::runtime_error("Unsupported device type: " + device);
#endif
}