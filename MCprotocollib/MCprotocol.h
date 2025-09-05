/**
 * @file MCProtocol.h
 * @brief MC Protocol Client Library for Mitsubishi PLC Communication(FX3u-ENET-ADAP FX3u-32M)
 * @author KhaiPV-Colmaxf
 * @date 2025
 * @version 1.0
 */

// mc_protocol.h
#ifndef MC_PROTOCOL_H
#define MC_PROTOCOL_H

#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "../config.h"
#ifdef ENABLE_LOG
#include "../logger/Logger.h"
#endif

/// Linux socket compatibility defines
#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket close

class MCProtocol {
public:
 /// @name Constructor & Destructor
    /// @{
    
    /**
     * @brief Constructor - Initialize MC Protocol client
     * @param ip PLC IP address (e.g., "192.168.3.5")
     * @param port PLC port number (default: 5000)
     * @param timer_250ms Monitoring timer in 250ms units (default: 10 = 2.5s)
     * @throw std::runtime_error if initialization fails
     */
    MCProtocol(const std::string& ip, int port = 5000, uint16_t timer_250ms = 10);
    
    /**
     * @brief Destructor - Clean up resources and close connection
     */
    ~MCProtocol();
    
    /// @}
    
    /// @name Connection Management
    /// @{
    
    /**
     * @brief Establish TCP connection to PLC
     * @return true if connection successful, false otherwise
     * @note Prints connection status to stdout
     */
    bool connect();
    
    /**
     * @brief Close connection to PLC
     * @note Safe to call multiple times
     */
    void disconnect();
    
    /**
     * @brief Check if currently connected to PLC
     * @return true if connected, false otherwise
     */
    bool isConnected() const;
    
    /// @}
    
    /// @name Word Register Operations
    /// @brief Operations for D, R, Z, T, C registers
    /// @{

    /**
     * @brief Read multiple word registers from PLC
     * @param device Device type ("D", "R", "Z", "T", "C")
     * @param start_addr Starting address (e.g., 100 for D100)
     * @param count Number of words to read (max depends on PLC)
     * @return Vector of 16-bit values in host byte order
     * @throw std::runtime_error if not connected, invalid device, or PLC error
     * 
     * @example
     * @code
     * // Read D100-D102 (3 words)
     * auto values = plc.readWords("D", 100, 3);
     * @endcode
     */
    std::vector<uint16_t> readWords(const std::string& device, uint32_t start_addr, uint16_t count);
    
    /**
     * @brief Write multiple word registers to PLC
     * @param device Device type ("D", "R", "Z", "T", "C")
     * @param start_addr Starting address (e.g., 100 for D100)
     * @param values Vector of 16-bit values to write
     * @return true if write successful, false otherwise
     * @throw std::runtime_error if not connected, invalid device, or PLC error
     * 
     * @example
     * @code
     * // Write to D100-D102
     * std::vector<uint16_t> data = {0x1234, 0x5678, 0x9ABC};
     * plc.writeWords("D", 100, data);
     * @endcode
     */
    bool writeWords(const std::string& device, uint32_t start_addr, const std::vector<uint16_t>& values);
    
    /// @}

    /// @name Bit Device Operations
    /// @brief Operations for Y, X, M bit devices (returned as word format)
    /// @{

    /**
     * @brief Read bit devices as word format from PLC
     * @param device Device type ("Y", "X", "M")
     * @param start_addr Starting bit address (e.g., 0 for Y000)
     * @param word_count Number of words to read (1 word = 16 bits)
     * @return Vector of 16-bit values, each containing 16 bit states
     * @throw std::runtime_error if not connected, invalid device, or PLC error
     * 
     * @note For bit devices, each word contains 16 consecutive bits.
     *       For example, reading Y000 with word_count=1 returns Y000-Y017.
     * 
     * @example
     * @code
     * // Read Y000-Y017 (1 word = 16 bits)
     * auto y_values = plc.readBits("Y", 0, 1);
     * uint16_t y_word = y_values[0];
     * bool y005_state = (y_word >> 5) & 1;  // Check Y005 state
     * @endcode
     */
    std::vector<uint16_t> readBits(const std::string& device, uint32_t start_addr, uint16_t word_count);
    
    /// @}

    /// @name Helper Functions
    /// @brief Convenience functions for single operations
    /// @{

    /**
     * @brief Read a single word register
     * @param device Device type ("D", "R", "Z", "T", "C")
     * @param addr Register address
     * @return 16-bit register value
     * @throw std::runtime_error if read fails
     */
    uint16_t readSingleWord(const std::string& device, uint32_t addr);

    /**
     * @brief Write a single word register
     * @param device Device type ("D", "R", "Z", "T", "C")
     * @param addr Register address
     * @param value 16-bit value to write
     * @return true if write successful
     * @throw std::runtime_error if write fails
     */
    bool writeSingleWord(const std::string& device, uint32_t addr, uint16_t value);
    
    /// @}

    /// @name Configuration
    /// @{

    /**
     * @brief Set monitoring timer for PLC communication
     * @param timer_250ms Timer value in 250ms units (e.g., 10 = 2.5 seconds)
     * @note Default is 10 (2.5 seconds). Increase for slower networks.
     */
    void setMonitoringTimer(uint16_t timer_250ms);
    
    /// @}

    // Module logger - TỰ ĐỘNG QUẢN LÝ APP_ID
    DECLARE_MODULE_LOGGER()
private:
    SOCKET m_socket_fd;               ///< Socket file descriptor
    std::string m_plc_ip;             ///< PLC IP address
    int m_plc_port;                   // PLC port (default 5000)
    bool m_is_connected;              ///< Connection status flag
    uint16_t m_monitoring_timer;      ///< Monitoring timer in 250ms units

    /**
     * @brief Convert device name to device code
     * @param device Device name (D, Y, X, M, R, Z, T, C)
     * @return 16-bit device code in little endian format
     * @throw std::runtime_error if device type is not supported
     */
    uint16_t getDeviceCode(const std::string& device);
    std::string vectorToHexString(const std::vector<uint8_t>& data);
    std::string vectorToHexString(const std::vector<uint16_t>& data);

};

#endif // MC_PROTOCOL_H