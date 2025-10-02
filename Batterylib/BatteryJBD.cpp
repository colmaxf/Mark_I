/**
 * @file BatteryJBD.cpp
 * @brief Triển khai lớp JBDBMSSingleton để giao tiếp với JBD BMS.
 * @details Lớp này sử dụng mẫu thiết kế Singleton để đảm bảo chỉ có một thực thể duy nhất
 *          quản lý kết nối và dữ liệu từ BMS, đồng thời đảm bảo an toàn luồng (thread-safe).
 */
#include "BatteryJBD.h"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

/// @brief Con trỏ duy nhất đến thực thể của lớp Singleton.
std::unique_ptr<JBDBMSSingleton> JBDBMSSingleton::instance = nullptr;
/// @brief Mutex để bảo vệ việc khởi tạo thực thể Singleton.
std::mutex JBDBMSSingleton::instanceMutex;

/**
 * @brief Lấy thực thể duy nhất của lớp JBDBMSSingleton.
 * @details Nếu thực thể chưa được tạo, nó sẽ được tạo mới.
 *          Hàm này sử dụng mutex để đảm bảo an toàn luồng khi tạo thực thể.
 * @param port Đường dẫn đến cổng serial (ví dụ: "/dev/ttyAMA0").
 * @return Tham chiếu đến thực thể duy nhất của JBDBMSSingleton.
 */
JBDBMSSingleton& JBDBMSSingleton::getInstance(const std::string& port) {
    std::lock_guard<std::mutex> lock(instanceMutex);
    if (!instance) {
        instance = std::unique_ptr<JBDBMSSingleton>(new JBDBMSSingleton(port));
    }
    return *instance;
}

/**
 * @brief Hàm hủy của lớp JBDBMSSingleton.
 * @details Tự động đóng kết nối serial khi đối tượng bị hủy.
 */
JBDBMSSingleton::~JBDBMSSingleton() {
    closeSerial();
}

/**
 * @brief Khởi tạo và cấu hình kết nối cổng serial đến BMS.
 * @return `true` nếu kết nối và cấu hình thành công, `false` nếu thất bại.
 */
bool JBDBMSSingleton::initialize() {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    if (isConnected) {
        return true;
    }
    
    // Open serial port
    serialPort = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialPort < 0) {
       LOG_WARNING << "Cannot open serial port: " << portName ;
        return false;
    }
    
    // Configure serial port
    struct termios options;
    tcgetattr(serialPort, &options);
    
    // Set baud rate to 9600
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    
    // 8N1 configuration
    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;   // Clear size bits
    options.c_cflag |= CS8;      // 8 data bits
    options.c_cflag |= CREAD | CLOCAL;  // Enable receiver, ignore modem control lines
    
    // Disable hardware flow control
    options.c_cflag &= ~CRTSCTS;
    
    // Raw input
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Raw output
    options.c_oflag &= ~OPOST;
    
    // No software flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // Timeout settings
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;  // 1 second timeout
    
    tcsetattr(serialPort, TCSANOW, &options);
    
    // Clear input/output buffers
    tcflush(serialPort, TCIOFLUSH);
    
    isConnected = true;
    LOG_INFO << "Connected to BMS via " << portName ;
    return true;
}

/**
 * @brief Đóng kết nối cổng serial.
 * @details Hàm này an toàn để gọi ngay cả khi cổng đã được đóng.
 */
void JBDBMSSingleton::closeSerial() {
    std::lock_guard<std::mutex> lock(dataMutex);
    if (serialPort >= 0) {
        close(serialPort);
        serialPort = -1;
        isConnected = false;
    }
}

/**
 * @brief Cập nhật tất cả dữ liệu từ BMS.
 * @details Gửi các lệnh cần thiết để đọc thông tin cơ bản và điện áp các cell, sau đó cập nhật cấu trúc dữ liệu nội bộ.
 * @return `true` nếu cập nhật thành công, `false` nếu có lỗi trong quá trình giao tiếp.
 */
bool JBDBMSSingleton::updateBatteryData() {
    if (!isConnected || serialPort < 0) {
        LOG_WARNING << "BMS is not connected" ;
        return false;
    }
    
    // Initialize communication
    if (!writeRequestStart() || !writeRequestEnd()) {
        return false;
    }
    if (!writeRequestStart() || !writeRequestEnd()) {
        return false;
    }
    
    // Read cell voltages
    if (!readCellVoltages()) {
        LOG_WARNING << "Failed to read cell voltages" ;
        return false;
    }
    
    // Read basic information
    if (!readBasicInfo()) {
        LOG_WARNING << "Failed to read basic info" ;
        return false;
    }
    
    // Update timestamp and calculate power
    std::lock_guard<std::mutex> lock(dataMutex);
    batteryData.lastUpdate = std::chrono::system_clock::now();
    batteryData.packPower = batteryData.packVoltage * batteryData.packCurrent;
    
    return true;
}

/**
 * @brief Lấy một bản sao của toàn bộ dữ liệu pin hiện tại.
 * @return Một đối tượng `BatteryData` chứa thông tin pin.
 * @note Hàm này an toàn luồng (thread-safe).
 */
BatteryData JBDBMSSingleton::getBatteryData() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return batteryData;
}

/**
 * @brief Lấy điện áp tổng của pack pin.
 * @return Điện áp pack pin (V).
 */
float JBDBMSSingleton::getPackVoltage() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return batteryData.packVoltage;
}

/**
 * @brief Lấy dòng điện hiện tại của pack pin.
 * @return Dòng điện pack pin (A). Giá trị dương là sạc, âm là xả.
 */
float JBDBMSSingleton::getPackCurrent() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return batteryData.packCurrent;
}

/**
 * @brief Lấy công suất hiện tại của pack pin.
 * @return Công suất pack pin (W).
 */
float JBDBMSSingleton::getPackPower() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return batteryData.packPower;
}

/**
 * @brief Lấy trạng thái sạc (State of Charge - SOC).
 * @return Phần trăm pin (%).
 */
int JBDBMSSingleton::getStateOfCharge() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return batteryData.stateOfCharge;
}

/**
 * @brief Lấy dung lượng còn lại.
 * @return Dung lượng còn lại (Ah).
 */
float JBDBMSSingleton::getRemainingCapacity() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return batteryData.remainingCapacity;
}

/**
 * @brief Lấy nhiệt độ từ cảm biến 1.
 * @return Nhiệt độ (°C).
 */
float JBDBMSSingleton::getTemperature1() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return batteryData.temperature1;
}

/**
 * @brief Lấy nhiệt độ từ cảm biến 2.
 * @return Nhiệt độ (°C).
 */
float JBDBMSSingleton::getTemperature2() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return batteryData.temperature2;
}

/**
 * @brief Lấy số lượng cell pin.
 * @return Số lượng cell.
 */
int JBDBMSSingleton::getNumCells() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return batteryData.numCells;
}

/**
 * @brief Lấy danh sách điện áp của tất cả các cell.
 * @return Vector chứa điện áp của các cell (V).
 */
std::vector<float> JBDBMSSingleton::getCellVoltages() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return std::vector<float>(batteryData.cellVoltages.begin(), 
                             batteryData.cellVoltages.begin() + batteryData.numCells);
}

/**
 * @brief Lấy điện áp của một cell cụ thể.
 * @param cellIndex Chỉ số của cell (bắt đầu từ 0).
 * @return Điện áp của cell (V), hoặc 0.0 nếu chỉ số không hợp lệ.
 */
float JBDBMSSingleton::getCellVoltage(int cellIndex) const {
    std::lock_guard<std::mutex> lock(dataMutex);
    if (cellIndex >= 0 && cellIndex < batteryData.numCells) {
        return batteryData.cellVoltages[cellIndex];
    }
    return 0.0f;
}

/**
 * @brief Kiểm tra trạng thái của MOSFET sạc.
 * @return `true` nếu MOSFET sạc đang BẬT, `false` nếu đang TẮT.
 */
bool JBDBMSSingleton::isChargeFetOn() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return batteryData.chargeFet;
}

/**
 * @brief Kiểm tra trạng thái của MOSFET xả.
 * @return `true` nếu MOSFET xả đang BẬT, `false` nếu đang TẮT.
 */
bool JBDBMSSingleton::isDischargeFetOn() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return batteryData.dischargeFet;
}

std::string JBDBMSSingleton::getProtectionStateText() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    if (batteryData.protectionState == BMS_STATUS_OK) {
        return "OK";
    }
    
    std::string status;
    if (batteryData.cellOvervoltage) status += "Cell OverVoltage ";
    if (batteryData.cellUndervoltage) status += "Cell UnderVoltage ";
    if (batteryData.packOvervoltage) status += "Pack OverVoltage ";
    if (batteryData.packUndervoltage) status += "Pack UnderVoltage ";
    if (batteryData.chargeOverTemp) status += "Charge OverTemp ";
    if (batteryData.chargeUnderTemp) status += "Charge UnderTemp ";
    if (batteryData.dischargeOverTemp) status += "Discharge OverTemp ";
    if (batteryData.dischargeUnderTemp) status += "Discharge UnderTemp ";
    if (batteryData.chargeOvercurrent) status += "Charge OverCurrent ";
    if (batteryData.dischargeOvercurrent) status += "Discharge OverCurrent ";
    if (batteryData.shortCircuit) status += "Short Circuit ";
    if (batteryData.afeError) status += "AFE Error ";
    if (batteryData.softLock) status += "Software Lock ";
    if (batteryData.chargeOvertime) status += "Charge Overtime ";
    if (batteryData.dischargeOvertime) status += "Discharge Overtime ";
    
    return status.empty() ? "Unknown" : status;
}

/**
 * @brief In thông tin chi tiết của pin ra console.
 * @details Bao gồm thông tin pack, trạng thái MOSFET, thông tin cell và trạng thái bảo vệ.
 */
void JBDBMSSingleton::printBatteryInfo() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    LOG_INFO << "\n=== JBD BMS Battery Information ===" ;
    LOG_INFO << std::fixed << std::setprecision(3);
    
    // Pack info
    LOG_INFO << "Pack Voltage: " << batteryData.packVoltage << "V" ;
    LOG_INFO << "Pack Current: " << batteryData.packCurrent << "A" ;
    LOG_INFO << "Pack Power: " << batteryData.packPower << "W" ;
    LOG_INFO << "Remaining Capacity: " << batteryData.remainingCapacity << "Ah" ;
    LOG_INFO << "State of Charge: " << batteryData.stateOfCharge << "%" ;
    LOG_INFO << "Cycle Count: " << batteryData.cycleCount ;
    LOG_INFO << "Temperature 1: " << batteryData.temperature1 << "°C" ;
    LOG_INFO << "Temperature 2: " << batteryData.temperature2 << "°C" ;
    
    // MOSFET status
    LOG_INFO << "Charge FET: " << (batteryData.chargeFet ? "ON" : "OFF") ;
    LOG_INFO << "Discharge FET: " << (batteryData.dischargeFet ? "ON" : "OFF") ;
    
    // Cell information
    LOG_INFO << "\n--- Cell Information ---" ;
    LOG_INFO << "Number of Cells: " << batteryData.numCells ;
    for (int i = 0; i < batteryData.numCells; i++) {
        LOG_INFO << "Cell " << (i+1) << ": " << batteryData.cellVoltages[i] << "V";
        if (batteryData.balancerStates[i]) LOG_INFO << " (Balancing)";
        LOG_INFO ;
    }
    
    LOG_INFO << "Cell Max: " << batteryData.cellMax << "V" ;
    LOG_INFO << "Cell Min: " << batteryData.cellMin << "V" ;
    LOG_INFO << "Cell Average: " << batteryData.cellAverage << "V" ;
    LOG_INFO << "Cell Difference: " << batteryData.cellDifference << "V" ;
    
    // Protection status
    LOG_INFO << "\n--- Protection Status ---" ;
    if (batteryData.afeError) LOG_INFO << "  AFE ERROR" ;
    if (batteryData.shortCircuit) LOG_INFO << "  SHORT CIRCUIT" ;
    if (batteryData.cellOvervoltage) LOG_INFO << "  Cell Overvoltage" ;
    if (batteryData.cellUndervoltage) LOG_INFO << "  Cell Undervoltage" ;
    if (batteryData.packOvervoltage) LOG_INFO << "  Pack Overvoltage" ;
    if (batteryData.packUndervoltage) LOG_INFO << "  Pack Undervoltage" ;
    if (batteryData.chargeOverTemp) LOG_INFO << "  Charge Overtemp" ;
    if (batteryData.chargeUnderTemp) LOG_INFO << "  Charge Undertemp" ;
    if (batteryData.dischargeOverTemp) LOG_INFO << "  Discharge Overtemp" ;
    if (batteryData.dischargeUnderTemp) LOG_INFO << "  Discharge Undertemp" ;
    if (batteryData.chargeOvercurrent) LOG_INFO << "  Charge Overcurrent" ;
    if (batteryData.dischargeOvercurrent) LOG_INFO << "  Discharge Overcurrent" ;
    
    if (!hasAnyProtection()) {
        if (batteryData.packCurrent > 0.1f) LOG_INFO << "✅ Status: Charging" ;
        else if (batteryData.packCurrent < -0.1f) LOG_INFO << "✅ Status: Discharging" ;
        else LOG_INFO << "✅ Status: Idle" ;
    }
    
    LOG_INFO << "=================================" ;
}

/**
 * @brief Kiểm tra xem kết nối serial có hợp lệ và đang hoạt động không.
 * @return `true` nếu kết nối hợp lệ, `false` nếu ngược lại.
 */
bool JBDBMSSingleton::isConnectionValid() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return isConnected && serialPort >= 0;
}

/**
 * @brief Chuyển đổi 2 byte (high, low) thành một số nguyên không dấu 16-bit.
 * @param high Byte cao.
 * @param low Byte thấp.
 * @return Giá trị uint16_t.
 */
uint16_t JBDBMSSingleton::convertTwoIntsToUint16(uint8_t high, uint8_t low) {
    return (static_cast<uint16_t>(high) << 8) | low;
}

/**
 * @brief Chuyển đổi 2 byte (high, low) thành một số nguyên có dấu 16-bit.
 * @param high Byte cao.
 * @param low Byte thấp.
 * @return Giá trị int16_t.
 */
int16_t JBDBMSSingleton::convertTwoIntsToInt16(uint8_t high, uint8_t low) {
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

/**
 * @brief Đảo ngược thứ tự các bit trong một byte.
 * @details Dùng để phân tích trạng thái cân bằng cell.
 * @param byte Byte cần đảo bit.
 * @return Byte đã được đảo bit.
 */
uint8_t JBDBMSSingleton::reverseBits(uint8_t byte) {
    byte = ((byte >> 1) & 0x55) | ((byte << 1) & 0xaa);
    byte = ((byte >> 2) & 0x33) | ((byte << 2) & 0xcc);
    byte = ((byte >> 4) & 0x0f) | ((byte << 4) & 0xf0);
    return byte;
}

/**
 * @brief Xóa bộ đệm đầu vào và đầu ra của cổng serial.
 * @details Chờ một khoảng thời gian ngắn trước và sau khi xóa để đảm bảo hoạt động ổn định.
 */
void JBDBMSSingleton::flushSerial() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    tcflush(serialPort, TCIOFLUSH);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

/**
 * @brief Gửi một vector byte (lệnh) đến cổng serial.
 * @param data Vector chứa dữ liệu lệnh cần gửi.
 * @return `true` nếu gửi thành công, `false` nếu thất bại.
 */
bool JBDBMSSingleton::writeCommand(const std::vector<uint8_t>& data) {
    flushSerial();
    ssize_t bytesWritten = write(serialPort, data.data(), data.size());
    return bytesWritten == static_cast<ssize_t>(data.size());
}

/**
 * @brief Gửi lệnh khởi tạo giao tiếp (bắt đầu).
 */
bool JBDBMSSingleton::writeRequestStart() {
    std::vector<uint8_t> data = {0xDD, 0x5A, 0x00, 0x02, 0x56, 0x78, 0xFF, 0x30, 0x77};
    return writeCommand(data);
}

/**
 * @brief Gửi lệnh khởi tạo giao tiếp (kết thúc).
 */
bool JBDBMSSingleton::writeRequestEnd() {
    std::vector<uint8_t> data = {0xDD, 0x5A, 0x01, 0x02, 0x00, 0x00, 0xFF, 0xFD, 0x77};
    return writeCommand(data);
}

bool JBDBMSSingleton::readCellVoltages() {
    std::vector<uint8_t> command = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
    
    // Gửi lệnh yêu cầu đọc điện áp các cell.
    if (!writeCommand(command)) return false;
    
    std::vector<uint8_t> response;
    if (!getBMSResponse(response)) return false;
    
    if (response.size() < 4) return false;
    
    // Byte thứ 3 chứa độ dài của dữ liệu cell.
    int dataLength = response[3];
    // Mỗi cell được biểu diễn bằng 2 byte, nên số cell = độ dài / 2.
    batteryData.numCells = dataLength / 2;
    
    if (response.size() < static_cast<size_t>(4 + dataLength)) return false;
    
    // Reset cell statistics
    // Đặt giá trị min ban đầu cao và max ban đầu thấp để so sánh.
    batteryData.cellMax = 0.0f;
    batteryData.cellMin = 5.0f;
    batteryData.cellAverage = 0.0f;
    
    for (int i = 0; i < batteryData.numCells; i++) {
        int dataIndex = 4 + (i * 2);
        uint16_t cellVoltageRaw = (response[dataIndex] << 8) | response[dataIndex + 1];
        // Chuyển đổi giá trị thô (mV) sang Volt.
        batteryData.cellVoltages[i] = cellVoltageRaw / 1000.0f;
        
        batteryData.cellAverage += batteryData.cellVoltages[i];
        // Cập nhật giá trị max/min.
        if (batteryData.cellVoltages[i] > batteryData.cellMax) {
            batteryData.cellMax = batteryData.cellVoltages[i];
        }
        if (batteryData.cellVoltages[i] < batteryData.cellMin) {
            batteryData.cellMin = batteryData.cellVoltages[i];
        }
    }
    
    if (batteryData.numCells > 0) {
        // Tính toán giá trị trung bình và chênh lệch.
        batteryData.cellAverage /= batteryData.numCells;
        batteryData.cellDifference = batteryData.cellMax - batteryData.cellMin;
    }
    
    return true;
}

/**
 * @brief Đọc và phân tích thông tin cơ bản từ BMS.
 * @details Bao gồm điện áp, dòng điện, SOC, nhiệt độ, trạng thái bảo vệ, v.v.
 * @return `true` nếu đọc và phân tích thành công, `false` nếu thất bại.
 */
bool JBDBMSSingleton::readBasicInfo() {
    std::vector<uint8_t> command = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77}; // Lệnh đọc thông tin cơ bản
    if (!writeCommand(command)) return false;
    
    std::vector<uint8_t> response;
    if (!getBMSResponse(response)) return false;
    
    if (response.size() < 27) return false;
    
    // Pack voltage (bytes 4-5)
    // Giá trị thô là 100 * Volt.
    uint16_t packVoltageRaw = (response[4] << 8) | response[5];
    batteryData.packVoltage = packVoltageRaw / 100.0f;
    
    // Pack current (bytes 6-7) - signed
    // Giá trị thô là 100 * Ampe.
    int16_t packCurrentRaw = (response[6] << 8) | response[7];
    batteryData.packCurrent = packCurrentRaw / 100.0f;
    
    // Remaining capacity (bytes 8-9)
    // Giá trị thô là 100 * Ah.
    uint16_t remainingCapacityRaw = (response[8] << 8) | response[9];
    batteryData.remainingCapacity = remainingCapacityRaw / 100.0f;
    
    // Cycle count (bytes 12-13)
    // Số chu kỳ sạc/xả.
    if (response.size() > 13) {
        batteryData.cycleCount = (response[12] << 8) | response[13];
    }
    
    // Balance states (byte 17)
    // Trạng thái cân bằng của các cell, mỗi bit tương ứng một cell.
    if (response.size() > 17) {
        uint8_t balanceCode = reverseBits(response[17]);
        for (int i = 0; i < batteryData.numCells && i < 8; i++) {
            batteryData.balancerStates[i] = (balanceCode >> i) & 1; // Kiểm tra từng bit
        }
    }
    
    // Protection status (bytes 20-21)
    // Phân tích các bit trong 2 byte trạng thái bảo vệ.
    if (response.size() > 21) {
        batteryData.protectionState = (response[20] << 8) | response[21];
        
        batteryData.cellOvervoltage = (response[21] >> 0) & 1;
        batteryData.cellUndervoltage = (response[21] >> 1) & 1;
        batteryData.packOvervoltage = (response[21] >> 2) & 1;
        batteryData.packUndervoltage = (response[21] >> 3) & 1;
        batteryData.chargeOverTemp = (response[21] >> 4) & 1;
        batteryData.chargeUnderTemp = (response[21] >> 5) & 1;
        batteryData.dischargeOverTemp = (response[21] >> 6) & 1;
        batteryData.dischargeUnderTemp = (response[21] >> 7) & 1;
        
        batteryData.chargeOvercurrent = (response[20] >> 0) & 1;
        batteryData.dischargeOvercurrent = (response[20] >> 1) & 1;
        batteryData.shortCircuit = (response[20] >> 2) & 1;
        batteryData.afeError = (response[20] >> 3) & 1;
        batteryData.softLock = (response[20] >> 4) & 1;
    }
    
    // SOC (byte 23)
    if (response.size() > 23) {
        batteryData.stateOfCharge = response[23]; // Giá trị phần trăm trực tiếp.
    }
    
    // MOSFET status (byte 24)
    if (response.size() > 24) {
        batteryData.chargeFet = response[24] & 1; // Bit 0
        batteryData.dischargeFet = (response[24] >> 1) & 1; // Bit 1
    }
    
    // Temperature probes (bytes 27-30)
    // Giá trị thô là (Nhiệt độ °C + 273.1) * 10.
    if (response.size() > 30) {
        uint16_t temp1Raw = (response[27] << 8) | response[28];
        uint16_t temp2Raw = (response[29] << 8) | response[30];
        batteryData.temperature1 = (temp1Raw - 2731) / 10.0f;
        batteryData.temperature2 = (temp2Raw - 2731) / 10.0f;
    }
    
    return true;
}

/**
 * @brief Đọc phản hồi từ BMS qua cổng serial.
 * @param[out] response Vector để lưu trữ dữ liệu phản hồi.
 * @return `true` nếu đọc được dữ liệu, `false` nếu không có dữ liệu hoặc có lỗi.
 */
bool JBDBMSSingleton::getBMSResponse(std::vector<uint8_t>& response) {
    response.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    uint8_t buffer[256];
    ssize_t bytesRead = read(serialPort, buffer, sizeof(buffer));
    
    if (bytesRead <= 0) {
        return false;
    }
    
    response.assign(buffer, buffer + bytesRead);
    return true;
}

/**
 * @brief Kiểm tra xem có bất kỳ trạng thái bảo vệ nào đang được kích hoạt không.
 * @return `true` nếu có ít nhất một lỗi bảo vệ, `false` nếu không có.
 */
bool JBDBMSSingleton::hasAnyProtection() const {
    return batteryData.cellOvervoltage || batteryData.cellUndervoltage || 
           batteryData.packOvervoltage || batteryData.packUndervoltage ||
           batteryData.chargeOverTemp || batteryData.chargeUnderTemp || 
           batteryData.dischargeOverTemp || batteryData.dischargeUnderTemp ||
           batteryData.chargeOvercurrent || batteryData.dischargeOvercurrent || 
           batteryData.shortCircuit || batteryData.afeError;
}

/**
 * @brief Hàm ví dụ để minh họa cách sử dụng lớp JBDBMSSingleton.
 */
void demonstrateUsage() {
    // Get singleton instance
    JBDBMSSingleton& bms = JBDBMSSingleton::getInstance(BATTERY_BMS_SERIAL_PORT);
    
    // Initialize connection
    if (!bms.initialize()) {
        LOG_WARNING << "Không thể khởi tạo kết nối BMS" ;
        return;
    }
    
    LOG_INFO << "JBD BMS Singleton Reader started." ;
    
    // Read data in loop
    for (int i = 0; i < 5; i++) {
        if (bms.updateBatteryData()) {
            // Print all battery information
            bms.printBatteryInfo();
            
            // Or get specific data
            LOG_INFO << "\n--- Quick Status ---" ;
            LOG_INFO << "Voltage: " << bms.getPackVoltage() << "V" ;
            LOG_INFO << "Current: " << bms.getPackCurrent() << "A" ;
            LOG_INFO << "SOC: " << bms.getStateOfCharge() << "%" ;
            LOG_INFO << "Temperature: " << bms.getTemperature1() << "°C" ;
            LOG_INFO << "Protection: " << bms.getProtectionStateText() ;
        } else {
            LOG_WARNING << "Không thể đọc dữ liệu BMS" ;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
}