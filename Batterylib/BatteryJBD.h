#ifndef JBD_BMS_SINGLETON_H
#define JBD_BMS_SINGLETON_H

#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>
#include <cstdint>
#include <logger/Logger.h>
#include <../config.h>

// BMS Status Masks
#define BMS_STATUS_OK               0
#define BMS_STATUS_CELL_OVP         1
#define BMS_STATUS_CELL_UVP         2
#define BMS_STATUS_PACK_OVP         4
#define BMS_STATUS_PACK_UVP         8
#define BMS_STATUS_CHG_OTP          16
#define BMS_STATUS_CHG_UTP          32
#define BMS_STATUS_DSG_OTP          64
#define BMS_STATUS_DSG_UTP          128
#define BMS_STATUS_CHG_OCP          256
#define BMS_STATUS_DSG_OCP          512
#define BMS_STATUS_SHORT_CIRCUIT    1024
#define BMS_STATUS_AFE_ERROR        2048
#define BMS_STATUS_SOFT_LOCK        4096
#define BMS_STATUS_CHGOVERTIME      8192
#define BMS_STATUS_DSGOVERTIME      16384

/**
 * @struct BatteryData
 * @brief Cấu trúc lưu trữ tất cả thông tin đọc được từ BMS.
 */
struct BatteryData {
    // Pack information
    float packVoltage;
    float packCurrent;
    float packPower;
    float remainingCapacity;
    int stateOfCharge;
    int cycleCount;
    
    // Temperature
    float temperature1;  // External temperature
    float temperature2;  // Internal temperature
    
    // FET status
    bool chargeFet;
    bool dischargeFet;
    
    // Cell information
    int numCells;
    std::vector<float> cellVoltages;
    std::vector<bool> balancerStates;
    float cellMax;
    float cellMin;
    float cellAverage;
    float cellDifference;
    
    // Protection status
    uint16_t protectionState;
    bool cellOvervoltage;
    bool cellUndervoltage;
    bool packOvervoltage;
    bool packUndervoltage;
    bool chargeOverTemp;
    bool chargeUnderTemp;
    bool dischargeOverTemp;
    bool dischargeUnderTemp;
    bool chargeOvercurrent;
    bool dischargeOvercurrent;
    bool shortCircuit;
    bool afeError;
    bool softLock;
    bool chargeOvertime;
    bool dischargeOvertime;
    
    // Timestamp
    std::chrono::system_clock::time_point lastUpdate;
    
    /**
     * @brief Hàm khởi tạo, gọi hàm reset() để đặt các giá trị về mặc định.
     */
    BatteryData() {
        reset();
    }
    
    /**
     * @brief Đặt lại tất cả các thành viên dữ liệu về giá trị ban đầu.
     */
    void reset() {
        packVoltage = 0.0f;
        packCurrent = 0.0f;
        packPower = 0.0f;
        remainingCapacity = 0.0f;
        stateOfCharge = 0;
        cycleCount = 0;
        temperature1 = 0.0f;
        temperature2 = 0.0f;
        chargeFet = false;
        dischargeFet = false;
        numCells = 0;
        cellVoltages.clear();
        balancerStates.clear();
        cellMax = 0.0f;
        cellMin = 5.0f;
        cellAverage = 0.0f;
        cellDifference = 0.0f;
        protectionState = 0;
        cellOvervoltage = false;
        cellUndervoltage = false;
        packOvervoltage = false;
        packUndervoltage = false;
        chargeOverTemp = false;
        chargeUnderTemp = false;
        dischargeOverTemp = false;
        dischargeUnderTemp = false;
        chargeOvercurrent = false;
        dischargeOvercurrent = false;
        shortCircuit = false;
        afeError = false;
        softLock = false;
        chargeOvertime = false;
        dischargeOvertime = false;
        lastUpdate = std::chrono::system_clock::now();
    }
};

/**
 * @class JBDBMSSingleton
 * @brief Lớp quản lý giao tiếp với JBD BMS sử dụng mẫu Singleton.
 * @details Đảm bảo chỉ có một kết nối duy nhất đến BMS và cung cấp các phương thức an toàn luồng (thread-safe) để truy cập dữ liệu.
 */
class JBDBMSSingleton {
private:
    static std::unique_ptr<JBDBMSSingleton> instance;
    static std::mutex instanceMutex;
    
    std::string portName;
    BatteryData batteryData;
    mutable std::mutex dataMutex;
    int serialPort;
    bool isConnected;
    
    /**
     * @brief Hàm khởi tạo riêng tư để thực thi mẫu Singleton.
     * @param port Đường dẫn đến cổng serial.
     * @details Khởi tạo các giá trị ban đầu và cấp phát bộ nhớ cho các vector.
     */
    JBDBMSSingleton(const std::string& port = "/dev/ttyAMA0") 
        : portName(port), serialPort(-1), isConnected(false) {
        batteryData.cellVoltages.resize(20, 0.0f);
        batteryData.balancerStates.resize(20, false);
    }
    
    // Disable copy constructor and assignment operator
    JBDBMSSingleton(const JBDBMSSingleton&) = delete;
    JBDBMSSingleton& operator=(const JBDBMSSingleton&) = delete;

public:
    /**
     * @brief Lấy thực thể duy nhất của lớp JBDBMSSingleton.
     * @param port Đường dẫn đến cổng serial (ví dụ: "/dev/ttyUSB0").
     * @return Tham chiếu đến thực thể duy nhất của JBDBMSSingleton.
     */
    static JBDBMSSingleton& getInstance(const std::string& port = BATTERY_BMS_SERIAL_PORT);

    /**
     * @brief Hàm hủy, tự động đóng kết nối serial.
     */
    ~JBDBMSSingleton();
    
    /**
     * @brief Khởi tạo và cấu hình kết nối cổng serial đến BMS.
     * @return `true` nếu kết nối thành công, `false` nếu thất bại.
     */
    bool initialize();
    
    /**
     * @brief Đóng kết nối cổng serial.
     */
    void closeSerial();
    
    /**
     * @brief Cập nhật tất cả dữ liệu từ BMS.
     * @return `true` nếu cập nhật thành công, `false` nếu có lỗi.
     */
    bool updateBatteryData();
    
    /**
     * @brief Lấy một bản sao của toàn bộ dữ liệu pin hiện tại.
     * @return Một đối tượng `BatteryData` chứa thông tin pin.
     * @note Hàm này an toàn luồng (thread-safe).
     */
    BatteryData getBatteryData() const;
    
    // Individual getter methods (thread-safe)
    /**
     * @brief Lấy điện áp tổng của pack pin.
     * @return Điện áp pack pin (V).
     */
    float getPackVoltage() const;
    /**
     * @brief Lấy dòng điện hiện tại của pack pin.
     * @return Dòng điện pack pin (A). Dương là sạc, âm là xả.
     */
    float getPackCurrent() const;
    /**
     * @brief Lấy công suất hiện tại của pack pin.
     * @return Công suất pack pin (W).
     */
    float getPackPower() const;
    /**
     * @brief Lấy trạng thái sạc (State of Charge - SOC).
     * @return Phần trăm pin (%).
     */
    int getStateOfCharge() const;
    /**
     * @brief Lấy dung lượng còn lại.
     * @return Dung lượng còn lại (Ah).
     */
    float getRemainingCapacity() const;
    /**
     * @brief Lấy nhiệt độ từ cảm biến 1.
     * @return Nhiệt độ (°C).
     */
    float getTemperature1() const;
    /**
     * @brief Lấy nhiệt độ từ cảm biến 2.
     * @return Nhiệt độ (°C).
     */
    float getTemperature2() const;
    /**
     * @brief Lấy số lượng cell pin.
     * @return Số lượng cell.
     */
    int getNumCells() const;
    /**
     * @brief Lấy danh sách điện áp của tất cả các cell.
     * @return Vector chứa điện áp của các cell (V).
     */
    std::vector<float> getCellVoltages() const;
    /**
     * @brief Lấy điện áp của một cell cụ thể.
     * @param cellIndex Chỉ số của cell (bắt đầu từ 0).
     * @return Điện áp của cell (V), hoặc 0.0 nếu chỉ số không hợp lệ.
     */
    float getCellVoltage(int cellIndex) const;
    /**
     * @brief Kiểm tra trạng thái của MOSFET sạc.
     * @return `true` nếu MOSFET sạc đang BẬT, `false` nếu đang TẮT.
     */
    bool isChargeFetOn() const;
    /**
     * @brief Kiểm tra trạng thái của MOSFET xả.
     * @return `true` nếu MOSFET xả đang BẬT, `false` nếu đang TẮT.
     */
    bool isDischargeFetOn() const;
    
    /**
     * @brief Lấy chuỗi mô tả các trạng thái bảo vệ đang hoạt động.
     * @return Chuỗi văn bản mô tả lỗi, hoặc "OK" nếu không có lỗi.
     */
    std::string getProtectionStateText() const;
    
    /**
     * @brief In thông tin chi tiết của pin ra console.
     */
    void printBatteryInfo() const;
    
    /**
     * @brief Kiểm tra xem kết nối serial có hợp lệ và đang hoạt động không.
     * @return `true` nếu kết nối hợp lệ, `false` nếu ngược lại.
     */
    bool isConnectionValid() const;

private:
    // Helper functions
    uint16_t convertTwoIntsToUint16(uint8_t high, uint8_t low);
    int16_t convertTwoIntsToInt16(uint8_t high, uint8_t low);
    uint8_t reverseBits(uint8_t byte);
    void flushSerial();
    bool writeCommand(const std::vector<uint8_t>& data);
    bool writeRequestStart();
    bool writeRequestEnd();
    bool readCellVoltages();
    bool readBasicInfo();
    bool getBMSResponse(std::vector<uint8_t>& response);
    bool hasAnyProtection() const;
};

/**
 * @brief Hàm ví dụ để minh họa cách sử dụng lớp JBDBMSSingleton.
 */
void demonstrateUsage();

#endif // JBD_BMS_SINGLETON_H