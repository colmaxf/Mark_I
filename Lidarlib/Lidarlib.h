#ifndef LIDAR_PROCESSOR_H
#define LIDAR_PROCESSOR_H

#include "Data_SDK/LakiBeamUDP.h"
#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <functional>
#include <chrono>
#include <thread>
#include <memory>

#include "../config.h"
#include "../logger/Logger.h"


// Cấu trúc điểm 2D với timestamp
struct LidarPoint {
    float x, y;              // Tọa độ Cartesian (meters)
    float distance;          // Khoảng cách (meters)
    float angle;             // Góc (radians)
    uint16_t intensity;      // Cường độ tín hiệu
    long timestamp;          // Timestamp (milliseconds)
    
    LidarPoint(float x = 0, float y = 0, float dist = 0, float ang = 0, 
               uint16_t intens = 0, long ts = 0)
        : x(x), y(y), distance(dist), angle(ang), intensity(intens), timestamp(ts) {}
    
    bool isValid() const {
        return distance > 0.01f && distance < 100.0f; // 1cm to 100m range
    }
};

// Cấu trúc để lưu trữ một scan hoàn chỉnh
struct LidarScan {
    std::vector<LidarPoint> points;
    long timestamp;
    int scan_id;
    
    LidarScan(long ts = 0, int id = 0) : timestamp(ts), scan_id(id) {}
    
    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
};

// Bộ lọc nhiễu cho dữ liệu LiDAR
class NoiseFilter {
private:
    float min_distance, max_distance;
    float max_angle_jump;
    int min_neighbors;
    
public:
    NoiseFilter(float min_dist = 0.05f, float max_dist = 50.0f, 
                float max_jump = 0.5f, int min_neigh = 2)
        : min_distance(min_dist), max_distance(max_dist), 
          max_angle_jump(max_jump), min_neighbors(min_neigh) {}
    
    std::vector<LidarPoint> filter(const std::vector<LidarPoint>& raw_points);
    
private:
    bool isPointValid(const LidarPoint& point, 
                     const std::vector<LidarPoint>& neighbors) const;
    std::vector<LidarPoint> removeOutliers(const std::vector<LidarPoint>& points) const;
    std::vector<LidarPoint> smoothAngularJumps(const std::vector<LidarPoint>& points) const;
};

// Bộ đệm dữ liệu LiDAR với khả năng đồng bộ
class LidarBuffer {
private:
    std::deque<LidarScan> scan_buffer;
    size_t max_buffer_size;
    mutable std::mutex buffer_mutex;
    std::atomic<int> next_scan_id;
    
public:
    LidarBuffer(size_t max_size = 100) 
        : max_buffer_size(max_size), next_scan_id(0) {}
    
    void addScan(const LidarScan& scan);
    LidarScan getLatestScan() const;
    std::vector<LidarScan> getRecentScans(int count) const;
    size_t size() const;
    void clear();
    
    // Lấy scan theo timestamp
    LidarScan getScanByTimestamp(long timestamp) const;
    std::vector<LidarScan> getScansInTimeRange(long start_time, long end_time) const;
};

// Bộ ổn định hóa dữ liệu realtime
class RealtimeStabilizer {
private:
    std::deque<std::vector<LidarPoint>> point_history;
    std::vector<LidarPoint> stable_points;
    
    int history_window;
    float stability_threshold;
    float outlier_threshold;
    bool is_stable;
    int stable_frame_count;
    
    mutable std::mutex stabilizer_mutex;
    
public:
    RealtimeStabilizer(int window = 5, float stab_thresh = 0.5f, float outlier_thresh = 0.1f)
        : history_window(window), stability_threshold(stab_thresh), 
          outlier_threshold(outlier_thresh), is_stable(false), stable_frame_count(0) {}
    
    bool update(const std::vector<LidarPoint>& new_points);
    std::vector<LidarPoint> getStablePoints() const;
    bool isDataStable() const;
    float getStabilityScore() const;
    
private:
    std::vector<LidarPoint> computeStabilizedPoints();
    bool checkStability(const std::vector<LidarPoint>& new_points);
    std::vector<LidarPoint> removeTemporalOutliers(const std::vector<LidarPoint>& points);
};

// Class chính xử lý LiDAR
class LidarProcessor {
private:
     // Hardware connection
    std::unique_ptr<LakiBeamUDP> lidar;
    std::string local_ip, local_port, laser_ip, laser_port;
    
    // Data processing components
    std::unique_ptr<NoiseFilter> noise_filter;
    std::unique_ptr<LidarBuffer> data_buffer;
    std::unique_ptr<RealtimeStabilizer> stabilizer;
    
    // Threading
    std::atomic<bool> is_running;
    std::atomic<bool> is_processing;
    std::unique_ptr<std::thread> processing_thread;
    
    // Statistics
    std::atomic<long> total_scans;
    std::atomic<long> valid_scans;
    std::atomic<long> stable_scans;
    std::atomic<float> average_points_per_scan;
    std::atomic<float> processing_rate; // Hz
    
    // Timing
    std::chrono::high_resolution_clock::time_point last_process_time;
    std::chrono::high_resolution_clock::time_point start_time;
    
    // Callbacks
    std::function<void(const std::vector<LidarPoint>&)> realtime_callback;
    std::function<void(const LidarScan&)> scan_callback;
    std::function<void(const std::vector<LidarPoint>&)> stable_points_callback;
    
    mutable std::mutex callback_mutex;
    
public:
    LidarProcessor(const std::string& local_ip = LIDAR_HOST_IP,
                    const std::string& local_port = std::to_string(LIDAR_PORT),
                    const std::string& laser_ip = LIDAR_CLIENT_IP,
                    const std::string& laser_port = std::to_string(LIDAR_CLIENT_PORT));

    ~LidarProcessor();
    
    // Initialization
    bool initialize();
    bool start();
    void stop();
    
    // Configuration
    void setNoiseFilterParams(float min_dist, float max_dist, float max_jump, int min_neighbors);
    void setStabilizerParams(int window, float stability_thresh, float outlier_thresh);
    void setBufferSize(size_t max_size);
    
    // Callbacks registration
    void setRealtimeCallback(std::function<void(const std::vector<LidarPoint>&)> callback);
    void setScanCallback(std::function<void(const LidarScan&)> callback);
    void setStablePointsCallback(std::function<void(const std::vector<LidarPoint>&)> callback);
    
    // Data access
    std::vector<LidarPoint> getCurrentPoints() const;
    std::vector<LidarPoint> getStablePoints() const;
    LidarScan getLatestScan() const;
    std::vector<LidarScan> getRecentScans(int count = 10) const;
    
    // Status
    bool isConnected() const;
    bool isProcessing() const;
    bool isDataStable() const;
    float getStabilityScore() const;
    
    // Statistics
    long getTotalScans() const { return total_scans.load(); }
    long getValidScans() const { return valid_scans.load(); }
    long getStableScans() const { return stable_scans.load(); }
    float getAveragePointsPerScan() const { return average_points_per_scan.load(); }
    float getProcessingRate() const { return processing_rate.load(); }
    float getDataValidityRatio() const;
    float getUptime() const;
    
    // Control
    void pauseProcessing();
    void resumeProcessing();
    void resetStatistics();

    
private:
    // Main processing loop
    void processLidarData();
    
    // Data conversion
    std::vector<LidarPoint> convertRawDataToPoints(const repark_t& pack);
    LidarScan createScanFromPoints(const std::vector<LidarPoint>& points);
    
    // Utilities
    long getCurrentTimestamp() const;
    void updateProcessingRate();
    void printStatus() const;
    
    // Data validation
    bool validateScanData(const repark_t& pack) const;
    std::vector<LidarPoint> preprocessPoints(const std::vector<LidarPoint>& raw_points);
};

// Utility functions
namespace LidarUtils {
    // Coordinate conversion
    float angleToRadians(uint16_t angle_raw);
    float distanceToMeters(uint16_t distance_raw);
    
    // Point operations
    float calculateDistance(const LidarPoint& p1, const LidarPoint& p2);
    float calculateAngleDifference(float angle1, float angle2);
    
    // Data analysis
    std::vector<LidarPoint> findNearestNeighbors(const LidarPoint& point, 
                                                const std::vector<LidarPoint>& points, 
                                                int k = 5);
    
    // Performance monitoring
    void printPointStatistics(const std::vector<LidarPoint>& points);
    void exportPointsToCSV(const std::vector<LidarPoint>& points, const std::string& filename);
    
    // Coordinate system utilities
    std::vector<LidarPoint> filterByDistance(const std::vector<LidarPoint>& points, 
                                           float min_dist, float max_dist);
    std::vector<LidarPoint> filterByAngle(const std::vector<LidarPoint>& points, 
                                         float min_angle, float max_angle);
    std::vector<LidarPoint> filterByIntensity(const std::vector<LidarPoint>& points, 
                                             uint16_t min_intensity);
}

// Configuration structure
struct LidarConfig {
    // Network settings
    std::string local_ip = LIDAR_HOST_IP;
    std::string local_port = std::to_string(LIDAR_PORT);
    std::string laser_ip = LIDAR_CLIENT_IP;
    std::string laser_port = std::to_string(LIDAR_CLIENT_PORT);

    // Processing settings
    int buffer_size = 100;
    int stabilizer_window = 5;
    float stability_threshold = 0.5f;//0.02f;
    float outlier_threshold = 0.1f;
    
    // Filter settings
    float min_distance = 0.05f;   // 5cm
    float max_distance = 50.0f;   // 50m
    float max_angle_jump = 0.5f;  // radians
    int min_neighbors = 2;
    
    // Performance settings
    int max_processing_rate = 100; // Hz
    bool enable_threading = true;
    bool enable_realtime_callbacks = true;
    
    // Debug settings
    bool verbose_logging = false;
    bool export_debug_data = false;
    std::string debug_output_path = "./lidar_debug/";
};

#endif // LIDAR_PROCESSOR_H