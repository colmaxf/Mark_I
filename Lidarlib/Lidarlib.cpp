#include "Lidarlib.h"
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <numeric>

using namespace std;
using namespace chrono;

// === NoiseFilter Implementation ===
vector<LidarPoint> NoiseFilter::filter(const vector<LidarPoint>& raw_points) {
    if (raw_points.empty()) return {};
    
    vector<LidarPoint> filtered_points;
    filtered_points.reserve(raw_points.size());
    
    // Bước 1: Lọc theo khoảng cách và cường độ cơ bản
    for (const auto& point : raw_points) {
        if (point.distance >= min_distance && 
            point.distance <= max_distance && 
            point.intensity > 50) {
            filtered_points.push_back(point);
        }
    }
    
    // Bước 2: Loại bỏ các điểm gây ra bước nhảy góc đột ngột
    filtered_points = smoothAngularJumps(filtered_points);
    
    // Bước 3: Loại bỏ các điểm ngoại lai
    filtered_points = removeOutliers(filtered_points);
    
    return filtered_points;
}

vector<LidarPoint> NoiseFilter::removeOutliers(const vector<LidarPoint>& points) const {
    if (points.size() < 3) return points;
    
    vector<LidarPoint> cleaned_points;
    cleaned_points.reserve(points.size());
    
    for (size_t i = 0; i < points.size(); i++) {
        const auto& current = points[i];
        vector<LidarPoint> neighbors;
        
        for (size_t j = 0; j < points.size(); j++) {
            if (i == j) continue;
            float angle_diff = abs(current.angle - points[j].angle);
            if (angle_diff < 0.1f) { 
                neighbors.push_back(points[j]);
            }
        }
        
        if (isPointValid(current, neighbors)) {
            cleaned_points.push_back(current);
        }
    }
    
    return cleaned_points;
}

vector<LidarPoint> NoiseFilter::smoothAngularJumps(const vector<LidarPoint>& points) const {
    if (points.size() < 2) return points;
    
    vector<LidarPoint> smoothed_points;
    smoothed_points.reserve(points.size());
    smoothed_points.push_back(points[0]);
    
    for (size_t i = 1; i < points.size(); i++) {
        float angle_diff = abs(points[i].angle - points[i-1].angle);
        if (angle_diff < max_angle_jump) {
            smoothed_points.push_back(points[i]);
        }
    }
    
    return smoothed_points;
}

bool NoiseFilter::isPointValid(const LidarPoint& point, 
                              const vector<LidarPoint>& neighbors) const {
    if (neighbors.size() < min_neighbors) return false;
    
    float avg_distance = 0;
    for (const auto& neighbor : neighbors) {
        float dx = point.x - neighbor.x;
        float dy = point.y - neighbor.y;
        avg_distance += sqrt(dx*dx + dy*dy);
    }
    avg_distance /= neighbors.size();
    
    return avg_distance < 0.5f;
}

// === LidarBuffer Implementation ===
void LidarBuffer::addScan(const LidarScan& scan) {
    lock_guard<mutex> lock(buffer_mutex);
    
    LidarScan new_scan = scan;
    new_scan.scan_id = next_scan_id++;
    
    scan_buffer.push_back(new_scan);
    
    if (scan_buffer.size() > max_buffer_size) {
        scan_buffer.pop_front();
    }
}

LidarScan LidarBuffer::getLatestScan() const {
    lock_guard<mutex> lock(buffer_mutex);
    if (scan_buffer.empty()) return LidarScan();
    return scan_buffer.back();
}

vector<LidarScan> LidarBuffer::getRecentScans(int count) const {
    lock_guard<mutex> lock(buffer_mutex);
    vector<LidarScan> recent_scans;
    
    int start_index = max(0, (int)scan_buffer.size() - count);
    for (int i = start_index; i < (int)scan_buffer.size(); i++) {
        recent_scans.push_back(scan_buffer[i]);
    }
    
    return recent_scans;
}

size_t LidarBuffer::size() const {
    lock_guard<mutex> lock(buffer_mutex);
    return scan_buffer.size();
}

void LidarBuffer::clear() {
    lock_guard<mutex> lock(buffer_mutex);
    scan_buffer.clear();
    next_scan_id = 0;
}

LidarScan LidarBuffer::getScanByTimestamp(long timestamp) const {
    lock_guard<mutex> lock(buffer_mutex);
    
    for (const auto& scan : scan_buffer) {
        if (abs(scan.timestamp - timestamp) < 50) {
            return scan;
        }
    }
    return LidarScan();
}

vector<LidarScan> LidarBuffer::getScansInTimeRange(long start_time, long end_time) const {
    lock_guard<mutex> lock(buffer_mutex);
    vector<LidarScan> range_scans;
    
    for (const auto& scan : scan_buffer) {
        if (scan.timestamp >= start_time && scan.timestamp <= end_time) {
            range_scans.push_back(scan);
        }
    }
    return range_scans;
}

// === RealtimeStabilizer Implementation ===
bool RealtimeStabilizer::update(const vector<LidarPoint>& new_points) {
    lock_guard<mutex> lock(stabilizer_mutex);
    
    if (new_points.empty()) return false;
    
    point_history.push_back(new_points);
    
    if (point_history.size() > history_window) {
        point_history.pop_front();
    }
    
    if (point_history.size() < 3) {
        is_stable = false;
        return false;
    }
    
    vector<LidarPoint> candidate_stable = computeStabilizedPoints();
    
    if (checkStability(candidate_stable)) {
        stable_frame_count++;
        if (stable_frame_count >= 3) {
            stable_points = candidate_stable;
            is_stable = true;
            return true;
        }
    } else {
        stable_frame_count = 0;
        is_stable = false;
    }
    
    return false;
}

vector<LidarPoint> RealtimeStabilizer::getStablePoints() const {
    lock_guard<mutex> lock(stabilizer_mutex);
    return stable_points;
}

bool RealtimeStabilizer::isDataStable() const {
    lock_guard<mutex> lock(stabilizer_mutex);
    return is_stable;
}

float RealtimeStabilizer::getStabilityScore() const {
    lock_guard<mutex> lock(stabilizer_mutex);
    if (point_history.size() < 2) return 0.0f;
    
    float total_variance = 0.0f;
    int comparison_count = 0;
    
    for (size_t i = 1; i < point_history.size(); i++) {
        const auto& prev = point_history[i-1];
        const auto& curr = point_history[i];
        
        size_t min_size = min(prev.size(), curr.size());
        for (size_t j = 0; j < min_size; j++) {
            float dx = curr[j].x - prev[j].x;
            float dy = curr[j].y - prev[j].y;
            total_variance += dx*dx + dy*dy;
            comparison_count++;
        }
    }
    
    if (comparison_count == 0) return 0.0f;
    float avg_variance = total_variance / comparison_count;
    
    return 1.0f / (1.0f + avg_variance * 100.0f);
}

vector<LidarPoint> RealtimeStabilizer::computeStabilizedPoints() {
    if (point_history.empty()) return {};
    
    map<int, vector<LidarPoint>> angle_groups; 
    
    for (const auto& frame : point_history) {
        for (const auto& point : frame) {
            int angle_key = (int)(point.angle * 180.0f / M_PI * 10.0f);
            angle_groups[angle_key].push_back(point);
        }
    }
    
    vector<LidarPoint> stabilized;
    
    for (const auto& [angle_key, points] : angle_groups) {
        if (points.size() < 2) continue;
        
        float sum_x = 0, sum_y = 0, sum_dist = 0;
        float sum_angle = 0;
        uint32_t sum_intensity = 0;
        long latest_timestamp = 0;
        
        for (const auto& p : points) {
            sum_x += p.x;
            sum_y += p.y;
            sum_dist += p.distance;
            sum_angle += p.angle;
            sum_intensity += p.intensity;
            latest_timestamp = max(latest_timestamp, p.timestamp);
        }
        
        float inv_count = 1.0f / points.size();
        LidarPoint stabilized_point(
            sum_x * inv_count,
            sum_y * inv_count,
            sum_dist * inv_count,
            sum_angle * inv_count,
            sum_intensity / points.size(),
            latest_timestamp
        );
        
        stabilized.push_back(stabilized_point);
    }
    
    sort(stabilized.begin(), stabilized.end(), 
         [](const LidarPoint& a, const LidarPoint& b) {
             return a.angle < b.angle;
         });
    
    return stabilized;
}

bool RealtimeStabilizer::checkStability(const vector<LidarPoint>& new_points) {
    if (stable_points.empty()) return true;
    if (new_points.size() != stable_points.size()) return false;
    
    float total_deviation = 0.0f;
    int valid_comparisons = 0;
    
    for (size_t i = 0; i < min(new_points.size(), stable_points.size()); i++) {
        float dx = new_points[i].x - stable_points[i].x;
        float dy = new_points[i].y - stable_points[i].y;
        float deviation = sqrt(dx*dx + dy*dy);
        
        total_deviation += deviation;
        valid_comparisons++;
    }
    
    if (valid_comparisons == 0) return false;
    
    float avg_deviation = total_deviation / valid_comparisons;
    return avg_deviation < stability_threshold;
}

// === LidarProcessor Implementation ===
LidarProcessor::LidarProcessor(const string& local_ip, const string& local_port,
                                             const string& laser_ip, const string& laser_port)
    : local_ip(local_ip), local_port(local_port), laser_ip(laser_ip), laser_port(laser_port),
      is_running(false), is_processing(false), total_scans(0), valid_scans(0), 
      stable_scans(0), average_points_per_scan(0), processing_rate(0) {
    
// Đăng ký App ID riêng cho LIDAR
    LOG_REGISTER_APP("LIDAR", "Library for LiDAR Processing");

    // Đăng ký các context cho LIDAR
    LOG_REGISTER_CONTEXT("CORE-LIDAR", "Core LiDAR Functions");


    // Set app mặc định cho LIDAR
    LOG_SET_APP("LIBA");
    LOG_SET_CONTEXT("CORE");

    noise_filter = make_unique<NoiseFilter>();
    data_buffer = make_unique<LidarBuffer>(100);
    stabilizer = make_unique<RealtimeStabilizer>();
}

LidarProcessor::~LidarProcessor() {
    stop();
}

bool LidarProcessor::initialize() {
    Logger& logger = Logger::get_instance();
    try {
        #ifdef ENABLE_LOG
            LOG_INFO << "[LIDAR-RT] Initializing Realtime LiDAR System...";
            LOG_INFO << "[LIDAR-RT] Local: " << local_ip << ":" << local_port;
            LOG_INFO << "[LIDAR-RT] Laser: " << laser_ip << ":" << laser_port;
        #else
            cout << "[LIDAR-RT] Initializing Realtime LiDAR System..." << endl;
            cout << "[LIDAR-RT] Local: " << local_ip << ":" << local_port << endl;
            cout << "[LIDAR-RT] Laser: " << laser_ip << ":" << laser_port << endl;
        #endif
        
        // Tạo đối tượng LakiBeamUDP
        lidar = make_unique<LakiBeamUDP>(local_ip, local_port, laser_ip, laser_port);
        
        // THÊM: Kiểm tra kết nối thực sự bằng cách thử lấy dữ liệu
        cout << "[LIDAR-RT] Waiting for LiDAR data..." << endl;
        
        // Thử lấy dữ liệu trong vòng 5 giây
        auto start_wait = chrono::steady_clock::now();
        const int timeout_seconds = 5;
        bool data_received = false;
        
        while (chrono::steady_clock::now() - start_wait < chrono::seconds(timeout_seconds)) {
            repark_t test_packet;
            if (lidar->get_repackedpack(test_packet)) {
                if (test_packet.maxdots > 0 && test_packet.maxdots <= CONFIG_CIRCLE_DOTS) {
                    data_received = true;
#ifdef ENABLE_LOG
                    LOG_INFO << "[LIDAR-RT] LiDAR data received! Points: " << test_packet.maxdots;
#else
                    cout << "[LIDAR-RT] LiDAR data received! Points: " << test_packet.maxdots << endl;
#endif
                    break;
                }
            }
            this_thread::sleep_for(chrono::milliseconds(100));
        }
        
        if (!data_received) {
#ifdef ENABLE_LOG
            LOG_ERROR << "[LIDAR-RT] No data received from LiDAR after " << timeout_seconds << " seconds!";
            LOG_ERROR << "[LIDAR-RT] Please check: ";
            LOG_ERROR << "[LIDAR-RT]   1. LiDAR is powered on";
            LOG_ERROR << "[LIDAR-RT]   2. Network cable is connected";
            LOG_ERROR << "[LIDAR-RT]   3. IP addresses are correct";
            LOG_ERROR << "[LIDAR-RT]   4. Firewall is not blocking UDP port " << local_port;
#else
            cerr << "[LIDAR-RT] ERROR: No data received from LiDAR after " << timeout_seconds << " seconds!" << endl;
            cerr << "[LIDAR-RT] Please check: " << endl;
            cerr << "[LIDAR-RT]   1. LiDAR is powered on" << endl;
            cerr << "[LIDAR-RT]   2. Network cable is connected" << endl;
            cerr << "[LIDAR-RT]   3. IP addresses are correct" << endl;
            cerr << "[LIDAR-RT]   4. Firewall is not blocking UDP port " << local_port << endl;
#endif
            
            lidar.reset(); // Clean up
            return false;
        }
        
#ifdef ENABLE_LOG
        LOG_INFO << "[LIDAR-RT] Initialization successful! LiDAR is responding.";
#else
        cout << "[LIDAR-RT] Initialization successful! LiDAR is responding." << endl;
#endif
        return true;
        
    } catch (const exception& e) {
#ifdef ENABLE_LOG
        LOG_ERROR << "[LIDAR-RT] Initialization failed: " << e.what();
#else
        cerr << "[LIDAR-RT] Initialization failed: " << e.what() << endl;
#endif
        return false;
    }
}

bool LidarProcessor::start() {
    if (!lidar) {
        #ifdef ENABLE_LOG
            LOG_ERROR << "[LIDAR-RT] LiDAR not initialized!";
        #else
            cout << "[LIDAR-RT]  LiDAR not initialized!" << endl;
        #endif
        return false;
    }
    
    if (is_running) {
#ifdef ENABLE_LOG
            LOG_WARNING << "[LIDAR-RT] Already running!";
#else
            cout << "[LIDAR-RT] ⚠️ Already running!" << endl;
#endif
        return true;
    }
    
    is_running = true;
    is_processing = true;
    start_time = high_resolution_clock::now();
    
    processing_thread = make_unique<thread>(&LidarProcessor::processLidarData, this);
    
#ifdef ENABLE_LOG
        LOG_INFO << "[LIDAR-RT] Realtime processing started!";
#else
        cout << "[LIDAR-RT] Realtime processing started!" << endl;
#endif
    return true;
}

void LidarProcessor::stop() {
    if (!is_running) return;
    
#ifdef ENABLE_LOG
        LOG_INFO << "[LIDAR-RT] Stopping realtime processing...";
#else
        cout << "[LIDAR-RT] Stopping realtime processing..." << endl;
#endif
    
    is_running = false;
    
    if (processing_thread && processing_thread->joinable()) {
        processing_thread->join();
    }
    
    // LakiBeamUDP tự động dọn dẹp trong destructor
    lidar.reset();
    
    printStatus();
#ifdef ENABLE_LOG
        LOG_INFO << "[LIDAR-RT] Stopped successfully.";
#else
        cout << "[LIDAR-RT] Stopped successfully." << endl;
#endif
}

// Main processing loop - PHẦN QUAN TRỌNG NHẤT
void LidarProcessor::processLidarData() {
#ifdef ENABLE_LOG
        LOG_INFO << "[LIDAR-RT] Processing thread started";
#else
        cout << "[LIDAR-RT] Processing thread started" << endl;
#endif
    
    while (is_running) {
        if (!is_processing) {
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }
        
        // Lấy dữ liệu từ LiDAR SDK
        repark_t raw_packet;
        
        // Sử dụng get_repackedpack (non-blocking) để tránh block thread
        bool has_data = lidar->get_repackedpack(raw_packet);
        
        if (!has_data) {
            // Không có dữ liệu mới, chờ một chút
            this_thread::sleep_for(chrono::milliseconds(1));
            continue;
        }
        
        // Kiểm tra dữ liệu hợp lệ
        if (raw_packet.maxdots == 0 || raw_packet.maxdots > CONFIG_CIRCLE_DOTS) {
            continue;
        }
        
        // Chuyển đổi raw data sang điểm
        vector<LidarPoint> raw_points = convertRawDataToPoints(raw_packet);
        
        if (raw_points.empty()) {
            continue;
        }
        
        // Áp dụng noise filter
        vector<LidarPoint> filtered_points = noise_filter->filter(raw_points);
        
        // Cập nhật stabilizer
        bool became_stable = stabilizer->update(filtered_points);
        
        // Tạo scan object
        LidarScan scan = createScanFromPoints(filtered_points);
        
        // Thêm vào buffer
        data_buffer->addScan(scan);
        
        // Cập nhật thống kê
        total_scans++;
        if (!filtered_points.empty()) {
            valid_scans++;
        }
        if (stabilizer->isDataStable()) {
            stable_scans++;
        }
        
        // Tính average points per scan
        if (total_scans > 0) {
            average_points_per_scan = (average_points_per_scan * (total_scans - 1) + filtered_points.size()) / total_scans;
        }
        
        // Gọi callbacks nếu có
        {
            lock_guard<mutex> lock(callback_mutex);
            
            // Realtime callback cho mọi frame
            if (realtime_callback) {
                realtime_callback(filtered_points);
            }
            
            // Scan callback
            if (scan_callback) {
                scan_callback(scan);
            }
            
            // Stable points callback khi dữ liệu vừa ổn định
            if (stable_points_callback && became_stable) {
                stable_points_callback(stabilizer->getStablePoints());
            }
        }
        
        // Cập nhật processing rate
        updateProcessingRate();
        
        // Giảm tải CPU
        this_thread::sleep_for(chrono::microseconds(100));
    }
    
#ifdef ENABLE_LOG
        LOG_INFO << "[LIDAR-RT] Processing thread stopped";
#else
        cout << "[LIDAR-RT] Processing thread stopped" << endl;
#endif
}

// Chuyển đổi dữ liệu thô sang điểm
vector<LidarPoint> LidarProcessor::convertRawDataToPoints(const repark_t& pack) {
    vector<LidarPoint> points;
    points.reserve(pack.maxdots);
    
    long timestamp = getCurrentTimestamp();
    
    // Duyệt qua tất cả các điểm trong gói dữ liệu
    for (uint16_t i = 0; i < pack.maxdots; i++) {
        const cicle_pack_t& dot = pack.dotcloud[i];
        float angle_deg = dot.angle / 100.0f;
        // Chuyển đổi từ dữ liệu thô
        // // Chuyển đổi góc (từ raw * 100 sang độ)
        // float angle_deg = dot.angle / 100.0;
        // angle đã ở dạng độ * 100 trong cicle_pack_t
        float angle = dot.angle / 100.0f * M_PI / 180.0f; // Chuyển sang radian
        float distance = dot.distance / 1000.0f; // mm sang mét
        uint16_t intensity = dot.rssi;
        
        // Bỏ qua các điểm không hợp lệ
        if (distance <= 0.001f || distance > 100.0f) {
            continue;
        }

        


        // Chuyển sang tọa độ Cartesian
        float x= distance * cos(angle);
        float y = distance * sin(angle);
        
        points.emplace_back(x, y, distance, angle, intensity, timestamp);
    }
    
    return points;
}

// Tạo scan từ điểm
LidarScan LidarProcessor::createScanFromPoints(const vector<LidarPoint>& points) {
    LidarScan scan(getCurrentTimestamp());
    scan.points = points;
    return scan;
}

// Utilities
long LidarProcessor::getCurrentTimestamp() const {
    return chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now().time_since_epoch()
    ).count();
}

void LidarProcessor::updateProcessingRate() {
    auto now = high_resolution_clock::now();
    if (last_process_time.time_since_epoch().count() > 0) {
        auto duration = duration_cast<microseconds>(now - last_process_time);
        float rate = 1000000.0f / duration.count();
        processing_rate = rate;
    }
    last_process_time = now;
}

void LidarProcessor::printStatus() const {
#ifdef ENABLE_LOG
        LOG_INFO << "\n--- LiDAR Processor Status ---\n"
                 << "Total Scans: " << total_scans.load() << "\n"
                 << "Valid Scans: " << valid_scans.load() << "\n"
                 << "Stable Scans: " << stable_scans.load() << "\n"
                 << "Processing Rate: " << processing_rate.load() << " Hz\n"
                 << "Data Validity: " << getDataValidityRatio() * 100 << "%\n"
                 << "Stability Score: " << getStabilityScore() << "\n"
                 << "Uptime: " << getUptime() << " seconds\n"
                 << "-------------------------------";
#else
            cout << "\n--- LiDAR Processor Status ---" << endl;
            cout << "Total Scans: " << total_scans.load() << endl;
            cout << "Valid Scans: " << valid_scans.load() << endl;
            cout << "Stable Scans: " << stable_scans.load() << endl;
            cout << "Processing Rate: " << processing_rate.load() << " Hz" << endl;
            cout << "Data Validity: " << getDataValidityRatio() * 100 << "%" << endl;
            cout << "Stability Score: " << getStabilityScore() << endl;
            cout << "Uptime: " << getUptime() << " seconds" << endl;
            cout << "-------------------------------\n" << endl;
#endif
}

// Validation
bool LidarProcessor::validateScanData(const repark_t& pack) const {
    // Kiểm tra số lượng điểm hợp lệ
    if (pack.maxdots == 0 || pack.maxdots > CONFIG_CIRCLE_DOTS) {
        return false;
    }
    
    // Kiểm tra interval time hợp lệ
    if (pack.interval == 0) {
        return false;
    }
    
    return true;
}

vector<LidarPoint> LidarProcessor::preprocessPoints(const vector<LidarPoint>& raw_points) {
    // Có thể thêm preprocessing logic ở đây nếu cần
    return raw_points;
}

// Các hàm getter
bool LidarProcessor::isDataStable() const {
    if (stabilizer) {
        return stabilizer->isDataStable();
    }
    return false;
}

float LidarProcessor::getStabilityScore() const {
    if (stabilizer) {
        return stabilizer->getStabilityScore();
    }
    return 0.0f;
}

float LidarProcessor::getDataValidityRatio() const {
    long total = total_scans.load();
    if (total == 0) return 0.0f;
    return static_cast<float>(valid_scans.load()) / total;
}

float LidarProcessor::getUptime() const {
    if (!is_running) return 0.0f;
    auto now = high_resolution_clock::now();
    return duration_cast<duration<float>>(now - start_time).count();
}

void LidarProcessor::pauseProcessing() {
    is_processing = false;
}

void LidarProcessor::resumeProcessing() {
    is_processing = true;
}

void LidarProcessor::resetStatistics() {
    total_scans = 0;
    valid_scans = 0;
    stable_scans = 0;
}

// Setters cho parameters
void LidarProcessor::setNoiseFilterParams(float min_dist, float max_dist, 
                                                 float max_jump, int min_neighbors) {
    noise_filter = make_unique<NoiseFilter>(min_dist, max_dist, max_jump, min_neighbors);
}

void LidarProcessor::setStabilizerParams(int window, float stability_thresh, 
                                               float outlier_thresh) {
    stabilizer = make_unique<RealtimeStabilizer>(window, stability_thresh, outlier_thresh);
}

void LidarProcessor::setBufferSize(size_t max_size) {
    data_buffer = make_unique<LidarBuffer>(max_size);
}

// Callbacks
void LidarProcessor::setRealtimeCallback(function<void(const vector<LidarPoint>&)> callback) {
    lock_guard<mutex> lock(callback_mutex);
    realtime_callback = callback;
}

void LidarProcessor::setScanCallback(function<void(const LidarScan&)> callback) {
    lock_guard<mutex> lock(callback_mutex);
    scan_callback = callback;
}

void LidarProcessor::setStablePointsCallback(function<void(const vector<LidarPoint>&)> callback) {
    lock_guard<mutex> lock(callback_mutex);
    stable_points_callback = callback;
}

// Data access
vector<LidarPoint> LidarProcessor::getCurrentPoints() const {
    LidarScan latest = data_buffer->getLatestScan();
    return latest.points;
}

vector<LidarPoint> LidarProcessor::getStablePoints() const {
    if (stabilizer) {
        return stabilizer->getStablePoints();
    }
    return {};
}

LidarScan LidarProcessor::getLatestScan() const {
    return data_buffer->getLatestScan();
}

vector<LidarScan> LidarProcessor::getRecentScans(int count) const {
    return data_buffer->getRecentScans(count);
}

bool LidarProcessor::isConnected() const {
    return lidar != nullptr;
}

bool LidarProcessor::isProcessing() const {
    return is_processing.load();
}