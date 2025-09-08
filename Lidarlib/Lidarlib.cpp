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

/**
 * @brief Lọc dữ liệu điểm LiDAR thô để loại bỏ nhiễu.
 * @param raw_points Vector chứa các điểm LiDAR thô cần được lọc.
 * @return Vector chứa các điểm LiDAR đã được lọc và làm sạch.
 * @details Quá trình lọc bao gồm 3 bước: lọc cơ bản, làm mịn bước nhảy góc và loại bỏ điểm ngoại lai.
 */
// === NoiseFilter Implementation ===
vector<LidarPoint> NoiseFilter::filter(const vector<LidarPoint>& raw_points) {
    if (raw_points.empty()) return {};
    
    vector<LidarPoint> filtered_points;
    filtered_points.reserve(raw_points.size());
    
    // Bước 1: Lọc cơ bản theo khoảng cách (min/max) và cường độ tín hiệu tối thiểu.
    for (const auto& point : raw_points) {
        if (point.distance >= min_distance && 
            point.distance <= max_distance && 
            point.intensity > 50) {
            filtered_points.push_back(point);
        }
    }
    
    // Bước 2: Loại bỏ các điểm gây ra sự thay đổi góc quá lớn so với điểm liền trước.
    filtered_points = smoothAngularJumps(filtered_points);
    
    // Bước 3: Loại bỏ các điểm ngoại lai dựa trên khoảng cách với các điểm lân cận.
    filtered_points = removeOutliers(filtered_points);
    
    return filtered_points;
}

/**
 * @brief Loại bỏ các điểm ngoại lai (outliers) khỏi tập hợp điểm.
 * @param points Vector chứa các điểm đã qua lọc sơ bộ.
 * @return Vector chứa các điểm đã được loại bỏ outlier.
 * @details Một điểm được coi là hợp lệ nếu nó có đủ số lượng lân cận trong một phạm vi góc nhỏ.
 */
vector<LidarPoint> NoiseFilter::removeOutliers(const vector<LidarPoint>& points) const {
    if (points.size() < 3) return points;
    
    vector<LidarPoint> cleaned_points;
    cleaned_points.reserve(points.size());
    
    for (size_t i = 0; i < points.size(); i++) {
        const auto& current = points[i];
        vector<LidarPoint> neighbors;
        
        // Tìm các điểm lân cận trong một phạm vi góc nhỏ (0.1 radian).
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

/**
 * @brief Làm mịn dữ liệu bằng cách loại bỏ các điểm có bước nhảy góc lớn.
 * @param points Vector chứa các điểm đầu vào.
 * @return Vector chứa các điểm đã được làm mịn.
 * @details Hàm này duyệt qua các điểm và chỉ giữ lại những điểm mà sự thay đổi về góc so với điểm trước đó nhỏ hơn ngưỡng `max_angle_jump`.
 */
vector<LidarPoint> NoiseFilter::smoothAngularJumps(const vector<LidarPoint>& points) const {
    if (points.size() < 2) return points;
    
    vector<LidarPoint> smoothed_points;
    smoothed_points.reserve(points.size());
    smoothed_points.push_back(points[0]);
    
    for (size_t i = 1; i < points.size(); i++) {
        // Tính sự khác biệt tuyệt đối về góc giữa điểm hiện tại và điểm trước đó.
        float angle_diff = abs(points[i].angle - points[i-1].angle);
        if (angle_diff < max_angle_jump) {
            smoothed_points.push_back(points[i]);
        }
    }
    
    return smoothed_points;
}

/**
 * @brief Kiểm tra xem một điểm có phải là điểm hợp lệ hay không dựa trên các điểm lân cận.
 * @param point Điểm cần kiểm tra.
 * @param neighbors Vector các điểm lân cận của `point`.
 * @return `true` nếu điểm hợp lệ, ngược lại `false`.
 * @details Một điểm được coi là hợp lệ nếu có đủ số lân cận (`min_neighbors`) và khoảng cách trung bình tới các lân cận đó nhỏ hơn một ngưỡng nhất định (0.5m).
 */
bool NoiseFilter::isPointValid(const LidarPoint& point, 
                              const vector<LidarPoint>& neighbors) const {
    // Nếu không có đủ số lượng lân cận, coi như là điểm nhiễu.
    if (neighbors.size() < min_neighbors) return false;
    
    float avg_distance = 0;
    for (const auto& neighbor : neighbors) {
        // Tính khoảng cách Euclidean giữa điểm đang xét và lân cận.
        float dx = point.x - neighbor.x;
        float dy = point.y - neighbor.y;
        avg_distance += sqrt(dx*dx + dy*dy);
    }
    // Tính khoảng cách trung bình tới các lân cận.
    avg_distance /= neighbors.size();
    
    // Nếu khoảng cách trung bình nhỏ hơn ngưỡng, điểm được coi là hợp lệ.
    return avg_distance < 0.5f;
}

// === LidarBuffer Implementation ===
/**
 * @brief Thêm một LidarScan mới vào bộ đệm.
 * @param scan Đối tượng LidarScan để thêm vào.
 * @details Hàm này thread-safe. Nó cũng tự động gán ID cho scan và quản lý kích thước bộ đệm.
 */
void LidarBuffer::addScan(const LidarScan& scan) {
    lock_guard<mutex> lock(buffer_mutex);
    
    LidarScan new_scan = scan;
    new_scan.scan_id = next_scan_id++; // Gán ID duy nhất và tăng biến đếm.
    
    scan_buffer.push_back(new_scan);
    
    if (scan_buffer.size() > max_buffer_size) {
        scan_buffer.pop_front();
    }
}

/**
 * @brief Lấy scan gần nhất từ bộ đệm.
 * @return Đối tượng LidarScan gần nhất. Trả về scan rỗng nếu bộ đệm trống.
 * @details Hàm này thread-safe.
 */
LidarScan LidarBuffer::getLatestScan() const {
    lock_guard<mutex> lock(buffer_mutex);
    if (scan_buffer.empty()) return LidarScan();
    return scan_buffer.back();
}

/**
 * @brief Lấy một số lượng các scan gần đây nhất.
 * @param count Số lượng scan muốn lấy.
 * @return Vector chứa các đối tượng LidarScan.
 * @details Hàm này thread-safe.
 */
vector<LidarScan> LidarBuffer::getRecentScans(int count) const {
    lock_guard<mutex> lock(buffer_mutex);
    vector<LidarScan> recent_scans;
    
    // Tính toán chỉ số bắt đầu để không truy cập ngoài phạm vi.
    int start_index = max(0, (int)scan_buffer.size() - count); 
    for (int i = start_index; i < (int)scan_buffer.size(); i++) {
        recent_scans.push_back(scan_buffer[i]);
    }
    
    return recent_scans;
}

/**
 * @brief Lấy kích thước hiện tại của bộ đệm.
 * @return Số lượng scan trong bộ đệm.
 */
size_t LidarBuffer::size() const {
    lock_guard<mutex> lock(buffer_mutex);
    return scan_buffer.size();
}

/**
 * @brief Xóa toàn bộ dữ liệu trong bộ đệm.
 */
void LidarBuffer::clear() {
    lock_guard<mutex> lock(buffer_mutex);
    scan_buffer.clear();
    next_scan_id = 0;
}

/**
 * @brief Tìm kiếm và lấy scan dựa trên timestamp.
 * @param timestamp Dấu thời gian (ms) của scan cần tìm.
 * @return LidarScan tìm thấy. Trả về scan rỗng nếu không tìm thấy.
 * @details Tìm scan có timestamp gần nhất với giá trị đầu vào (sai số dưới 50ms).
 */
LidarScan LidarBuffer::getScanByTimestamp(long timestamp) const {
    lock_guard<mutex> lock(buffer_mutex);
    
    // Duyệt qua bộ đệm để tìm scan có timestamp phù hợp.
    for (const auto& scan : scan_buffer) {
        if (abs(scan.timestamp - timestamp) < 50) {
            return scan;
        }
    }
    return LidarScan();
}

/**
 * @brief Lấy tất cả các scan trong một khoảng thời gian.
 * @param start_time Thời gian bắt đầu (ms).
 * @param end_time Thời gian kết thúc (ms).
 * @return Vector chứa các LidarScan trong khoảng thời gian đã cho.
 */
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
/**
 * @brief Cập nhật bộ ổn định với một frame điểm mới.
 * @param new_points Vector các điểm LiDAR của frame mới.
 * @return `true` nếu dữ liệu vừa chuyển sang trạng thái ổn định, ngược lại `false`.
 * @details Hàm này quản lý một cửa sổ lịch sử các frame, tính toán các điểm ổn định, và kiểm tra tính ổn định qua nhiều frame.
 */
bool RealtimeStabilizer::update(const vector<LidarPoint>& new_points) {
    lock_guard<mutex> lock(stabilizer_mutex);
    
    if (new_points.empty()) return false;
    
    // Thêm frame mới vào lịch sử và quản lý kích thước cửa sổ.
    point_history.push_back(new_points);
    if (point_history.size() > history_window) {
        point_history.pop_front();
    }
    
    // Cần ít nhất 3 frame trong lịch sử để bắt đầu đánh giá.
    if (point_history.size() < 3) {
        is_stable = false;
        return false;
    }
    
    // Tính toán tập hợp điểm ổn định ứng viên từ lịch sử.
    vector<LidarPoint> candidate_stable = computeStabilizedPoints();
    
    // Kiểm tra xem tập hợp ứng viên có ổn định so với tập hợp ổn định trước đó không.
    if (checkStability(candidate_stable)) {
        stable_frame_count++;
        // Dữ liệu được coi là ổn định nếu nó ổn định trong 3 frame liên tiếp.
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

/**
 * @brief Lấy tập hợp các điểm đã được ổn định.
 * @return Vector các điểm LidarPoint đã ổn định.
 * @details Hàm này thread-safe.
 */
vector<LidarPoint> RealtimeStabilizer::getStablePoints() const {
    lock_guard<mutex> lock(stabilizer_mutex);
    return stable_points;
}

/**
 * @brief Kiểm tra xem dữ liệu hiện tại có đang ở trạng thái ổn định không.
 * @return `true` nếu ổn định, ngược lại `false`.
 */
bool RealtimeStabilizer::isDataStable() const {
    lock_guard<mutex> lock(stabilizer_mutex);
    return is_stable;
}

/**
 * @brief Tính toán và trả về một điểm số thể hiện mức độ ổn định của dữ liệu.
 * @return Điểm số ổn định, từ 0.0 (không ổn định) đến 1.0 (rất ổn định).
 * @details Điểm số được tính dựa trên phương sai (variance) trung bình của vị trí các điểm giữa các frame liên tiếp trong lịch sử.
 */
float RealtimeStabilizer::getStabilityScore() const {
    lock_guard<mutex> lock(stabilizer_mutex);
    if (point_history.size() < 2) return 0.0f;
    
    float total_variance = 0.0f;
    int comparison_count = 0;
    
    // So sánh từng frame với frame liền trước nó trong lịch sử.
    for (size_t i = 1; i < point_history.size(); i++) {
        const auto& prev = point_history[i-1];
        const auto& curr = point_history[i];
        
        // So sánh các điểm tương ứng giữa hai frame.
        size_t min_size = min(prev.size(), curr.size());
        for (size_t j = 0; j < min_size; j++) {
            float dx = curr[j].x - prev[j].x;
            float dy = curr[j].y - prev[j].y;
            total_variance += dx*dx + dy*dy;
            comparison_count++;
        }
    }
    
    if (comparison_count == 0) return 0.0f;
    // Tính phương sai trung bình.
    float avg_variance = total_variance / comparison_count;
    
    // Chuyển đổi phương sai thành điểm số ổn định (phương sai càng thấp, điểm càng cao).
    return 1.0f / (1.0f + avg_variance * 100.0f);
}

/**
 * @brief Tính toán các điểm ổn định bằng cách lấy trung bình các điểm trong lịch sử.
 * @return Vector các điểm LidarPoint đã được làm trung bình và ổn định hóa.
 * @details Các điểm từ tất cả các frame trong lịch sử được nhóm lại theo góc. Sau đó, các điểm trong cùng một nhóm góc sẽ được lấy trung bình để tạo ra một điểm ổn định duy nhất.
 */
vector<LidarPoint> RealtimeStabilizer::computeStabilizedPoints() {
    if (point_history.empty()) return {};
    
    // Sử dụng map để nhóm các điểm theo khóa góc (angle_key).
    // Khóa góc được tạo bằng cách nhân góc (độ) với 10 để tăng độ phân giải.
    map<int, vector<LidarPoint>> angle_groups; 
    
    for (const auto& frame : point_history) {
        for (const auto& point : frame) {
            int angle_key = (int)(point.angle * 180.0f / M_PI * 10.0f);
            angle_groups[angle_key].push_back(point);
        }
    }
    
    vector<LidarPoint> stabilized;
    
    // Duyệt qua từng nhóm góc.
    for (const auto& [angle_key, points] : angle_groups) {
        // Bỏ qua nếu nhóm có quá ít điểm.
        if (points.size() < 2) continue;
        
        float sum_x = 0, sum_y = 0, sum_dist = 0;
        float sum_angle = 0;
        uint32_t sum_intensity = 0;
        long latest_timestamp = 0;
        
        // Tính tổng các thuộc tính của các điểm trong nhóm.
        for (const auto& p : points) {
            sum_x += p.x;
            sum_y += p.y;
            sum_dist += p.distance;
            sum_angle += p.angle;
            sum_intensity += p.intensity;
            latest_timestamp = max(latest_timestamp, p.timestamp);
        }
        
        // Tính giá trị trung bình và tạo ra điểm ổn định.
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
    LOG_INFO << "[LIDAR-RT] Stability: Generated "     << stabilized.size() << " points";
    // // Tạo stringstream để xây dựng chuỗi log
    // std::ostringstream log_stream;
    // log_stream << "[LIDAR-RT] Stability: Generated " 
    //            << stabilized.size() << " points: [";

    // // Duyệt qua vector và thêm thông tin từng điểm vào stream
    // for (size_t i = 0; i < stabilized.size(); ++i) {
    //     const auto& p = stabilized[i];
        
    //     // Thêm nội dung của điểm. Ví dụ: (angle, distance)
    //     // Bạn có thể thêm bất kỳ thuộc tính nào bạn muốn (p.x, p.y...)
    //     log_stream << "(" << p.angle << " rad, " << p.distance << " m)";
        
    //     if (i < stabilized.size() - 1) {
    //         log_stream << ", "; // Thêm dấu phẩy giữa các điểm
    //     }
    // }
    // log_stream << "]"; // Đóng ngoặc

    // // Lấy chuỗi hoàn chỉnh từ stream và đưa vào LOG_INFO
    // LOG_INFO << log_stream.str();
    
    return stabilized;
}

/**
 * @brief Kiểm tra mức độ ổn định của một tập hợp điểm mới so với tập hợp điểm ổn định hiện tại.
 * @param new_points Tập hợp điểm ứng viên mới.
 * @return `true` nếu độ lệch trung bình nhỏ hơn ngưỡng `stability_threshold`, ngược lại `false`.
 */
bool RealtimeStabilizer::checkStability(const vector<LidarPoint>& new_points) {
    if (stable_points.empty()) return true;
    if (new_points.size() != stable_points.size()) return false;
    
    float total_deviation = 0.0f;
    int valid_comparisons = 0;
    
    // So sánh từng điểm tương ứng giữa tập hợp mới và tập hợp ổn định cũ.
    for (size_t i = 0; i < min(new_points.size(), stable_points.size()); i++) {
        // Tính độ lệch (khoảng cách Euclidean).
        float dx = new_points[i].x - stable_points[i].x;
        float dy = new_points[i].y - stable_points[i].y;
        float deviation = sqrt(dx*dx + dy*dy);
        
        total_deviation += deviation;
        valid_comparisons++;
    }
    
    // Nếu không có điểm nào để so sánh, coi như không ổn định.
    if (valid_comparisons == 0) return false;
    
    // Tính độ lệch trung bình.
    float avg_deviation = total_deviation / valid_comparisons;
    
    // So sánh với ngưỡng ổn định.
    return avg_deviation < stability_threshold;
}

// === LidarProcessor Implementation ===
LidarProcessor::LidarProcessor(const string& local_ip, const string& local_port,
                                             const string& laser_ip, const string& laser_port)
    : local_ip(local_ip), local_port(local_port), laser_ip(laser_ip), laser_port(laser_port),
      is_running(false), is_processing(false), total_scans(0), valid_scans(0), 
      stable_scans(0), average_points_per_scan(0), processing_rate(0) /*,INIT_MODULE_LOGGER("LIDA")*/ {
    
    LOG_INFO << "[LIDAR-RT] LidarProcessor instance created.";

    // Khởi tạo các thành phần xử lý
    noise_filter = make_unique<NoiseFilter>();
    data_buffer = make_unique<LidarBuffer>(100);
    stabilizer = make_unique<RealtimeStabilizer>();
}

/**
 * @brief Hủy đối tượng LidarProcessor.
 * @details Tự động gọi hàm stop() để đảm bảo luồng xử lý được dừng một cách an toàn.
 */
LidarProcessor::~LidarProcessor() {
    stop();
}

bool LidarProcessor::initialize() {
    try {

        LOG_INFO  << "[LIDAR-RT] Initializing Realtime LiDAR System...";
        LOG_INFO  << "[LIDAR-RT] Local: " << local_ip << ":" << local_port;
        LOG_INFO  << "[LIDAR-RT] Laser: " << laser_ip << ":" << laser_port;       
        
        // Tạo đối tượng LakiBeamUDP
        lidar = make_unique<LakiBeamUDP>(local_ip, local_port, laser_ip, laser_port);
        
        // THÊM: Kiểm tra kết nối thực sự bằng cách thử lấy dữ liệu

        LOG_INFO  << "[LIDAR-RT] Waiting for LiDAR data...";

        // Cố gắng lấy dữ liệu trong một khoảng thời gian chờ (timeout) để xác nhận kết nối.
        auto start_wait = chrono::steady_clock::now();
        const int timeout_seconds = 5;
        bool data_received = false;
        
        while (chrono::steady_clock::now() - start_wait < chrono::seconds(timeout_seconds)) {
            repark_t test_packet;
            if (lidar->get_repackedpack(test_packet)) {
                // Kiểm tra xem gói dữ liệu nhận được có hợp lệ không.
                if (test_packet.maxdots > 0 && test_packet.maxdots <= CONFIG_CIRCLE_DOTS) {

                    data_received = true;
                    LOG_INFO  << "[LIDAR-RT] LiDAR data received! Points: " << test_packet.maxdots;

                    break;
                }
            }
            this_thread::sleep_for(chrono::milliseconds(100));
        }
        
        if (!data_received) {
            // Nếu không nhận được dữ liệu sau khoảng thời gian chờ, báo lỗi và hướng dẫn người dùng.

            LOG_ERROR << "[LIDAR-RT] No data received from LiDAR after " << timeout_seconds << " seconds!";
            LOG_ERROR << "[LIDAR-RT] Please check: ";
            LOG_ERROR << "[LIDAR-RT]   1. LiDAR is powered on";
            LOG_ERROR << "[LIDAR-RT]   2. Network cable is connected";
            LOG_ERROR << "[LIDAR-RT]   3. IP addresses are correct";
            LOG_ERROR << "[LIDAR-RT]   4. Firewall is not blocking UDP port " << local_port;
            
            lidar.reset(); // Clean up
            return false;
        }
        
         LOG_INFO  << "[LIDAR-RT] Initialization successful! LiDAR is responding.";
        return true;
        
    } catch (const exception& e) {
        
        LOG_ERROR << "[LIDAR-RT] Initialization failed: " << e.what();
        return false;
    
    }
}

bool LidarProcessor::start() {
    if (!lidar) {

        LOG_ERROR << "[LIDAR-RT] LiDAR not initialized!";
        return false;

    }
    
    if (is_running) {
        LOG_WARNING << "[LIDAR-RT] Already running!";
        return true;
    }
    
    is_running = true;
    is_processing = true;
    start_time = high_resolution_clock::now();
    
    processing_thread = make_unique<thread>(&LidarProcessor::processLidarData, this);
    
    LOG_INFO  << "[LIDAR-RT] Realtime processing started!";

    return true;
}

void LidarProcessor::stop() {
    if (!is_running) return;
    

    LOG_INFO  << "[LIDAR-RT] Stopping realtime processing...";

    
    is_running = false;
    
    if (processing_thread && processing_thread->joinable()) {
        processing_thread->join();
    }
    
    // LakiBeamUDP tự động dọn dẹp trong destructor
    lidar.reset();
    
    printStatus();

    LOG_INFO  << "[LIDAR-RT] Stopped successfully.";


}

/**
 * @brief Vòng lặp chính xử lý dữ liệu LiDAR.
 * @details Hàm này chạy trong một luồng riêng, liên tục lấy dữ liệu từ LiDAR, xử lý (lọc, ổn định hóa), cập nhật trạng thái và gọi các callback.
 */
// Main processing loop - PHẦN QUAN TRỌNG NHẤT
void LidarProcessor::processLidarData() {

    LOG_INFO  << "[LIDAR-RT] Processing thread started";
    
    while (is_running) {
        if (!is_processing) {
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }
        
        // Lấy dữ liệu thô từ LakiBeam SDK.
        repark_t raw_packet;
        
        // Sử dụng get_repackedpack (non-blocking) để tránh block thread
        bool has_data = lidar->get_repackedpack(raw_packet);
        
        if (!has_data) {
            // Không có dữ liệu mới, chờ một chút
            this_thread::sleep_for(chrono::milliseconds(1));
            continue;
        }
        
        // Kiểm tra sơ bộ tính hợp lệ của gói dữ liệu.
        if (raw_packet.maxdots == 0 || raw_packet.maxdots > CONFIG_CIRCLE_DOTS) {
            continue;
        }
        
        // Bước 1: Chuyển đổi dữ liệu thô sang cấu trúc LidarPoint.
        vector<LidarPoint> raw_points = convertRawDataToPoints(raw_packet);
        
        if (raw_points.empty()) {
            continue;
        }
        
        // Bước 2: Áp dụng bộ lọc nhiễu.
        vector<LidarPoint> filtered_points = noise_filter->filter(raw_points);
        
        // Bước 3: Cập nhật bộ ổn định hóa với dữ liệu đã lọc.
        bool became_stable = stabilizer->update(filtered_points);
        
        // Tạo scan object
        LidarScan scan = createScanFromPoints(filtered_points);
        
        // Thêm vào buffer
        data_buffer->addScan(scan);
        
        // Bước 4: Cập nhật các chỉ số thống kê hiệu suất.
        total_scans++;
        if (!filtered_points.empty()) {
            valid_scans++;
        }
        if (stabilizer->isDataStable()) {
            stable_scans++;
        }
        
        // Tính số điểm trung bình trên mỗi scan (sử dụng trung bình động).
        if (total_scans > 0) {
            average_points_per_scan = (average_points_per_scan * (total_scans - 1) + filtered_points.size()) / total_scans;
        }
        
        // Bước 5: Gọi các hàm callback đã được đăng ký.
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
        
        // Cập nhật tốc độ xử lý (Hz).
        updateProcessingRate();
        
        // Giảm tải CPU
        this_thread::sleep_for(chrono::microseconds(100));
    }
    
        LOG_INFO  << "[LIDAR-RT] Processing thread stopped";

}

/**
 * @brief Chuyển đổi dữ liệu thô từ cấu trúc `repark_t` sang vector các `LidarPoint`.
 * @param pack Gói dữ liệu thô từ LiDAR SDK.
 * @return Vector các điểm `LidarPoint`.
 */
// Chuyển đổi dữ liệu thô sang điểm
vector<LidarPoint> LidarProcessor::convertRawDataToPoints(const repark_t& pack) {
    vector<LidarPoint> points;
    points.reserve(pack.maxdots);
    
    long timestamp = getCurrentTimestamp();
    
    // Duyệt qua tất cả các điểm trong gói dữ liệu
    for (uint16_t i = 0; i < pack.maxdots; i++) {
        const cicle_pack_t& dot = pack.dotcloud[i];
        // Dữ liệu góc trong `cicle_pack_t` là độ * 100.
        float angle = dot.angle / 100.0f * M_PI / 180.0f; // Chuyển sang radian
        float distance = dot.distance / 1000.0f; // Dữ liệu khoảng cách là mm, chuyển sang mét.
        uint16_t intensity = dot.rssi;
        
        // Bỏ qua các điểm không hợp lệ
        if (distance <= 0.001f || distance > 100.0f) {
            continue;
        }
        // Chuyển sang tọa độ Cartesian
        float x = distance * cos(angle);
        float y = distance * sin(angle);
        
        points.emplace_back(x, y, distance, angle, intensity, timestamp);
    }
    
    return points;
}

/**
 * @brief Tạo một đối tượng LidarScan từ một vector điểm.
 * @param points Vector các điểm LidarPoint.
 * @return Đối tượng LidarScan.
 */
// Tạo scan từ điểm
LidarScan LidarProcessor::createScanFromPoints(const vector<LidarPoint>& points) {
    LidarScan scan(getCurrentTimestamp());
    scan.points = points;
    return scan;
}

/**
 * @brief Lấy timestamp hiện tại của hệ thống.
 * @return Timestamp dưới dạng milliseconds kể từ epoch.
 */
// Utilities
long LidarProcessor::getCurrentTimestamp() const {
    return chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now().time_since_epoch()
    ).count();
}

/**
 * @brief Cập nhật tốc độ xử lý (scans per second).
 * @details Tính toán tốc độ dựa trên thời gian trôi qua giữa lần xử lý này và lần xử lý trước.
 */
void LidarProcessor::updateProcessingRate() {
    auto now = high_resolution_clock::now();
    if (last_process_time.time_since_epoch().count() > 0) {
        auto duration = duration_cast<microseconds>(now - last_process_time);
        float rate = 1000000.0f / duration.count();
        processing_rate = rate;
    }
    last_process_time = now;
}

/**
 * @brief In ra các thông tin trạng thái và hiệu suất của LidarProcessor.
 */
void LidarProcessor::printStatus() const {
         LOG_INFO << "--- LiDAR Processor Status ---";
         LOG_INFO << "Total Scans: " << total_scans.load();
         LOG_INFO << "Valid Scans: " << valid_scans.load();
         LOG_INFO << "Stable Scans: " << stable_scans.load();
         LOG_INFO << "Processing Rate: " << processing_rate.load() << " Hz";
         LOG_INFO << "Data Validity: " << getDataValidityRatio() * 100 << "%";
         LOG_INFO << "Stability Score: " << getStabilityScore();
         LOG_INFO << "Uptime: " << getUptime() << " seconds";
         LOG_INFO << "-------------------------------";
}

/**
 * @brief Kiểm tra tính hợp lệ của một gói dữ liệu thô.
 * @param pack Gói dữ liệu `repark_t` cần kiểm tra.
 * @return `true` nếu hợp lệ, ngược lại `false`.
 */
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

/**
 * @brief Tiền xử lý các điểm thô (hiện tại chưa có logic, để dành cho mở rộng).
 * @param raw_points Vector các điểm thô.
 * @return Vector các điểm đã được tiền xử lý.
 */
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