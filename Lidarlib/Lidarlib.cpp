#include "Lidarlib.h"
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <unistd.h>
#include <chrono>

using namespace std;

// === SimpleSLAM Implementation ===

/**
 * @brief Constructor cho lớp SimpleSLAM.
 * @param max_corr_dist Khoảng cách tối đa để xem xét hai điểm là tương ứng trong ICP.
 * @param loop_threshold Ngưỡng khoảng cách để phát hiện một vòng lặp (loop closure).
 */
SimpleSLAM::SimpleSLAM(float max_corr_dist, float loop_threshold)
    : max_correspondence_distance(max_corr_dist), loop_closure_threshold(loop_threshold),
      next_node_id(0), current_pose(0, 0, 0, 0), last_pose(0, 0, 0, 0) {}

/**
 * @brief Thêm một scan mới vào hệ thống SLAM, cập nhật pose và pose graph.
 * @param scan_points Vector các điểm 2D từ lần quét LiDAR mới nhất.
 * @param[out] estimated_pose Pose được ước tính của robot sau khi xử lý scan.
 * @return true nếu scan được thêm thành công.
 */
bool SimpleSLAM::add_scan(const vector<Point2D>& scan_points, RobotPose& estimated_pose) {
    lock_guard<mutex> lock(slam_mutex);
    
    // Lấy timestamp từ pose hiện tại hoặc scan
    long timestamp = 0;
    if (!pose_graph.empty()) timestamp = pose_graph.back()->pose.timestamp + 100; // Giả định 100ms mỗi scan

    if (pose_graph.empty()) {
        // First scan - set initial pose
        estimated_pose = RobotPose(0, 0, 0, timestamp);
        current_pose = estimated_pose;
    } else {
        // Estimate pose using scan matching
        estimated_pose = estimate_pose_from_scan(scan_points);
        current_pose = estimated_pose;
        
        // Check for loop closure
        int loop_node = find_loop_closure(current_pose);
        if (loop_node >= 0) {
            cout << "[SLAM] 🔄 Loop closure detected with node " << loop_node << endl;
            // Simplified: adjust current pose based on loop closure
            auto& loop_pose = pose_graph[loop_node]->pose;
            current_pose.x = (current_pose.x + loop_pose.x) / 2.0f;
            current_pose.y = (current_pose.y + loop_pose.y) / 2.0f;
            estimated_pose = current_pose;
        }
    }
    
    // Tạo node mới trong pose graph
    auto new_node = make_shared<PoseGraphNode>(estimated_pose, scan_points, next_node_id++);
    
    // Tính convex hull cho node này
    vector<Point2D> local_points = scan_points; // Tạo bản sao để tính toán
    new_node->convex_hull = compute_convex_hull(local_points);
    
    pose_graph.push_back(new_node);
    last_pose = current_pose;
    
    return true;
}

/**
 * @brief Ước tính pose của robot từ scan hiện tại dựa trên scan trước đó.
 * 
 * Sử dụng một mô hình chuyển động đơn giản kết hợp với ICP để tinh chỉnh vị trí.
 * @param current_scan Vector các điểm 2D của lần quét hiện tại.
 * @return Pose mới được ước tính của robot.
 */
RobotPose SimpleSLAM::estimate_pose_from_scan(const vector<Point2D>& current_scan) {
    if (pose_graph.empty()) {
        return RobotPose(0, 0, 0, 0);
    }
    
    // Simplified odometry model - assume small movement
    float dx = 0.05f;  // 5cm forward per scan (example)
    float dy = 0.0f;
    float dtheta = 0.01f; // 0.01 radian rotation per scan
    
    // Simple motion model update
    RobotPose new_pose;
    new_pose.x = last_pose.x + dx * cos(last_pose.theta) - dy * sin(last_pose.theta);
    new_pose.y = last_pose.y + dx * sin(last_pose.theta) + dy * cos(last_pose.theta);
    new_pose.theta = last_pose.theta + dtheta;
    //new_pose.timestamp = current_scan.empty() ? 0 : current_scan[0].timestamp;
    
    // Cập nhật timestamp
    if (!pose_graph.empty()) new_pose.timestamp = pose_graph.back()->pose.timestamp + 100;
    else new_pose.timestamp = 0;
    
    // Refined using simple ICP if we have previous scans
    if (pose_graph.size() > 1) {
        auto& last_scan = pose_graph.back()->scan_points;
        RobotPose icp_correction = simple_icp(current_scan, last_scan);
        
        // Apply correction
        new_pose.x += icp_correction.x * 0.3f; // Weight factor
        new_pose.y += icp_correction.y * 0.3f;
        new_pose.theta += icp_correction.theta * 0.3f;
    }
    
    return new_pose;
}

/**
 * @brief Tìm kiếm các vòng lặp (loop closures) trong pose graph.
 * 
 * So sánh pose hiện tại với các pose cũ trong graph để tìm các vị trí đã ghé thăm lại.
 * @param current_pose Pose hiện tại của robot.
 * @return ID của node tạo thành vòng lặp, hoặc -1 nếu không tìm thấy.
 */
int SimpleSLAM::find_loop_closure(const RobotPose& current_pose) {
    // Bỏ qua các node gần đây để tránh phát hiện sai
    if (pose_graph.size() < 10) return -1;

    for (size_t i = 0; i < pose_graph.size() - 10; i++) {
        if (pose_distance(current_pose, pose_graph[i]->pose) < loop_closure_threshold) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Chuyển đổi một tập hợp các điểm từ hệ tọa độ cục bộ của robot sang hệ tọa độ toàn cục.
 * @param local_points Vector các điểm trong hệ tọa độ cục bộ.
 * @param robot_pose Pose của robot (vị trí và hướng) trong hệ tọa độ toàn cục.
 * @return Vector các điểm đã được chuyển đổi sang hệ tọa độ toàn cục.
 */
vector<Point2D> SimpleSLAM::transform_points_to_global(const vector<Point2D>& local_points, 
                                                      const RobotPose& robot_pose) {
    vector<Point2D> global_points;
    global_points.reserve(local_points.size());
    
    for (const auto& point : local_points) {
        Point2D global_point = transform_point(point, robot_pose);
        global_points.push_back(global_point);
    }
    
    return global_points;
}

/**
 * @brief Lấy tất cả các convex hull trong pose graph và chuyển đổi chúng sang hệ tọa độ toàn cục.
 * @return Một vector chứa các convex hull, mỗi hull là một vector các điểm toàn cục.
 */
vector<vector<Point2D>> SimpleSLAM::get_all_global_hulls() const {
    lock_guard<mutex> lock(slam_mutex);
    vector<vector<Point2D>> all_hulls;
    
    for (const auto& node : pose_graph) {
        if (!node->convex_hull.empty()) {
            vector<Point2D> global_hull = transform_points_to_global(node->convex_hull, node->pose);
            all_hulls.push_back(global_hull);
        }
    }
    
    return all_hulls;
}

/**
 * @brief Lấy pose hiện tại được ước tính của robot.
 * @return Pose hiện tại của robot.
 */
RobotPose SimpleSLAM::get_current_pose() const {
    lock_guard<mutex> lock(slam_mutex);
    return current_pose;
}

/**
 * @brief Lấy toàn bộ pose graph.
 * @return Một vector các con trỏ chia sẻ đến các node trong graph.
 */
vector<shared_ptr<PoseGraphNode>> SimpleSLAM::get_pose_graph() const {
    lock_guard<mutex> lock(slam_mutex);
    return pose_graph;
}

/**
 * @brief (Private) Thực hiện thuật toán Iterative Closest Point (ICP) đơn giản.
 * 
 * Ước tính sự dịch chuyển (translation) giữa hai đám mây điểm (source và target).
 * @param source Đám mây điểm nguồn (scan hiện tại).
 * @param target Đám mây điểm mục tiêu (scan trước đó).
 * @return Một RobotPose chỉ chứa thông tin về sự dịch chuyển (x, y) được ước tính.
 */
RobotPose SimpleSLAM::simple_icp(const vector<Point2D>& source, const vector<Point2D>& target) {
    // Simplified ICP - just find average displacement
    if (source.empty() || target.empty()) {
        return RobotPose(0, 0, 0, 0);
    }
    
    float sum_dx = 0, sum_dy = 0;
    int matches = 0;
    
    for (const auto& src_point : source) {
        //
        float min_dist_sq = 1e12; // Use squared distance for efficiency
        const Point2D* closest_target_ptr = nullptr;
        //
        for (const auto& tgt_point : target) {
            float dx = src_point.x - tgt_point.x;
            float dy = src_point.y - tgt_point.y;
            float dist_sq = dx*dx + dy*dy;
            
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_target_ptr = &tgt_point;
            }
        }
        
        if (closest_target_ptr && min_dist_sq < (max_correspondence_distance * max_correspondence_distance)) {
            sum_dx += closest_target_ptr->x - src_point.x;
            sum_dy += closest_target_ptr->y - src_point.y;
            matches++;
        }
    }
    
    if (matches > 5) { // Require a minimum number of matches
        return RobotPose(sum_dx / matches, sum_dy / matches, 0, 0);
    }
    
    return RobotPose(0, 0, 0, 0);
}

/**
 * @brief (Private) Tính khoảng cách Euclide giữa hai pose (chỉ xét x, y).
 * @param p1 Pose thứ nhất.
 * @param p2 Pose thứ hai.
 * @return Khoảng cách giữa hai pose.
 */
float SimpleSLAM::pose_distance(const RobotPose& p1, const RobotPose& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}

/**
 * @brief (Private) Chuyển đổi một điểm từ hệ tọa độ cục bộ sang toàn cục.
 * @param point Điểm trong hệ tọa độ cục bộ.
 * @param pose Pose của robot trong hệ tọa độ toàn cục.
 * @return Điểm đã được chuyển đổi.
 */
Point2D SimpleSLAM::transform_point(const Point2D& point, const RobotPose& pose) {
    float cos_theta = cos(pose.theta);
    float sin_theta = sin(pose.theta);
    
    float global_x = pose.x + point.x * cos_theta - point.y * sin_theta;
    float global_y = pose.y + point.x * sin_theta + point.y * cos_theta;
    
    return Point2D(global_x, global_y, point.distance);
}

/**
 * @brief (Private) Tính tích có hướng (cross product) của 3 điểm.
 * @param O Điểm gốc.
 * @param A Điểm thứ nhất.
 * @param B Điểm thứ hai.
 * @return > 0 cho rẽ trái, < 0 cho rẽ phải, = 0 nếu thẳng hàng.
 */
float SimpleSLAM::cross_product(const Point2D& O, const Point2D& A, const Point2D& B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

/**
 * @brief (Private) Tính toán convex hull của một tập hợp các điểm bằng thuật toán Andrew's monotone chain.
 * @param points Một vector các điểm Point2D.
 * @return Convex hull dưới dạng một vector các điểm Point2D.
 */
vector<Point2D> SimpleSLAM::compute_convex_hull(vector<Point2D> points) {
    if (points.size() <= 1) return points;
    
    sort(points.begin(), points.end());
    points.erase(unique(points.begin(), points.end()), points.end());
    
    if (points.size() <= 1) return points;
    
    vector<Point2D> hull;
    
    // Lower hull
    for (const auto& p : points) {
        while (hull.size() >= 2 && 
               cross_product(hull[hull.size()-2], hull[hull.size()-1], p) <= 0) {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    
    // Upper hull
    int lower_size = hull.size();
    for (int i = points.size() - 2; i >= 0; i--) {
        const auto& p = points[i];
        while (hull.size() > lower_size && 
               cross_product(hull[hull.size()-2], hull[hull.size()-1], p) <= 0) {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    
    if (hull.size() > 1) hull.pop_back();
    return hull;
}

// === SlamConvexHullManager Implementation ===

/**
 * @brief Constructor cho lớp SlamConvexHullManager.
 * @param history_size Số lượng hull tối đa để lưu trong lịch sử.
 * @param threshold Ngưỡng chênh lệch để xác định hull là ổn định.
 */
SlamConvexHullManager::SlamConvexHullManager(int history_size, float threshold)
    : max_history_size(history_size), stability_threshold(threshold), 
      is_stable(false), stable_count(0) {}

/**
 * @brief Cập nhật với một convex hull mới và pose của robot, kiểm tra tính ổn định.
 * @param new_hull Convex hull mới trong hệ tọa độ cục bộ.
 * @param robot_pose Pose hiện tại của robot.
 * @return true nếu hull vừa trở nên ổn định.
 */
bool SlamConvexHullManager::update(const vector<Point2D>& new_hull, const RobotPose& robot_pose) {
    lock_guard<mutex> lock(hull_mutex);
    
    hull_history.push_back(new_hull);
    
    if (hull_history.size() > (size_t)max_history_size) {
        hull_history.pop_front();
    }
    
    if (hull_history.size() < 3) {
        is_stable = false;
        return false;
    }
    
    vector<Point2D> avg_hull = compute_average_hull();
    
    if (is_hull_stable_internal(avg_hull)) {
        stable_count++;
        if (stable_count >= 3) {
            stable_hull = avg_hull;
            
            // Transform to global coordinate
            global_hull = transform_hull_to_global(stable_hull, robot_pose);
            
            is_stable = true;
            return true;
        }
    } else {
        stable_count = 0;
        is_stable = false;
    }
    
    return false;
}

/**
 * @brief Lấy convex hull ổn định gần nhất trong hệ tọa độ cục bộ.
 * @return Vector các điểm 2D tạo thành hull ổn định cục bộ.
 */
vector<Point2D> SlamConvexHullManager::get_stable_hull() const {
    lock_guard<mutex> lock(hull_mutex);
    return stable_hull;
}

/**
 * @brief Lấy convex hull ổn định gần nhất trong hệ tọa độ toàn cục.
 * @return Vector các điểm 2D tạo thành hull ổn định toàn cục.
 */
vector<Point2D> SlamConvexHullManager::get_global_hull() const {
    lock_guard<mutex> lock(hull_mutex);
    return global_hull;
}

/**
 * @brief Kiểm tra xem hull hiện tại có được coi là ổn định hay không.
 * @return true nếu hull ổn định.
 */
bool SlamConvexHullManager::is_hull_stable() const {
    lock_guard<mutex> lock(hull_mutex);
    return is_stable;
}

/**
 * @brief (Private) Tính toán một "hull trung bình" từ lịch sử các hull.
 * @return Một convex hull mới được tính từ tất cả các điểm trong lịch sử.
 */
vector<Point2D> SlamConvexHullManager::compute_average_hull() {
    if (hull_history.empty()) return {};
    
    vector<Point2D> all_points;
    for (const auto& hull : hull_history) {
        all_points.insert(all_points.end(), hull.begin(), hull.end());
    }
    
    return compute_convex_hull(all_points);
}

/**
 * @brief (Private) Kiểm tra nội bộ xem một hull mới có ổn định so với hull ổn định hiện tại không.
 * @param new_hull Hull ứng cử viên để kiểm tra.
 * @return true nếu chênh lệch nằm trong ngưỡng cho phép.
 */
bool SlamConvexHullManager::is_hull_stable_internal(const vector<Point2D>& new_hull) {
    if (stable_hull.empty()) return true;
    if (new_hull.size() != stable_hull.size()) return false;
    
    float total_diff = 0.0f;
    for (size_t i = 0; i < new_hull.size(); i++) {
        float dx = new_hull[i].x - stable_hull[i].x;
        float dy = new_hull[i].y - stable_hull[i].y;
        total_diff += sqrt(dx*dx + dy*dy);
    }
    
    if (new_hull.empty()) return true; // Avoid division by zero
    float avg_diff = total_diff / new_hull.size();
    return avg_diff < stability_threshold;
}

/**
 * @brief (Private) Tính toán convex hull của một tập hợp các điểm.
 * @param points Vector các điểm đầu vào.
 * @return Vector các điểm tạo thành convex hull.
 */
vector<Point2D> SlamConvexHullManager::compute_convex_hull(vector<Point2D> points) {
    if (points.size() <= 1) return points;
    
    sort(points.begin(), points.end());
    points.erase(unique(points.begin(), points.end()), points.end());
    
    if (points.size() <= 1) return points;
    
    vector<Point2D> hull;
    
    // Lower hull
    for (const auto& p : points) {
        while (hull.size() >= 2 && 
               cross_product(hull[hull.size()-2], hull[hull.size()-1], p) <= 0) {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    
    // Upper hull
    int lower_size = hull.size();
    for (int i = points.size() - 2; i >= 0; i--) {
        const auto& p = points[i];
        while (hull.size() > lower_size && 
               cross_product(hull[hull.size()-2], hull[hull.size()-1], p) <= 0) {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    
    if (hull.size() > 1) hull.pop_back();
    return hull;
}

/**
 * @brief (Private) Tính tích có hướng (cross product) của 3 điểm.
 * @param O Điểm gốc.
 * @param A Điểm thứ nhất.
 * @param B Điểm thứ hai.
 * @return Giá trị > 0 cho rẽ trái, < 0 cho rẽ phải, và = 0 nếu thẳng hàng.
 */
float SlamConvexHullManager::cross_product(const Point2D& O, const Point2D& A, const Point2D& B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

/**
 * @brief (Private) Chuyển đổi một hull từ tọa độ cục bộ sang tọa độ toàn cục.
 * @param local_hull Hull trong tọa độ cục bộ.
 * @param robot_pose Pose của robot.
 * @return Hull trong tọa độ toàn cục.
 */
vector<Point2D> SlamConvexHullManager::transform_hull_to_global(const vector<Point2D>& local_hull, 
                                                                const RobotPose& robot_pose) {
    vector<Point2D> global_hull_points;
    global_hull_points.reserve(local_hull.size());
    float cos_theta = cos(robot_pose.theta);
    float sin_theta = sin(robot_pose.theta);

    for (const auto& point : local_hull) {
        float global_x = robot_pose.x + point.x * cos_theta - point.y * sin_theta;
        float global_y = robot_pose.y + point.x * sin_theta + point.y * cos_theta;
        global_hull_points.emplace_back(global_x, global_y, point.distance);
    }
    return global_hull_points;
}

// === LidarProcessor Implementation ===

/**
 * @brief Constructor cho LidarProcessor.
 * @param local_ip Địa chỉ IP cục bộ để lắng nghe.
 * @param local_port Cổng cục bộ để lắng nghe.
 * @param laser_ip Địa chỉ IP của cảm biến LiDAR.
 * @param laser_port Cổng của cảm biến LiDAR.
 */
LidarProcessor::LidarProcessor(const string& local_ip, const string& local_port,
                                       const string& laser_ip, const string& laser_port)
    : local_ip(local_ip), local_port(local_port), laser_ip(laser_ip), laser_port(laser_port),
      is_running(false), is_processing(false), scan_count(0), valid_hulls_count(0), 
      loop_closures_count(0) {
    
    hull_manager = make_unique<SlamConvexHullManager>(8, 0.15f);
    slam_system = make_unique<SimpleSLAM>(2.0f, 1.5f);
}

/**
 * @brief Destructor cho LidarProcessor.
 */
LidarProcessor::~LidarProcessor() {
    stop_processing();
}

/**
 * @brief Khởi tạo kết nối UDP đến LiDAR.
 * @return true nếu khởi tạo thành công.
 */
bool LidarProcessor::initialize() {
    try {
        cout << "[SLAM-LIDAR] Khởi tạo SLAM LiDAR system..." << endl;
        lidar = make_unique<LakiBeamUDP>(local_ip, local_port, laser_ip, laser_port);
        cout << "[SLAM-LIDAR] ✅ Khởi tạo thành công!" << endl;
        return true;
    } catch (const exception& e) {
        cout << "[SLAM-LIDAR] ❌ Lỗi khởi tạo: " << e.what() << endl;
        return false;
    }
}

/**
 * @brief Bắt đầu vòng lặp xử lý dữ liệu LiDAR trong một luồng riêng.
 * @return true nếu có thể bắt đầu.
 */
bool LidarProcessor::start_processing() {
    if (!lidar) {
        cout << "[SLAM-LIDAR] ❌ LiDAR chưa được khởi tạo!" << endl;
        return false;
    }
    
    is_running = true;
    is_processing = true;
    cout << "[SLAM-LIDAR] 🚀 Bắt đầu SLAM processing..." << endl;
    return true;
}

/**
 * @brief Dừng vòng lặp xử lý dữ liệu LiDAR.
 */
void LidarProcessor::stop_processing() {
    is_running = false;
    is_processing = false;
    cout << "[SLAM-LIDAR] ⏹️ Dừng SLAM processing." << endl;
}

/**
 * @brief Đăng ký một hàm callback để nhận convex hull ổn định và pose.
 * @param callback Hàm callback có dạng `void(const vector<Point2D>&, const RobotPose&)`.
 */
void LidarProcessor::set_hull_callback(function<void(const vector<Point2D>&, const RobotPose&)> callback) {
    hull_callback = callback;
}

/**
 * @brief Đăng ký một hàm callback để nhận bản đồ (tập hợp các hull toàn cục).
 * @param callback Hàm callback có dạng `void(const vector<vector<Point2D>>&)`.
 */
void LidarProcessor::set_map_callback(function<void(const vector<vector<Point2D>>&)> callback) {
    map_callback = callback;
}

/**
 * @brief Lấy convex hull ổn định hiện tại trong tọa độ cục bộ.
 * @return Vector các điểm của hull ổn định cục bộ.
 */
vector<Point2D> LidarProcessor::get_current_stable_hull() const {
    if (hull_manager) {
        return hull_manager->get_stable_hull();
    }
    return {};
}

/**
 * @brief Lấy convex hull ổn định hiện tại trong tọa độ toàn cục.
 * @return Vector các điểm của hull ổn định toàn cục.
 */
vector<Point2D> LidarProcessor::get_global_hull() const {
    if (hull_manager) {
        return hull_manager->get_global_hull();
    }
    return {};
}

/**
 * @brief Lấy pose hiện tại của robot.
 * @return Pose hiện tại của robot.
 */
RobotPose LidarProcessor::get_current_pose() const {
    lock_guard<mutex> lock(pose_mutex);
    return current_robot_pose;
}

/**
 * @brief Lấy tất cả các hull đã được ánh xạ trong tọa độ toàn cục.
 * @return Vector chứa các hull toàn cục.
 */
vector<vector<Point2D>> LidarProcessor::get_mapped_hulls() const {
    if (slam_system) {
        return slam_system->get_all_global_hulls();
    }
    return {};
}

/**
 * @brief Kiểm tra xem kết nối LiDAR đã được khởi tạo chưa.
 * @return true nếu đã kết nối.
 */
bool LidarProcessor::is_lidar_connected() const {
    return lidar != nullptr;
}

/**
 * @brief Kiểm tra xem convex hull có đang ở trạng thái ổn định không.
 * @return true nếu ổn định.
 */
bool LidarProcessor::is_hull_stable() const {
    if (hull_manager) {
        return hull_manager->is_hull_stable();
    }
    return false;
}

/**
 * @brief Kiểm tra xem hệ thống SLAM có đang hoạt động không.
 * @return true nếu đang xử lý.
 */
bool LidarProcessor::is_slam_active() const {
    return is_processing.load();
}

/**
 * @brief Cập nhật thủ công pose của robot (ví dụ từ odometry).
 * @param new_pose Pose mới của robot.
 */
void LidarProcessor::update_robot_pose(const RobotPose& new_pose) {
    lock_guard<mutex> lock(pose_mutex);
    current_robot_pose = new_pose;
}

/**
 * @brief (Private) Lấy timestamp hiện tại dưới dạng milliseconds.
 * @return Timestamp hiện tại.
 */
long LidarProcessor::get_current_timestamp() const {
    auto now = chrono::system_clock::now();
    return chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
}

/**
 * @brief Vòng lặp chính xử lý dữ liệu LiDAR và SLAM.
 * 
 * Hàm này được thiết kế để chạy trong một luồng riêng biệt.
 */
void LidarProcessor::process_slam_lidar_data() {
    if (!lidar || !is_running) {
        cout << "[SLAM-LIDAR] ❌ Không thể bắt đầu - hệ thống chưa sẵn sàng" << endl;
        return;
    }
    
    cout << "[SLAM-LIDAR] 🗺️ SLAM LiDAR thread đã bắt đầu..." << endl;
    
    // Biến global_running cần được định nghĩa ở đâu đó trong chương trình chính
    // để có thể dừng tất cả các luồng một cách an toàn.
    extern volatile bool global_running; 
    
    while (is_running && global_running) {
        repark_t scan_data;
        
        if (lidar->get_repackedpack(scan_data)) {
            process_single_scan_with_slam(scan_data);
        } else {
            usleep(1000); // 1ms sleep to prevent busy-waiting
        }
    }
    
    is_processing = false;
    print_slam_status();
    cout << "[SLAM-LIDAR] 🏁 SLAM LiDAR thread đã kết thúc." << endl;
}

/**
 * @brief (Private) Xử lý một gói dữ liệu scan từ LiDAR với SLAM.
 * @param pack Gói dữ liệu thô `repark_t` từ LiDAR.
 */
void LidarProcessor::process_single_scan_with_slam(const repark_t& pack) {
    if (pack.maxdots == 0) return;
    
    scan_count++;
    
    // Chuyển đổi dữ liệu LiDAR thành điểm 2D với timestamp
    vector<Point2D> points = convert_lidar_to_points(pack);
    if (points.empty()) return;

    // Cập nhật pose và thêm scan mới vào SLAM
    RobotPose estimated_pose;
    if (slam_system->add_scan(points, estimated_pose)) {
        lock_guard<mutex> lock(pose_mutex);
        current_robot_pose = estimated_pose;
    }

    // Cập nhật hull manager
    if (hull_manager->update(points, get_current_pose())) {
        valid_hulls_count++;
        // Gọi callback nếu có hull ổn định mới
        if (hull_callback) {
            hull_callback(hull_manager->get_global_hull(), get_current_pose());
        }
    }

    // Gọi callback map mỗi 10 scan
    if (scan_count % 10 == 0 && map_callback) {
        map_callback(get_mapped_hulls());
    }
}

/**
 * @brief (Private) Chuyển đổi dữ liệu thô từ LiDAR thành một vector các điểm 2D.
 * @param pack Gói dữ liệu thô `repark_t` từ LiDAR.
 * @return Một vector các điểm `Point2D`.
 */
vector<Point2D> LidarProcessor::convert_lidar_to_points(const repark_t& pack) {
    vector<Point2D> points;
    points.reserve(pack.maxdots);

    for (u16_t i = 0; i < pack.maxdots; i++) {
        const auto& point = pack.dotcloud[i];
        
        // Lọc nhiễu và các điểm không hợp lệ
        if (point.distance > 10 && point.distance < 50000) { // 1cm to 50m
            float angle_deg = angle_to_degrees(point.angle);
            float distance_m = distance_to_meters(point.distance);
            
            float angle_rad = angle_deg * M_PI / 180.0f;
            float x = distance_m * cos(angle_rad);
            float y = distance_m * sin(angle_rad);
            
            points.emplace_back(x, y, distance_m);
        }
    }
    return points;
}

/**
 * @brief (Private) In ra trạng thái hiện tại của hệ thống SLAM.
 */
void LidarProcessor::print_slam_status() const {
    cout << "\n--- SLAM Status Summary ---" << endl;
    cout << "Total Scans Processed: " << get_scan_count() << endl;
    cout << "Valid Stable Hulls Found: " << get_valid_hulls_count() << endl;
    cout << "Loop Closures Detected: " << get_loop_closures_count() << endl;
    if (slam_system) {
        RobotPose p = get_current_pose();
        cout << "Final Pose (x, y, theta): (" 
             << fixed << setprecision(2) << p.x << ", " << p.y << ", " << p.theta 
             << ")" << endl;
    }
    cout << "---------------------------\n" << endl;
}
