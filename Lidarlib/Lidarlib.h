#ifndef LIDAR_PROCESSOR_H
#define LIDAR_PROCESSOR_H

#include "Data_SDK/LakiBeamUDP.h"
#include <vector>
#include <deque>
#include <mutex>
#include <cmath>
#include <atomic>
#include <functional>

// Cấu trúc điểm 2D
struct Point2D {
    float x, y;
    float distance;
    
    Point2D(float x = 0, float y = 0, float dist = 0) : x(x), y(y), distance(dist) {}
    
    bool operator<(const Point2D& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
    
    bool operator==(const Point2D& other) const {
        return abs(x - other.x) < 1e-6 && abs(y - other.y) < 1e-6;
    }
};

// Pose của robot (vị trí và hướng)
struct RobotPose {
    float x, y, theta;  // x, y (meters), theta (radians)
    long timestamp;
    
    RobotPose(float x = 0, float y = 0, float theta = 0, long ts = 0)
        : x(x), y(y), theta(theta), timestamp(ts) {}
};

// Node trong pose graph
struct PoseGraphNode {
    RobotPose pose;
    std::vector<Point2D> scan_points;
    std::vector<Point2D> convex_hull;
    int node_id;
    
    PoseGraphNode(const RobotPose& p, const std::vector<Point2D>& points, int id)
        : pose(p), scan_points(points), node_id(id) {}
};

// Đơn giản hóa SLAM algorithm
class SimpleSLAM {
private:
    std::vector<std::shared_ptr<PoseGraphNode>> pose_graph;
    RobotPose current_pose;
    RobotPose last_pose;
    
    // SLAM parameters
    float max_correspondence_distance;
    float loop_closure_threshold;
    int next_node_id;
    
    mutable std::mutex slam_mutex;
    
public:
    SimpleSLAM(float max_corr_dist = 2.0f, float loop_threshold = 1.0f);
    
    // Thêm scan mới và cập nhật pose
    bool add_scan(const std::vector<Point2D>& scan_points, RobotPose& estimated_pose);
    
    // Scan matching để estimate pose
    RobotPose estimate_pose_from_scan(const std::vector<Point2D>& current_scan);
    
    // Tìm loop closure
    int find_loop_closure(const RobotPose& current_pose);
    
    // Transform điểm từ local frame sang global frame
    std::vector<Point2D> transform_points_to_global(const std::vector<Point2D>& local_points, 
                                                   const RobotPose& robot_pose);
    
    // Lấy tất cả convex hull đã mapped
    std::vector<std::vector<Point2D>> get_all_global_hulls() const;
    
    // Lấy pose hiện tại
    RobotPose get_current_pose() const;
    
    // Lấy pose graph
    std::vector<std::shared_ptr<PoseGraphNode>> get_pose_graph() const;
    
private:
    // ICP (Iterative Closest Point) đơn giản
    RobotPose simple_icp(const std::vector<Point2D>& source, 
                        const std::vector<Point2D>& target);
    
    // Tính distance giữa hai pose
    float pose_distance(const RobotPose& p1, const RobotPose& p2);
    
    // Transform một điểm
    Point2D transform_point(const Point2D& point, const RobotPose& pose);
};

// Class quản lý convex hull với SLAM
class SlamConvexHullManager {
private:
    std::deque<std::vector<Point2D>> hull_history;
    std::vector<Point2D> stable_hull;
    std::vector<Point2D> global_hull;  // Hull trong global coordinate
    
    int max_history_size;
    float stability_threshold;
    bool is_stable;
    int stable_count;
    mutable std::mutex hull_mutex;
    
public:
    SlamConvexHullManager(int history_size = 10, float threshold = 0.1);
    
    // Cập nhật với pose thông tin
    bool update(const std::vector<Point2D>& new_hull, const RobotPose& robot_pose);
    
    // Lấy hull trong local coordinate
    std::vector<Point2D> get_stable_hull() const;
    
    // Lấy hull trong global coordinate
    std::vector<Point2D> get_global_hull() const;
    
    bool is_hull_stable() const;
    
private:
    std::vector<Point2D> compute_average_hull();
    bool is_hull_stable_internal(const std::vector<Point2D>& new_hull);
    std::vector<Point2D> compute_convex_hull(std::vector<Point2D> points);
    float cross_product(const Point2D& O, const Point2D& A, const Point2D& B);
    
    // Transform hull sang global coordinate
    std::vector<Point2D> transform_hull_to_global(const std::vector<Point2D>& local_hull, 
                                                 const RobotPose& robot_pose);
};

// Class chính xử lý LiDAR
class LidarProcessor {
private:
    std::unique_ptr<LakiBeamUDP> lidar;
    std::unique_ptr<SlamConvexHullManager> hull_manager;
    std::unique_ptr<SimpleSLAM> slam_system;
    
    // Cấu hình LiDAR
    std::string local_ip, local_port, laser_ip, laser_port;
    
    // Thread control
    std::atomic<bool> is_running;
    std::atomic<bool> is_processing;
    
    // Callbacks
    std::function<void(const std::vector<Point2D>&, const RobotPose&)> hull_callback;
    std::function<void(const std::vector<std::vector<Point2D>>&)> map_callback;
    
    // Thống kê
    std::atomic<int> scan_count;
    std::atomic<int> valid_hulls_count;
    std::atomic<int> loop_closures_count;
    
    // Pose tracking
    RobotPose current_robot_pose;
    mutable std::mutex pose_mutex;  
    
public:
    LidarProcessor(const std::string& local_ip = "192.168.3.10",
                       const std::string& local_port = "2368",
                       const std::string& laser_ip = "192.168.3.101",
                       const std::string& laser_port = "2368");
    
    ~LidarProcessor();
    
    // Khởi tạo hệ thống
    bool initialize();
    
    // Bắt đầu/dừng xử lý
    bool start_processing();
    void stop_processing();
    
    // Đăng ký callbacks
    void set_hull_callback(std::function<void(const std::vector<Point2D>&, const RobotPose&)> callback);
    void set_map_callback(std::function<void(const std::vector<std::vector<Point2D>>&)> callback);
    
    // Lấy thông tin hiện tại
    std::vector<Point2D> get_current_stable_hull() const;
    std::vector<Point2D> get_global_hull() const;
    RobotPose get_current_pose() const;
    std::vector<std::vector<Point2D>> get_mapped_hulls() const;
    
    // Kiểm tra trạng thái
    bool is_lidar_connected() const;
    bool is_hull_stable() const;
    bool is_slam_active() const;
    
    // Thống kê
    int get_scan_count() const { return scan_count.load(); }
    int get_valid_hulls_count() const { return valid_hulls_count.load(); }
    int get_loop_closures_count() const { return loop_closures_count.load(); }
    
    // Hàm xử lý chính (chạy trong thread)
    void process_slam_lidar_data();
    
    // Manual pose correction (nếu có odometry từ robot)
    void update_robot_pose(const RobotPose& new_pose);
    
private:
    // Xử lý một scan với SLAM
    void process_single_scan_with_slam(const repark_t& pack);
    
    // Chuyển đổi dữ liệu LiDAR thành điểm 2D
    std::vector<Point2D> convert_lidar_to_points(const repark_t& pack);
    
    // Utility functions
    float angle_to_degrees(u16_t angle) const { return angle / 100.0f; }
    float distance_to_meters(u16_t distance) const { return distance / 1000.0f; }
    long get_current_timestamp() const;
    
    // Visualization helper
    void print_slam_status() const;
};

#endif // LIDAR_PROCESSOR_H