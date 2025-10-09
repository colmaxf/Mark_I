#include "cartographer_standalone.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/internal/voxel_filter.h"  // Cho downsampling

#include <absl/time/time.h>
#include <Eigen/Geometry>
#include <algorithm> // for std::min, std::max
#include "Lidarlib.h"  // Include để biết struct LidarPoint

/**
 * @brief Hàm khởi tạo mặc định.
 */
CartographerStandalone::CartographerStandalone() {
    current_pose_ = {0, 0, 0, 0, 0};
}

/**
 * @brief Hàm hủy, đảm bảo trajectory hiện tại được kết thúc một cách an toàn.
 */
CartographerStandalone::~CartographerStandalone() {
    if (map_builder_) {
        //map_builder_->FinishAllTrajectories();
        map_builder_->FinishTrajectory(trajectory_id_);
    }
}

/**
 * @brief Khởi tạo Cartographer với các tệp cấu hình.
 * @details Tải cấu hình từ tệp Lua, tạo MapBuilder, và bắt đầu một trajectory mới.
 * Thiết lập một callback để nhận cập nhật vị trí (pose) từ Cartographer.
 * @param configuration_directory Đường dẫn đến thư mục chứa các tệp cấu hình Lua.
 * @param configuration_basename Tên của tệp cấu hình chính (ví dụ: "agv_config.lua").
 * @return `true` nếu khởi tạo thành công và tạo được trajectory mới, ngược lại `false`.
 */
bool CartographerStandalone::Initialize(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
    
    // Load configuration from Lua file
    LoadOptions(configuration_directory, configuration_basename);
    
    // Create map builder
    map_builder_ = cartographer::mapping::CreateMapBuilder(map_builder_options_);
    
    // Start new trajectory
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> sensor_ids;
    sensor_ids.insert({cartographer::mapping::TrajectoryBuilderInterface::SensorId::SensorType::RANGE, 
                      kRangeSensorId});
    
    trajectory_id_ = map_builder_->AddTrajectoryBuilder(
        sensor_ids, trajectory_options_, 
        [this](const int /*trajectory_id*/,  // Unused
               const cartographer::common::Time time,
               const cartographer::transform::Rigid3d local_pose,
               cartographer::sensor::RangeData /*range_data*/,  // Unused
               const std::unique_ptr<const cartographer::mapping::TrajectoryBuilderInterface::InsertionResult>) {
            
            // Callback khi có pose mới
            std::lock_guard<std::mutex> lock(data_mutex_);
            
            // Convert 3D pose to 2D
            current_pose_.x = local_pose.translation().x();
            current_pose_.y = local_pose.translation().y();
            
            // Trích xuất góc yaw (xoay quanh trục Z) từ ma trận quay.
            const auto rotation_matrix = local_pose.rotation().toRotationMatrix();
            current_pose_.theta = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
            current_pose_.timestamp_us = cartographer::common::ToUniversal(time);  // Universal ns
            current_pose_.confidence = 1.0f;
        });
    
    return trajectory_id_ >= 0;
}

/**
 * @brief Thêm một frame dữ liệu quét LiDAR để xử lý SLAM.
 * @details Dữ liệu đầu vào được chuyển đổi sang định dạng của Cartographer, sau đó được
 * giảm mẫu (downsample) bằng VoxelFilter để giảm tải xử lý và nhiễu,
 * cuối cùng được đưa vào trajectory builder.
 * @param scan_points Vector chứa các điểm LiDAR thô từ một lần quét.
 */
void CartographerStandalone::AddSensorData(const std::vector<LidarPoint>& scan_points) {
    if (scan_points.empty()) return;
    
    // Convert to Cartographer format (với timestamp từ LidarPoint, intensity real)
    cartographer::sensor::TimedPointCloudData point_cloud = 
        ConvertToPointCloud(scan_points);
    
    // Áp dụng VoxelFilter để giảm số lượng điểm (downsampling), giúp giảm tải xử lý.
    cartographer::sensor::TimedPointCloud filtered_cloud = 
        cartographer::sensor::VoxelFilter(point_cloud.ranges, 0.05f);  // (data, resolution)
    
    cartographer::sensor::TimedPointCloudData filtered_point_cloud{
        point_cloud.time,
        point_cloud.origin,
        filtered_cloud,
        point_cloud.intensities
    };

    // Add to trajectory builder
    auto* trajectory_builder = map_builder_->GetTrajectoryBuilder(trajectory_id_);
    if (trajectory_builder) {
        trajectory_builder->AddSensorData(kRangeSensorId, point_cloud);
    }
}

/**
 * @brief Cập nhật bản đồ lưới chiếm dụng.
 * @details Hàm này là một alias cho `UpdateOccupancyGrid`. Nên được gọi định kỳ.
 */
void CartographerStandalone::UpdateMapRealtime() {
    UpdateOccupancyGrid();  // Gọi realtime nếu stable
}

/**
 * @brief Chuyển đổi vector `LidarPoint` sang định dạng `TimedPointCloudData` của Cartographer.
 * @details Lấy timestamp từ điểm đầu tiên của scan. Duyệt qua tất cả các điểm, chuyển đổi
 * tọa độ và thêm vào danh sách điểm của Cartographer. Cường độ (intensity) cũng được
 * trích xuất và lưu lại.
 * @param scan_points Vector các điểm LiDAR ở định dạng tùy chỉnh.
 * @return Dữ liệu đám mây điểm có dấu thời gian theo định dạng của Cartographer.
 */
cartographer::sensor::TimedPointCloudData
CartographerStandalone::ConvertToPointCloud(const std::vector<LidarPoint>& scan_points) {
    
    // Timestamp từ first point (realtime sync)
    long first_ts = scan_points.front().timestamp * 1000;  // ms → ns
    cartographer::common::Time time = cartographer::common::FromUniversal(first_ts);
    
    // Convert points với intensity real
    // cartographer::sensor::TimedPointCloud points;
    // points.reserve(scan_points.size());
    cartographer::sensor::TimedPointCloud ranges;
    std::vector<float> intensities;
    ranges.reserve(scan_points.size());
    intensities.reserve(scan_points.size());
    
    for (const auto& p : scan_points) {
        if (!p.isValid()) continue;
        ranges.push_back({
            Eigen::Vector3f(p.x, p.y, 0.0f),
            0.0f  // time offset
        });
        intensities.push_back(static_cast<float>(p.intensity));
    }
    
    return cartographer::sensor::TimedPointCloudData{
        time, 
        Eigen::Vector3f::Zero(),
        ranges,
        intensities
    };
}

/**
 * @brief Tạo/cập nhật bản đồ lưới chiếm dụng từ tất cả các submap đã có.
 * @details Hàm này thực hiện các bước sau:
 * 1. Lấy tất cả dữ liệu submap từ pose graph.
 * 2. Tính toán bounding box (khung bao) tổng thể chứa tất cả các submap để xác định kích thước và gốc của bản đồ cuối cùng.
 * 3. Khởi tạo một `GridMap` mới với kích thước và thông tin vừa tính toán.
 * 4. Duyệt qua từng submap, chuyển đổi tọa độ các ô của nó sang hệ tọa độ thế giới và vẽ chúng lên `GridMap` chung.
 *    Sử dụng `std::max` để hợp nhất, đảm bảo giá trị xác suất chiếm dụng cao nhất được giữ lại.
 */
void CartographerStandalone::UpdateOccupancyGrid() {
    // Get all submaps
    auto all_submaps = map_builder_->pose_graph()->GetAllSubmapData();
    if (all_submaps.empty()) return;
    
    const float resolution = 0.05f;  // 5cm
    // Dynamic size: Tính bounding box từ submaps
    Eigen::Vector2d global_min(1e6, 1e6), global_max(-1e6, -1e6);

    for (const auto& submap_id_data : all_submaps) {
        const auto& pose = submap_id_data.data.pose;
        const auto* submap_2d = dynamic_cast<const cartographer::mapping::Submap2D*>(submap_id_data.data.submap.get());
        if (!submap_2d) continue;
        const auto* grid = dynamic_cast<const cartographer::mapping::ProbabilityGrid*>(submap_2d->grid());
        if (!grid) continue;
        const auto& limits = grid->limits();

        Eigen::Vector2d submap_min(
            limits.max().x() - limits.cell_limits().num_x_cells * resolution,
            limits.max().y() - limits.cell_limits().num_y_cells * resolution
        );
        Eigen::Vector2d submap_max = limits.max();

        global_min = global_min.cwiseMin(submap_min + pose.translation().head<2>());
        global_max = global_max.cwiseMax(submap_max + pose.translation().head<2>());
    }

    int width = static_cast<int>((global_max.x() - global_min.x()) / resolution) + 1;
    int height = static_cast<int>((global_max.y() - global_min.y()) / resolution) + 1;

    if (width <= 0 || height <= 0) return; // Kích thước không hợp lệ
    width = std::min(width, 4000);  // Cap 200m max để tránh OOM
    height = std::min(height, 4000);
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_map_.width = width;
    current_map_.height = height;
    current_map_.resolution = resolution;
    current_map_.origin_x = global_min.x();
    current_map_.origin_y = global_min.y();
    current_map_.data.assign(width * height, 0);  // Unknown = 0
    
    // Hợp nhất các submap: duyệt qua từng ô của mỗi submap
    for (const auto& submap_id_data : all_submaps) {
        const auto& submap_pose = submap_id_data.data.pose;
        const auto* submap_2d = dynamic_cast<const cartographer::mapping::Submap2D*>(submap_id_data.data.submap.get());
        if (!submap_2d) continue;
        const auto* grid = dynamic_cast<const cartographer::mapping::ProbabilityGrid*>(submap_2d->grid());
        if (!grid) continue;
        
        const auto& limits = grid->limits();
        // Duyệt qua các ô có xác suất > 0 (bị chiếm/trống) để tối ưu hóa
        for (int cell_y = 0; cell_y < limits.cell_limits().num_y_cells; ++cell_y) {
            for (int cell_x = 0; cell_x < limits.cell_limits().num_x_cells; ++cell_x) {
                float prob = grid->GetProbability(Eigen::Array2i(cell_x, cell_y));
                if (prob == 0.f) continue;  // Skip empty/unknown
                
                // World coord từ cell
                Eigen::Vector2d world_xy(
                    limits.max().x() - (limits.cell_limits().num_x_cells - cell_x) * resolution,
                    limits.max().y() - (limits.cell_limits().num_y_cells - cell_y) * resolution
                );
                
                Eigen::Vector3d global_xyz = submap_pose * Eigen::Vector3d(world_xy.x(), world_xy.y(), 0);
                
                // Grid index
                int grid_x = static_cast<int>((global_xyz.x() - current_map_.origin_x) / resolution);
                int grid_y = static_cast<int>((global_xyz.y() - current_map_.origin_y) / resolution);
                
                if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                    uint8_t occ = static_cast<uint8_t>(prob * 255);  // 0-255, >127 occupied
                    int index = grid_y * width + grid_x;
                    current_map_.data[index] = std::max(current_map_.data[index], occ);  // Hợp nhất bằng cách lấy giá trị xác suất lớn nhất
                }
            }
        }
    }
}

ServerComm::AGVPose CartographerStandalone::GetCurrentPose() const {
/**
 * @brief Lấy vị trí và hướng (pose) hiện tại của robot.
 * @return Cấu trúc `AGVPose` chứa thông tin vị trí.
 */
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_pose_;
}

/**
 * @brief Lấy bản đồ lưới chiếm dụng hiện tại.
 * @return Cấu trúc `GridMap` chứa dữ liệu bản đồ.
 */
CartographerStandalone::GridMap CartographerStandalone::GetOccupancyGrid() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_map_;
}

/**
 * @brief Chuyển đổi dữ liệu bản đồ sang định dạng byte nhị phân để gửi qua mạng.
 * @param grid Đối tượng `GridMap` cần được serialize.
 * @return Vector các byte chứa dữ liệu bản đồ đã được serialize.
 */
std::vector<uint8_t> CartographerStandalone::SerializeGridToBytes(const GridMap& grid) const {
    std::vector<uint8_t> bytes;
    // Header: width(4B), height(4B), res(4B float), origin_x(4B), origin_y(4B)
    bytes.insert(bytes.end(), reinterpret_cast<const uint8_t*>(&grid.width), reinterpret_cast<const uint8_t*>(&grid.width) + sizeof(int));
    bytes.insert(bytes.end(), reinterpret_cast<const uint8_t*>(&grid.height), reinterpret_cast<const uint8_t*>(&grid.height) + sizeof(int));
    float res = grid.resolution;
    bytes.insert(bytes.end(), reinterpret_cast<const uint8_t*>(&res), reinterpret_cast<const uint8_t*>(&res) + sizeof(float));
    float ox = grid.origin_x, oy = grid.origin_y;
    bytes.insert(bytes.end(), reinterpret_cast<const uint8_t*>(&ox), reinterpret_cast<const uint8_t*>(&ox) + sizeof(float));
    bytes.insert(bytes.end(), reinterpret_cast<const uint8_t*>(&oy), reinterpret_cast<const uint8_t*>(&oy) + sizeof(float));
    // Data
    bytes.insert(bytes.end(), grid.data.begin(), grid.data.end());
    return bytes;
}

/**
 * @brief Tải các tùy chọn cấu hình từ tệp Lua.
 */
void CartographerStandalone::LoadOptions(const std::string& configuration_directory,
                                        const std::string& configuration_basename) {
    auto file_resolver = std::make_unique<cartographer::common::ConfigurationFileResolver>(
        std::vector<std::string>{configuration_directory});
    const std::string code = file_resolver->GetFileContentOrDie(configuration_basename);
    cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
        code, std::move(file_resolver));
    map_builder_options_ = cartographer::mapping::CreateMapBuilderOptions(
        lua_parameter_dictionary.GetDictionary("MAP_BUILDER").get());
    trajectory_options_ = cartographer::mapping::CreateTrajectoryBuilderOptions(
        lua_parameter_dictionary.GetDictionary("TRAJECTORY_BUILDER").get());
}

/**
 * @brief Lưu trạng thái hiện tại của map (pose graph) vào một tệp.
 * @param filename Đường dẫn đến tệp để lưu.
 * @return `true` nếu lưu thành công.
 */
bool CartographerStandalone::SaveMap(const std::string& filename) {
    cartographer::io::ProtoStreamWriter writer(filename);
    map_builder_->SerializeState(false, &writer);  // false = không include unfinished submaps
    return true;
}

/**
 * @brief Tải trạng thái map từ một tệp đã lưu.
 * @param filename Đường dẫn đến tệp để tải.
 * @return `true` nếu tải thành công.
 */
bool CartographerStandalone::LoadMap(const std::string& filename) {
    cartographer::io::ProtoStreamReader reader(filename);
    
    map_builder_->LoadState(&reader, true);  // true = load frozen state
    return true;
}