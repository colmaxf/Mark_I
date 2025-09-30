#include "cartographer_standalone.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/common/time.h"

#include <absl/time/time.h>

CartographerStandalone::CartographerStandalone() {
    current_pose_ = {0, 0, 0, 0, 0};
}

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
            
            // Extract yaw from rotation matrix
            const auto& rotation = local_pose.rotation();
            current_pose_.theta = std::atan2(rotation.matrix()(1, 0), 
                                            rotation.matrix()(0, 0));
            current_pose_.timestamp_us = cartographer::common::ToUniversal(time) / 1000;  // Convert to microseconds
            current_pose_.confidence = 1.0f;
        });
    
    return trajectory_id_ >= 0;
}

void CartographerStandalone::AddSensorData(const std::vector<LidarPoint>& scan_points) {
    if (scan_points.empty()) return;
    
    // Convert to Cartographer format
    cartographer::sensor::TimedPointCloudData point_cloud = 
        ConvertToPointCloud(scan_points);
    
    // Add to trajectory builder
    auto* trajectory_builder = map_builder_->GetTrajectoryBuilder(trajectory_id_);
    if (trajectory_builder) {
        trajectory_builder->AddSensorData(kRangeSensorId, point_cloud);
    }
    
    // Update map periodically
    static int frame_count = 0;
    if (++frame_count % 30 == 0) {  // Every 30 frames
        UpdateOccupancyGrid();
    }
}

cartographer::sensor::TimedPointCloudData 
CartographerStandalone::ConvertToPointCloud(const std::vector<LidarPoint>& scan_points) {
    
    // Get current time
    cartographer::common::Time time = cartographer::common::FromUniversal(
        absl::ToUnixMicros(absl::Now()) * 1000);  // Convert absl::Time to cartographer::common::Time
    
    // Convert points
    cartographer::sensor::TimedPointCloud points;
    points.reserve(scan_points.size());
    
    for (const auto& p : scan_points) {
        // Convert to 3D point (z=0 for 2D lidar)
        points.push_back({Eigen::Vector3f(p.x, p.y, 0.0f), 0.0f});
    }
    
    return cartographer::sensor::TimedPointCloudData{
        time, 
        Eigen::Vector3f::Zero(),
        points,
        {}  // intensities empty
    };
}

void CartographerStandalone::UpdateOccupancyGrid() {
    // Get all submaps
    auto all_trajectory_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
    auto all_submaps = map_builder_->pose_graph()->GetAllSubmapData();
    
    if (all_submaps.empty()) return;
    
    // Create occupancy grid from submaps
    const float resolution = 0.05f;  // 5cm per pixel
    const int map_size = 2000;  // 100m x 100m
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_map_.width = map_size;
    current_map_.height = map_size;
    current_map_.resolution = resolution;
    current_map_.origin_x = -map_size * resolution / 2;
    current_map_.origin_y = -map_size * resolution / 2;
    current_map_.data.assign(map_size * map_size, 0);  // Initialize as unknown (0)
    
    // Merge all submaps into single grid
    for (const auto& submap_id_data : all_submaps) {
        const auto& submap = submap_id_data.data.submap;
        
        // Cast to 2D submap
        const auto* submap_2d = 
            dynamic_cast<const cartographer::mapping::Submap2D*>(submap.get());
        if (!submap_2d) continue;
        
        // Get probability grid
        const auto* grid = dynamic_cast<const cartographer::mapping::ProbabilityGrid*>(submap_2d->grid());
        if (!grid) continue;
        
        const auto& limits = grid->limits();
        const auto& cell_limits = limits.cell_limits();
        
        // Get submap pose
        const auto submap_pose = submap_id_data.data.pose;
        const Eigen::Vector2f submap_translation(submap_pose.translation().x(), submap_pose.translation().y());
        const float submap_rotation = std::atan2(submap_pose.rotation().matrix()(1, 0), 
                                                 submap_pose.rotation().matrix()(0, 0));
        const float cos_theta = std::cos(submap_rotation);
        const float sin_theta = std::sin(submap_rotation);
        
        // Convert submap grid to global grid
        for (int i = 0; i < cell_limits.num_x_cells; ++i) {
            for (int j = 0; j < cell_limits.num_y_cells; ++j) {
                // Get probability value
                float value = grid->GetProbability(Eigen::Array2i(i, j));
                uint8_t occ_value = static_cast<uint8_t>(value * 255);  // 0=free, 255=occupied
                
                // Convert submap cell coordinates to world coordinates
                Eigen::Vector2f submap_point;
                // Tính tọa độ thế giới dựa trên resolution và max của MapLimits
                double resolution = limits.resolution();
                Eigen::Vector2d max = limits.max();
                submap_point.x() = max.x() - (cell_limits.num_x_cells - (i + 0.5)) * resolution;
                submap_point.y() = max.y() - (cell_limits.num_y_cells - (j + 0.5)) * resolution;
                
                // Apply submap pose transformation
                float global_x = submap_translation.x() + (submap_point.x() * cos_theta - submap_point.y() * sin_theta);
                float global_y = submap_translation.y() + (submap_point.x() * sin_theta + submap_point.y() * cos_theta);
                
                // Map to global grid indices
                int grid_x = static_cast<int>((global_x - current_map_.origin_x) / resolution);
                int grid_y = static_cast<int>((global_y - current_map_.origin_y) / resolution);
                
                // Check bounds
                if (grid_x >= 0 && grid_x < map_size && grid_y >= 0 && grid_y < map_size) {
                    int index = grid_y * map_size + grid_x;
                    current_map_.data[index] = occ_value;  // Store occupancy value
                }
            }
        }
    }
}

ServerComm::AGVPose CartographerStandalone::GetCurrentPose() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_pose_;
}

CartographerStandalone::GridMap CartographerStandalone::GetOccupancyGrid() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_map_;
}