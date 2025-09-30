#ifndef CARTOGRAPHER_STANDALONE_H
#define CARTOGRAPHER_STANDALONE_H

#include <memory>
#include <vector>
#include <mutex>

#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/transform.h"
#include <absl/time/time.h>  // Cho absl::Time
#include <cartographer/common/time.h>

#include "Lidarlib.h"
#include "../Serverlib/servercommunicator.h"

class CartographerStandalone {
public:
    struct GridMap {
        std::vector<uint8_t> data;  // 0=unknown, 1-127=free, 128-255=occupied
        int width, height;
        float resolution;  // meters per pixel
        float origin_x, origin_y;
    };

    CartographerStandalone();
    ~CartographerStandalone();
    
    bool Initialize(const std::string& configuration_directory,
                   const std::string& configuration_basename);
    
    // Add LiDAR scan
    void AddSensorData(const std::vector<LidarPoint>& scan_points);
    
    // Get current pose estimate
    ServerComm::AGVPose GetCurrentPose() const;
    
    // Get occupancy grid map
    GridMap GetOccupancyGrid() const;


    void UpdateOccupancyGrid();

    // Save/Load map
    bool SaveMap(const std::string& filename);
    bool LoadMap(const std::string& filename);
    
private:
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;
    int trajectory_id_;
    
    // Config
    cartographer::mapping::proto::MapBuilderOptions map_builder_options_;
    cartographer::mapping::proto::TrajectoryBuilderOptions trajectory_options_;
    
    // Current state
    ServerComm::AGVPose current_pose_;
    GridMap current_map_;
    mutable std::mutex data_mutex_;
    
    // Sensor name
    const std::string kRangeSensorId = "horizontal_laser";
    
    void LoadOptions(const std::string& configuration_directory,
                    const std::string& configuration_basename);
    
    cartographer::sensor::TimedPointCloudData ConvertToPointCloud(
        const std::vector<LidarPoint>& scan_points);
};

#endif // CARTOGRAPHER_STANDALONE_H

// Compare this snippet from Mark_I/SystemManager.cpp: