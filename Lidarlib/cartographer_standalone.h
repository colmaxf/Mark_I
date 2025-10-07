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
#include "cartographer/sensor/internal/voxel_filter.h"  // Thêm cho downsampling
#include "cartographer/transform/transform.h"
#include <absl/time/time.h>
#include <cartographer/common/time.h>

//#include "Lidarlib.h"
#include "../Serverlib/servercommunicator.h"


struct LidarPoint;  // Forward decl: Compiler biết type tồn tại, không cần full def ở header

/**
 * @class CartographerStandalone
 * @brief Lớp bao bọc (wrapper) cho thư viện Google Cartographer để thực hiện SLAM 2D.
 * @details Lớp này đơn giản hóa việc sử dụng Cartographer bằng cách cung cấp một giao diện
 * dễ sử dụng để khởi tạo, thêm dữ liệu cảm biến (LiDAR), và lấy ra kết quả là
 * vị trí (pose) của robot và bản đồ lưới chiếm dụng (occupancy grid).
 */
class CartographerStandalone {
public:
    /**
     * @struct GridMap
     * @brief Cấu trúc dữ liệu đại diện cho bản đồ lưới chiếm dụng (occupancy grid).
     */
    struct GridMap {
        std::vector<uint8_t> data;  ///< Dữ liệu bản đồ dưới dạng mảng 1D. Giá trị từ 0 (chưa biết/trống) đến 255 (bị chiếm).
        int width;                  ///< Chiều rộng của bản đồ (số ô).
        int height;                 ///< Chiều cao của bản đồ (số ô).
        float resolution;           ///< Độ phân giải của bản đồ (mét trên mỗi ô/pixel).
        float origin_x;             ///< Tọa độ X của góc dưới bên trái bản đồ trong hệ tọa độ thế giới (mét).
        float origin_y;             ///< Tọa độ Y của góc dưới bên trái bản đồ trong hệ tọa độ thế giới (mét).
    };

    /**
     * @brief Hàm khởi tạo mặc định.
     */
    CartographerStandalone();

    /**
     * @brief Hàm hủy.
     * @details Đảm bảo rằng trajectory hiện tại được kết thúc một cách an toàn.
     */
    ~CartographerStandalone();
    
    /**
     * @brief Khởi tạo Cartographer với các tệp cấu hình.
     * @param configuration_directory Đường dẫn đến thư mục chứa các tệp cấu hình Lua.
     * @param configuration_basename Tên của tệp cấu hình chính (ví dụ: "agv_config.lua").
     * @return `true` nếu khởi tạo thành công và tạo được trajectory mới, ngược lại `false`.
     */
    bool Initialize(const std::string& configuration_directory,
                   const std::string& configuration_basename);
    
    /**
     * @brief Thêm một frame dữ liệu quét LiDAR để xử lý SLAM.
     * @details Dữ liệu sẽ được chuyển đổi, lọc nhiễu (downsample) và đưa vào trajectory builder.
     * @param scan_points Vector chứa các điểm LiDAR thô từ một lần quét.
     */
    void AddSensorData(const std::vector<LidarPoint>& scan_points);
    
    /**
     * @brief Cập nhật bản đồ lưới chiếm dụng.
     * @details Hàm này là một alias cho `UpdateOccupancyGrid`. Nên được gọi định kỳ.
     */
    void UpdateMapRealtime();

    /**
     * @brief Tạo/cập nhật bản đồ lưới chiếm dụng từ tất cả các submap đã có.
     * @details Hàm này duyệt qua tất cả các submap trong pose graph, tính toán kích thước
     * tổng thể của bản đồ, và vẽ lại từng submap lên một bản đồ chung.
     */
    void UpdateOccupancyGrid();
    
    /**
     * @brief Lấy vị trí và hướng (pose) hiện tại của robot.
     * @details Dữ liệu được bảo vệ bởi mutex để đảm bảo an toàn luồng.
     * @return Cấu trúc `AGVPose` chứa thông tin vị trí.
     */
    ServerComm::AGVPose GetCurrentPose() const;
    
    /**
     * @brief Lấy bản đồ lưới chiếm dụng hiện tại.
     * @details Dữ liệu được bảo vệ bởi mutex để đảm bảo an toàn luồng.
     * @return Cấu trúc `GridMap` chứa dữ liệu bản đồ.
     */
    GridMap GetOccupancyGrid() const;

    /**
     * @brief Chuyển đổi dữ liệu bản đồ sang định dạng byte nhị phân để gửi qua mạng.
     * @param grid Đối tượng `GridMap` cần được serialize.
     * @return Vector các byte chứa dữ liệu bản đồ đã được serialize.
     */
    std::vector<uint8_t> SerializeGridToBytes(const GridMap& grid) const;

    /**
     * @brief Lưu trạng thái hiện tại của map (pose graph) vào một tệp.
     * @param filename Đường dẫn đến tệp để lưu.
     * @return `true` nếu lưu thành công.
     */
    bool SaveMap(const std::string& filename);

    /**
     * @brief Tải trạng thái map từ một tệp đã lưu.
     * @param filename Đường dẫn đến tệp để tải.
     * @return `true` nếu tải thành công.
     */
    bool LoadMap(const std::string& filename);
    
private:
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_; ///< Đối tượng chính của Cartographer để xây dựng bản đồ.
    int trajectory_id_; ///< ID của trajectory đang được xây dựng.
    
    // Cấu hình
    cartographer::mapping::proto::MapBuilderOptions map_builder_options_; ///< Tùy chọn cho MapBuilder, được tải từ tệp Lua.
    cartographer::mapping::proto::TrajectoryBuilderOptions trajectory_options_; ///< Tùy chọn cho TrajectoryBuilder, được tải từ tệp Lua.
    
    // Trạng thái hiện tại
    ServerComm::AGVPose current_pose_; ///< Vị trí và hướng hiện tại của robot.
    GridMap current_map_;              ///< Bản đồ lưới chiếm dụng hiện tại.
    mutable std::mutex data_mutex_;    ///< Mutex để bảo vệ truy cập vào `current_pose_` và `current_map_`.
    
    const std::string kRangeSensorId = "horizontal_laser"; ///< Tên định danh cho cảm biến LiDAR.
    
    /** @brief Tải các tùy chọn cấu hình từ tệp Lua. */
    void LoadOptions(const std::string& configuration_directory,
                    const std::string& configuration_basename);
    
    /** @brief Chuyển đổi vector `LidarPoint` sang định dạng `TimedPointCloudData` của Cartographer. */
    cartographer::sensor::TimedPointCloudData ConvertToPointCloud(
        const std::vector<LidarPoint>& scan_points);
};

#endif // CARTOGRAPHER_STANDALONE_H