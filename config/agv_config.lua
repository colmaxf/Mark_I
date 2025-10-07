-- backpack_2d.lua: Config cho AGV realtime mapping với LakiBeam 1
-- Spec LakiBeam 1: Range 0.05-25m, FOV 270°, 10-30Hz, 1440-3600 points/scan, intensity RSSI
-- Tối ưu di chuyển liên tục: update mỗi scan, downsample voxel, robust tracking

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = {
    use_trajectory_builder_2d = true,
    num_laser_scans = 0,  -- Point cloud từ LakiBeam UDP
    num_multi_echo_laser_scans = 0,
    num_subdivisions_per_laser_scan = 1,
    num_point_clouds = 1,  -- 1 LiDAR
    use_imu_data = false,  -- Không IMU
    use_odometry = false,  -- Không wheel odom (nếu có, set true)
    num_range_data = 1,
  },

  trajectory_builder_2d = {
    min_range = 0.05,      -- Min practical từ LakiBeam (m)
    max_range = 25.0,      -- Max theo spec LakiBeam 1 (m)
    missing_data_ray_length = 5.0,  -- Ray cho missing data
    use_imu_data = false,
    num_accumulated_range_data = 1,  -- Realtime: 1 scan/frame
    voxel_filter_size = 0.1,         -- Downsample 10cm (giảm noise motion)

    submaps = {
      num_range_data = 90,  -- Scans per submap (tăng nếu chậm)
      grid_options_2d = {
        grid_type = "PROBABILITY_GRID",  -- Occupancy dễ vẽ/send server
        resolution = 0.05,               -- 5cm/pixel, match angular res 0.1-0.25°
      },
      range_data_inserter = {
        range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
        probability_grid_range_data_inserter = {
          hit_probability = 0.55,        -- Threshold occupied (tùy RSSI intensity)
          miss_probability = 0.49,
        },
      },
      pose_graph_constraints = {
        sampling_ratio = 0.05,           -- Detect loops trong kho
        max_constraint_distance = 15.0,
        different_odom_frame = false,
      },
    },

    pose_extrapolator = {
      use_imu_based = false,
      constant_velocity_filter_max_distance = 0.5,  -- Extrapolate cho di chuyển nhanh
      constant_velocity_filter_max_time = 1.0,
    },

    ceres_solver_options = {
      use_nonmonotonic_steps = true,
      max_num_iterations = 10,
      num_threads = 4,  -- Pi 5 cores
    },
    real_time_correlative_scan_matcher = {
      linear_search_window = 0.5,      -- Window cho realtime motion
      branch_and_bound_depth = 7,
    },
  },

  pose_graph = {
    optimize_every_n_nodes = 90,
    constraint_builder = {
      min_score = 0.62,
      global_sampling_ratio = 0.003,
      match_global_tolerance = 3.0,
      loop_closure_search_radius = 1.5,  -- Radius cho kho rộng
    },
    max_num_final_iterations = 10,
    global_sampling_ratio = 0.003,
    log_residual_histograms = true,
  },

  max_num_laser_scans = 1,
  max_num_point_clouds = 1,
  num_background_threads = 4,
}

-- Global
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.5

POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options