-- 包含地图构建器和轨迹构建器的基础配置
include "map_builder.lua"
include "trajectory_builder.lua"

-- 全局配置选项
options = {
  -- 指定地图构建器和轨迹构建器
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- 坐标系设置
  map_frame = "map",            -- 全局地图坐标系
  tracking_frame = "base_link", -- 机器人跟踪坐标系
  published_frame = "base_link",-- 发布的坐标变换源坐标系
  odom_frame = "odom",          -- 里程计坐标系
  provide_odom_frame = true,    -- 是否提供里程计坐标系
  publish_frame_projected_to_2d = false, -- 是否将坐标系投影到2D平面

  -- 传感器使用设置
  use_odometry = false,         -- 是否使用里程计数据
  use_nav_sat = false,          -- 是否使用GPS数据
  use_landmarks = false,        -- 是否使用地标数据

  -- 激光雷达配置
  num_laser_scans = 1,          -- 使用1个单线激光雷达
  num_multi_echo_laser_scans = 0, -- 不使用多回波激光雷达
  -- num_subdivisions_per_laser_scan = 1, -- 每次激光扫描的细分次数
  num_subdivisions_per_laser_scan = 2,

  -- 点云数据配置
  num_point_clouds = 0,         -- 不使用点云数据

  -- 时间和频率设置
  lookup_transform_timeout_sec = 0.2,     -- 坐标变换查找超时时间(秒)
  submap_publish_period_sec = 0.3,        -- 子地图发布周期(秒)
  pose_publish_period_sec = 5e-3,         -- 位姿发布周期(秒)
  trajectory_publish_period_sec = 30e-3,  -- 轨迹发布周期(秒)

  -- 传感器数据采样率
  rangefinder_sampling_ratio = 1.,        -- 激光雷达数据采样率(1=全部使用)
  odometry_sampling_ratio = 1.,           -- 里程计数据采样率
  fixed_frame_pose_sampling_ratio = 1.,   -- 固定坐标系位姿采样率
  imu_sampling_ratio = 1.,                -- IMU数据采样率
  landmarks_sampling_ratio = 1.,          -- 地标数据采样率
}

-- 使用2D轨迹构建器
MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D轨迹构建器配置
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35,      -- 每个子地图包含的激光扫描帧数
TRAJECTORY_BUILDER_2D.min_range = 0.3,                  -- 最小有效测量距离(米)
TRAJECTORY_BUILDER_2D.max_range = 8.,                   -- 最大有效测量距离(米)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.,     -- 缺失数据的射线长度(米)
TRAJECTORY_BUILDER_2D.use_imu_data = false,             -- 不使用IMU数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true, -- 启用在线扫描匹配
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1, -- 线性搜索窗口(米)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10., -- 平移变化权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1, -- 旋转变化权重

-- 位姿图优化配置
POSE_GRAPH.optimization_problem.huber_scale = 1e2,      -- Huber损失函数尺度参数
POSE_GRAPH.optimize_every_n_nodes = 35,                 -- 每添加n个节点执行一次优化
POSE_GRAPH.constraint_builder.min_score = 0.65,         -- 约束创建所需的最小匹配分数

return options  -- 返回配置选项
