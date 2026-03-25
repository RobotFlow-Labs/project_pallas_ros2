#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <string>

namespace anima::pallas {

struct PipelineConfig {
  std::string lidar_type{"generic"};
  double stationary_init_sec{2.0};
  double gravity_mps2{9.80665};
  double max_accel_std_mps2{0.25};
  double max_gyro_std_radps{0.03};
  double max_accel_norm_error_mps2{0.75};
  double min_range_m{1.0};
  double max_range_m{200.0};
  double scan_voxel_size_m{0.10};
  double scan_normal_radius_m{0.30};
  std::size_t scan_min_points_for_normal{6};
  double map_voxel_size_m{0.20};
  std::size_t max_imu_buffer_size{20000};
  std::size_t map_max_surfels{50000};
  double map_max_age_sec{60.0};
  std::size_t map_publish_period_scans{1};
  std::size_t ct_min_control_points{4};
  std::size_t ct_max_control_points{64};
  Eigen::Isometry3d sensor_to_body{Eigen::Isometry3d::Identity()};
};

inline Eigen::Isometry3d MakeIsometry(
  const Eigen::Vector3d& translation,
  const Eigen::Vector3d& rpy_rad)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = translation;
  tf.linear() =
    Eigen::AngleAxisd(rpy_rad.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    Eigen::AngleAxisd(rpy_rad.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() *
    Eigen::AngleAxisd(rpy_rad.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
  return tf;
}

}  // namespace anima::pallas
