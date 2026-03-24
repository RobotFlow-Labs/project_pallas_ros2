#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <string>

namespace anima::pallas {

struct PipelineConfig {
  std::string lidar_type{"generic"};
  double stationary_init_sec{2.0};
  double gravity_mps2{9.80665};
  double min_range_m{1.0};
  double max_range_m{200.0};
  double scan_voxel_size_m{0.10};
  double map_voxel_size_m{0.20};
  std::size_t max_imu_buffer_size{20000};
  std::size_t max_map_scans{20};
  std::size_t map_publish_period_scans{1};
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
