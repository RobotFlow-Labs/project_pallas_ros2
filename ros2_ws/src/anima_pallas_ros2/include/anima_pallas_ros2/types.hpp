#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace anima::pallas {

struct TimedPoint {
  Eigen::Vector3d xyz{Eigen::Vector3d::Zero()};
  float intensity{0.0F};
  double relative_time_sec{0.0};
};

using TimedPointCloud = std::vector<TimedPoint>;

struct ImuSample {
  double stamp_sec{0.0};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
};

struct PoseState {
  double stamp_sec{0.0};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
};

inline Eigen::Vector3d TransformPoint(
  const PoseState& state,
  const Eigen::Vector3d& sensor_point,
  const Eigen::Isometry3d& sensor_to_body)
{
  const Eigen::Vector3d body_point = sensor_to_body * sensor_point;
  return state.orientation * body_point + state.position;
}

}  // namespace anima::pallas
