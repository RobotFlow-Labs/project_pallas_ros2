#pragma once

#include <anima_pallas_ros2/types.hpp>

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

namespace anima::pallas {

struct AttitudeBootstrapOptions {
  double stationary_window_sec{2.0};
  std::size_t min_samples{20};
  std::size_t max_samples{4096};
  double gravity_mps2{9.80665};
  double max_accel_std_mps2{0.25};
  double max_gyro_std_radps{0.03};
  double max_accel_norm_error_mps2{0.75};
};

struct AttitudeBootstrapResult {
  bool ready{false};
  PoseState seed_state{};
  Eigen::Vector3d mean_accel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mean_gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_std{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_std{Eigen::Vector3d::Zero()};
  std::size_t sample_count{0};
};

class AttitudeBootstrap {
public:
  explicit AttitudeBootstrap(AttitudeBootstrapOptions options = {});

  void Reset();
  void Push(const ImuSample& sample);

  std::size_t Size() const;
  AttitudeBootstrapOptions Options() const;
  void SetOptions(AttitudeBootstrapOptions options);

  std::optional<AttitudeBootstrapResult> TryBuild(double initial_yaw_rad = 0.0) const;
  std::optional<PoseState> TrySeed(double initial_yaw_rad = 0.0) const;

private:
  static Eigen::Quaterniond BuildLevelOrientation(
    const Eigen::Vector3d& mean_accel,
    double initial_yaw_rad);

  static void ComputeMeanAndStd(
    const std::deque<ImuSample>& samples,
    Eigen::Vector3d& mean_accel,
    Eigen::Vector3d& mean_gyro,
    Eigen::Vector3d& accel_std,
    Eigen::Vector3d& gyro_std);

  mutable std::mutex mutex_;
  std::deque<ImuSample> samples_;
  AttitudeBootstrapOptions options_;
};

}  // namespace anima::pallas
