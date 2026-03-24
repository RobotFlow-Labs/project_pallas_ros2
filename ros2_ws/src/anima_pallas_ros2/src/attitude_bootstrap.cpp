#include <anima_pallas_ros2/attitude_bootstrap.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace anima::pallas {

namespace {

constexpr double kSmallNumber = 1e-12;

}  // namespace

AttitudeBootstrap::AttitudeBootstrap(AttitudeBootstrapOptions options)
: options_(options) {}

void AttitudeBootstrap::Reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
}

void AttitudeBootstrap::Push(const ImuSample& sample)
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.push_back(sample);
  while (samples_.size() > options_.max_samples) {
    samples_.pop_front();
  }
}

std::size_t AttitudeBootstrap::Size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.size();
}

AttitudeBootstrapOptions AttitudeBootstrap::Options() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return options_;
}

void AttitudeBootstrap::SetOptions(AttitudeBootstrapOptions options)
{
  std::lock_guard<std::mutex> lock(mutex_);
  options_ = options;
  while (samples_.size() > options_.max_samples) {
    samples_.pop_front();
  }
}

void AttitudeBootstrap::ComputeMeanAndStd(
  const std::deque<ImuSample>& samples,
  Eigen::Vector3d& mean_accel,
  Eigen::Vector3d& mean_gyro,
  Eigen::Vector3d& accel_std,
  Eigen::Vector3d& gyro_std)
{
  mean_accel.setZero();
  mean_gyro.setZero();
  accel_std.setZero();
  gyro_std.setZero();

  if (samples.empty()) {
    return;
  }

  for (const auto& sample : samples) {
    mean_accel += sample.accel;
    mean_gyro += sample.gyro;
  }

  const double inv_count = 1.0 / static_cast<double>(samples.size());
  mean_accel *= inv_count;
  mean_gyro *= inv_count;

  Eigen::Vector3d accel_var = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_var = Eigen::Vector3d::Zero();
  for (const auto& sample : samples) {
    accel_var += (sample.accel - mean_accel).cwiseProduct(sample.accel - mean_accel);
    gyro_var += (sample.gyro - mean_gyro).cwiseProduct(sample.gyro - mean_gyro);
  }

  accel_var *= inv_count;
  gyro_var *= inv_count;

  accel_std = accel_var.cwiseMax(Eigen::Vector3d::Zero()).cwiseSqrt();
  gyro_std = gyro_var.cwiseMax(Eigen::Vector3d::Zero()).cwiseSqrt();
}

Eigen::Quaterniond AttitudeBootstrap::BuildLevelOrientation(
  const Eigen::Vector3d& mean_accel,
  double initial_yaw_rad)
{
  const Eigen::Vector3d world_z = Eigen::Vector3d::UnitZ();
  Eigen::Quaterniond roll_pitch = Eigen::Quaterniond::Identity();

  if (mean_accel.norm() > kSmallNumber) {
    const Eigen::Vector3d accel_dir = mean_accel.normalized();
    if ((accel_dir - world_z).norm() > 1e-9) {
      roll_pitch = Eigen::Quaterniond::FromTwoVectors(accel_dir, world_z);
    }
  }

  const Eigen::Quaterniond yaw(Eigen::AngleAxisd(initial_yaw_rad, world_z));
  Eigen::Quaterniond orientation = yaw * roll_pitch;
  orientation.normalize();
  return orientation;
}

std::optional<AttitudeBootstrapResult> AttitudeBootstrap::TryBuild(double initial_yaw_rad) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.size() < options_.min_samples) {
    return std::nullopt;
  }

  const double window_end = samples_.back().stamp_sec;
  const double window_start = window_end - options_.stationary_window_sec;
  if (samples_.front().stamp_sec > window_start) {
    return std::nullopt;
  }

  std::deque<ImuSample> window_samples;
  for (const auto& sample : samples_) {
    if (sample.stamp_sec >= window_start) {
      window_samples.push_back(sample);
    }
  }

  if (window_samples.size() < options_.min_samples) {
    return std::nullopt;
  }

  AttitudeBootstrapResult result;
  result.sample_count = window_samples.size();
  ComputeMeanAndStd(
    window_samples,
    result.mean_accel,
    result.mean_gyro,
    result.accel_std,
    result.gyro_std);

  if (result.accel_std.maxCoeff() > options_.max_accel_std_mps2) {
    return std::nullopt;
  }
  if (result.gyro_std.maxCoeff() > options_.max_gyro_std_radps) {
    return std::nullopt;
  }
  if (std::abs(result.mean_accel.norm() - options_.gravity_mps2) > options_.max_accel_norm_error_mps2) {
    return std::nullopt;
  }

  result.ready = true;
  result.seed_state.stamp_sec = window_end;
  result.seed_state.orientation = BuildLevelOrientation(result.mean_accel, initial_yaw_rad);
  result.seed_state.position.setZero();
  result.seed_state.velocity.setZero();
  result.seed_state.gyro_bias = result.mean_gyro;
  result.seed_state.accel_bias =
    result.mean_accel -
    result.seed_state.orientation.inverse() * Eigen::Vector3d(0.0, 0.0, options_.gravity_mps2);
  return result;
}

std::optional<PoseState> AttitudeBootstrap::TrySeed(double initial_yaw_rad) const
{
  const auto result = TryBuild(initial_yaw_rad);
  if (!result.has_value()) {
    return std::nullopt;
  }
  return result->seed_state;
}

}  // namespace anima::pallas
