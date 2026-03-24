#include <anima_pallas_ros2/imu_buffer.hpp>

#include <algorithm>

namespace anima::pallas {

ImuBuffer::ImuBuffer(std::size_t max_size)
: max_size_(max_size) {}

void ImuBuffer::Push(const ImuSample& sample)
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.push_back(sample);
  while (samples_.size() > max_size_) {
    samples_.pop_front();
  }
}

std::vector<ImuSample> ImuBuffer::SamplesBetween(double start_sec, double end_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<ImuSample> selected;
  for (const auto& sample : samples_) {
    if (sample.stamp_sec >= start_sec && sample.stamp_sec <= end_sec) {
      selected.push_back(sample);
    }
  }
  return selected;
}

std::optional<ImuSample> ImuBuffer::Latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.empty()) {
    return std::nullopt;
  }
  return samples_.back();
}

StationaryInitResult ImuBuffer::TryInitialize(
  double stationary_sec,
  double initial_yaw_rad,
  double gravity_mps2) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  StationaryInitResult result;
  if (samples_.size() < 10) {
    return result;
  }

  const double window_end = samples_.back().stamp_sec;
  const double window_start = window_end - stationary_sec;
  if (samples_.front().stamp_sec > window_start) {
    return result;
  }

  Eigen::Vector3d accel_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
  std::size_t count = 0;
  for (const auto& sample : samples_) {
    if (sample.stamp_sec < window_start) {
      continue;
    }
    accel_mean += sample.accel;
    gyro_mean += sample.gyro;
    ++count;
  }

  if (count < 10) {
    return result;
  }

  accel_mean /= static_cast<double>(count);
  gyro_mean /= static_cast<double>(count);

  const Eigen::Vector3d accel_dir =
    accel_mean.norm() > 1e-6 ? accel_mean.normalized() : Eigen::Vector3d::UnitZ();
  const Eigen::Quaterniond roll_pitch =
    Eigen::Quaterniond::FromTwoVectors(accel_dir, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond yaw(
    Eigen::AngleAxisd(initial_yaw_rad, Eigen::Vector3d::UnitZ()));

  result.seed_state.orientation = yaw * roll_pitch;
  result.seed_state.position.setZero();
  result.seed_state.velocity.setZero();
  result.seed_state.gyro_bias = gyro_mean;
  result.seed_state.accel_bias =
    accel_mean - result.seed_state.orientation.inverse() *
    Eigen::Vector3d(0.0, 0.0, gravity_mps2);
  result.seed_state.stamp_sec = window_end;
  result.ready = true;
  return result;
}

}  // namespace anima::pallas
