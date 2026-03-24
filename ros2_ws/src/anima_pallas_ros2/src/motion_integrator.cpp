#include <anima_pallas_ros2/motion_integrator.hpp>

#include <algorithm>

namespace anima::pallas {

namespace {

Eigen::Quaterniond IntegrateRotation(
  const Eigen::Quaterniond& current,
  const Eigen::Vector3d& omega_rad_s,
  double dt)
{
  const double angle = omega_rad_s.norm() * dt;
  if (angle < 1e-12) {
    return current;
  }

  const Eigen::AngleAxisd delta(angle, omega_rad_s.normalized());
  Eigen::Quaterniond next = current * Eigen::Quaterniond(delta);
  next.normalize();
  return next;
}

}  // namespace

MotionIntegrator::MotionIntegrator(double gravity_mps2)
: gravity_mps2_(gravity_mps2) {}

void MotionIntegrator::Initialize(const PoseState& seed_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  history_.clear();
  history_.push_back(seed_state);
  initialized_ = true;
  has_last_imu_ = false;
}

void MotionIntegrator::PushImu(const ImuSample& sample)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    return;
  }

  if (!has_last_imu_) {
    last_imu_ = sample;
    has_last_imu_ = true;
    return;
  }

  PoseState next = history_.back();
  const double dt = std::max(1e-4, sample.stamp_sec - last_imu_.stamp_sec);
  const Eigen::Vector3d unbiased_gyro = sample.gyro - next.gyro_bias;
  const Eigen::Vector3d unbiased_accel = sample.accel - next.accel_bias;

  next.orientation = IntegrateRotation(next.orientation, unbiased_gyro, dt);
  const Eigen::Vector3d world_accel =
    next.orientation * unbiased_accel - Eigen::Vector3d(0.0, 0.0, gravity_mps2_);

  next.position += next.velocity * dt + 0.5 * world_accel * dt * dt;
  next.velocity += world_accel * dt;
  next.stamp_sec = sample.stamp_sec;

  history_.push_back(next);
  while (history_.size() > 20000) {
    history_.pop_front();
  }
  last_imu_ = sample;
}

bool MotionIntegrator::IsInitialized() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

PoseState MotionIntegrator::LatestState() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return history_.empty() ? PoseState{} : history_.back();
}

PoseState MotionIntegrator::Interpolate(double stamp_sec) const
{
  if (history_.empty()) {
    return PoseState{};
  }
  if (stamp_sec <= history_.front().stamp_sec) {
    return history_.front();
  }
  if (stamp_sec >= history_.back().stamp_sec) {
    return history_.back();
  }

  const auto upper = std::lower_bound(
    history_.begin(), history_.end(), stamp_sec,
    [](const PoseState& state, double stamp) { return state.stamp_sec < stamp; });
  if (upper == history_.begin()) {
    return *upper;
  }

  const auto lower = std::prev(upper);
  const double span = upper->stamp_sec - lower->stamp_sec;
  const double alpha = span > 1e-9 ? (stamp_sec - lower->stamp_sec) / span : 0.0;

  PoseState interp = *lower;
  interp.stamp_sec = stamp_sec;
  interp.position = lower->position * (1.0 - alpha) + upper->position * alpha;
  interp.velocity = lower->velocity * (1.0 - alpha) + upper->velocity * alpha;
  interp.gyro_bias = lower->gyro_bias * (1.0 - alpha) + upper->gyro_bias * alpha;
  interp.accel_bias = lower->accel_bias * (1.0 - alpha) + upper->accel_bias * alpha;
  interp.orientation = lower->orientation.slerp(alpha, upper->orientation);
  interp.orientation.normalize();
  return interp;
}

PoseState MotionIntegrator::StateAt(double stamp_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return Interpolate(stamp_sec);
}

}  // namespace anima::pallas
