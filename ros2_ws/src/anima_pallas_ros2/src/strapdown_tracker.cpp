#include <anima_pallas_ros2/strapdown_tracker.hpp>

#include <Eigen/Geometry>

#include <algorithm>

namespace anima::pallas {

namespace {

constexpr double kSmallNumber = 1e-12;

}  // namespace

StrapdownTracker::StrapdownTracker(double gravity_mps2)
: options_{gravity_mps2, 4096, 1e-4} {}

StrapdownTracker::StrapdownTracker(StrapdownTrackerOptions options)
: options_(options) {}

void StrapdownTracker::Reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  initialized_ = false;
  history_.clear();
  has_last_imu_ = false;
  last_imu_ = {};
}

void StrapdownTracker::Initialize(const PoseState& seed_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  history_.clear();
  PoseState seed = seed_state;
  seed.orientation.normalize();
  history_.push_back(seed);
  initialized_ = true;
  has_last_imu_ = false;
  last_imu_ = {};
}

bool StrapdownTracker::IsInitialized() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

Eigen::Quaterniond StrapdownTracker::IntegrateRotation(
  const Eigen::Quaterniond& current,
  const Eigen::Vector3d& omega_rad_s,
  double dt)
{
  const double angle = omega_rad_s.norm() * dt;
  if (angle < kSmallNumber) {
    return current;
  }

  const Eigen::AngleAxisd delta(angle, omega_rad_s.normalized());
  Eigen::Quaterniond next = current * Eigen::Quaterniond(delta);
  next.normalize();
  return next;
}

void StrapdownTracker::Push(const ImuSample& sample)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || history_.empty()) {
    return;
  }

  if (!has_last_imu_) {
    last_imu_ = sample;
    has_last_imu_ = true;
    return;
  }

  if (sample.stamp_sec <= last_imu_.stamp_sec) {
    last_imu_ = sample;
    return;
  }

  PoseState next = history_.back();
  const double dt = std::max(options_.min_dt_sec, sample.stamp_sec - last_imu_.stamp_sec);
  const Eigen::Vector3d unbiased_gyro = sample.gyro - next.gyro_bias;
  const Eigen::Vector3d unbiased_accel = sample.accel - next.accel_bias;

  next.orientation = IntegrateRotation(next.orientation, unbiased_gyro, dt);
  const Eigen::Vector3d world_accel =
    next.orientation * unbiased_accel - Eigen::Vector3d(0.0, 0.0, options_.gravity_mps2);

  next.position += next.velocity * dt + 0.5 * world_accel * dt * dt;
  next.velocity += world_accel * dt;
  next.stamp_sec = sample.stamp_sec;

  history_.push_back(next);
  TrimHistoryUnlocked();
  last_imu_ = sample;
}

PoseState StrapdownTracker::Latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return history_.empty() ? PoseState{} : history_.back();
}

PoseState StrapdownTracker::InterpolateUnlocked(double stamp_sec) const
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
  const double alpha = span > kSmallNumber ? (stamp_sec - lower->stamp_sec) / span : 0.0;

  PoseState interp = *lower;
  interp.stamp_sec = stamp_sec;
  interp.position = lower->position * (1.0 - alpha) + upper->position * alpha;
  interp.velocity = lower->velocity * (1.0 - alpha) + upper->velocity * alpha;
  interp.gyro_bias = lower->gyro_bias * (1.0 - alpha) + upper->gyro_bias * alpha;
  interp.accel_bias = lower->accel_bias * (1.0 - alpha) + upper->accel_bias * alpha;
  Eigen::Quaterniond target = upper->orientation;
  if (lower->orientation.coeffs().dot(target.coeffs()) < 0.0) {
    target.coeffs() *= -1.0;
  }
  interp.orientation = lower->orientation.slerp(alpha, target);
  interp.orientation.normalize();
  return interp;
}

PoseState StrapdownTracker::StateAt(double stamp_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return InterpolateUnlocked(stamp_sec);
}

std::vector<PoseState> StrapdownTracker::History() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return std::vector<PoseState>(history_.begin(), history_.end());
}

void StrapdownTracker::TrimHistoryUnlocked()
{
  if (options_.max_history == 0) {
    return;
  }
  while (history_.size() > options_.max_history) {
    history_.pop_front();
  }
}

}  // namespace anima::pallas
