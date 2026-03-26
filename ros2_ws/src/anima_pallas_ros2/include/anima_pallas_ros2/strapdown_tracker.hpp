#pragma once

#include <anima_pallas_ros2/types.hpp>

#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

namespace anima::pallas {

struct StrapdownTrackerOptions {
  double gravity_mps2{9.80665};
  std::size_t max_history{4096};
  double min_dt_sec{1e-4};
};

class StrapdownTracker {
public:
  explicit StrapdownTracker(double gravity_mps2 = 9.80665);
  explicit StrapdownTracker(StrapdownTrackerOptions options);

  void Reset();
  void Initialize(const PoseState& seed_state);
  bool IsInitialized() const;
  void Push(const ImuSample& sample);

  PoseState Latest() const;
  PoseState StateAt(double stamp_sec) const;
  std::vector<PoseState> History() const;

  /// Apply an external pose correction (e.g. from scan-to-map alignment).
  /// Updates the latest state and optionally blends bias corrections.
  void ApplyCorrection(
    const Eigen::Vector3d& position_delta,
    const Eigen::Quaterniond& rotation_delta,
    const Eigen::Vector3d& gyro_bias_update,
    const Eigen::Vector3d& accel_bias_update,
    double bias_learning_rate = 0.1);

private:
  static Eigen::Quaterniond IntegrateRotation(
    const Eigen::Quaterniond& current,
    const Eigen::Vector3d& omega_rad_s,
    double dt);

  PoseState InterpolateUnlocked(double stamp_sec) const;
  void TrimHistoryUnlocked();

  StrapdownTrackerOptions options_;
  bool initialized_{false};
  mutable std::mutex mutex_;
  std::deque<PoseState> history_;
  ImuSample last_imu_{};
  bool has_last_imu_{false};
};

}  // namespace anima::pallas
