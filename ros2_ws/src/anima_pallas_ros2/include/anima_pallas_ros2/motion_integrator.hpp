#pragma once

#include <anima_pallas_ros2/types.hpp>

#include <deque>
#include <mutex>

namespace anima::pallas {

class MotionIntegrator {
public:
  explicit MotionIntegrator(double gravity_mps2);

  void Initialize(const PoseState& seed_state);
  void PushImu(const ImuSample& sample);
  bool IsInitialized() const;

  PoseState LatestState() const;
  PoseState StateAt(double stamp_sec) const;

private:
  PoseState Interpolate(double stamp_sec) const;

  double gravity_mps2_{9.80665};
  bool initialized_{false};
  mutable std::mutex mutex_;
  std::deque<PoseState> history_;
  ImuSample last_imu_{};
  bool has_last_imu_{false};
};

}  // namespace anima::pallas
