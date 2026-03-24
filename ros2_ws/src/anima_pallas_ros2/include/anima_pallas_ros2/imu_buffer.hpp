#pragma once

#include <anima_pallas_ros2/types.hpp>

#include <deque>
#include <mutex>
#include <optional>
#include <vector>

namespace anima::pallas {

struct StationaryInitResult {
  bool ready{false};
  PoseState seed_state{};
};

class ImuBuffer {
public:
  explicit ImuBuffer(std::size_t max_size);

  void Push(const ImuSample& sample);
  std::vector<ImuSample> SamplesBetween(double start_sec, double end_sec) const;
  std::optional<ImuSample> Latest() const;

  StationaryInitResult TryInitialize(
    double stationary_sec,
    double initial_yaw_rad,
    double gravity_mps2) const;

private:
  std::size_t max_size_{0};
  mutable std::mutex mutex_;
  std::deque<ImuSample> samples_;
};

}  // namespace anima::pallas
