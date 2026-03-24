#pragma once

#include <anima_pallas_ros2/core_runtime.hpp>
#include <anima_pallas_ros2/timed_pose_spline.hpp>

namespace anima::pallas {

class PallasCtRuntime {
public:
  explicit PallasCtRuntime(PipelineConfig config);

  void IngestImu(const ImuSample& sample);
  RuntimeOutputs IngestScan(const sensor_msgs::msg::PointCloud2& cloud_msg);

  bool IsInitialized() const;
  PoseState LatestOdom() const;

private:
  void AppendControlPoint(const PoseState& state);
  PoseState SmoothState(const PoseState& state) const;

  PipelineConfig config_;
  PallasCoreRuntime core_;
  TimedPoseSpline trajectory_;
};

}  // namespace anima::pallas
