#include <anima_pallas_ros2/ct_runtime.hpp>

#include <algorithm>
#include <utility>

namespace anima::pallas {

PallasCtRuntime::PallasCtRuntime(PipelineConfig config)
: config_(std::move(config)),
  core_(config_)
{
  config_.ct_min_control_points = std::max<std::size_t>(2, config_.ct_min_control_points);
  config_.ct_max_control_points =
    std::max(config_.ct_min_control_points, config_.ct_max_control_points);
}

void PallasCtRuntime::IngestImu(const ImuSample& sample)
{
  core_.IngestImu(sample);
}

RuntimeOutputs PallasCtRuntime::IngestScan(const sensor_msgs::msg::PointCloud2& cloud_msg)
{
  RuntimeOutputs outputs = core_.IngestScan(cloud_msg);
  if (!outputs.ready) {
    return outputs;
  }

  AppendControlPoint(outputs.scan_pose);
  if (trajectory_.Size() < config_.ct_min_control_points) {
    return outputs;
  }

  outputs.scan_pose = SmoothState(outputs.scan_pose);
  outputs.odom_pose = SmoothState(outputs.odom_pose);
  return outputs;
}

bool PallasCtRuntime::IsInitialized() const
{
  return core_.IsInitialized();
}

PoseState PallasCtRuntime::LatestOdom() const
{
  return SmoothState(core_.LatestOdom());
}

void PallasCtRuntime::AppendControlPoint(const PoseState& state)
{
  trajectory_.InsertControlPoint(TimedPoseSpline::ControlPoint{
    state.stamp_sec,
    state.position,
    state.orientation});

  while (trajectory_.Size() > config_.ct_max_control_points) {
    const auto& control_points = trajectory_.ControlPoints();
    if (control_points.empty()) {
      break;
    }
    trajectory_.EraseControlPoint(control_points.front().stamp_sec);
  }
}

PoseState PallasCtRuntime::SmoothState(const PoseState& state) const
{
  PoseState smoothed = state;

  const auto pose = trajectory_.EvaluatePose(state.stamp_sec);
  if (pose.has_value()) {
    smoothed.position = pose->position;
    smoothed.orientation = pose->orientation;
  }

  const auto linear_velocity = trajectory_.EvaluateLinearVelocity(state.stamp_sec);
  if (linear_velocity.has_value()) {
    smoothed.velocity = *linear_velocity;
  }

  return smoothed;
}

}  // namespace anima::pallas
