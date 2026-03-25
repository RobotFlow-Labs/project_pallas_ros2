#include <anima_pallas_ros2/ct_runtime.hpp>

#include <algorithm>
#include <utility>

namespace anima::pallas {

namespace {

PipelineConfig ClampCtConfig(PipelineConfig cfg) {
  cfg.ct_min_control_points = std::max<std::size_t>(2, cfg.ct_min_control_points);
  cfg.ct_max_control_points = std::max(cfg.ct_min_control_points, cfg.ct_max_control_points);
  return cfg;
}

}  // namespace

PallasCtRuntime::PallasCtRuntime(PipelineConfig config)
: config_(ClampCtConfig(std::move(config))),
  core_(config_)
{}

void PallasCtRuntime::IngestImu(const ImuSample& sample)
{
  core_.IngestImu(sample);
}

RuntimeOutputs PallasCtRuntime::IngestScan(const sensor_msgs::msg::PointCloud2& cloud_msg)
{
  RuntimeOutputs outputs;
  outputs.initialized = core_.IsInitialized();
  if (!outputs.initialized) {
    return outputs;
  }

  const auto prepared = core_.PrepareScan(cloud_msg);
  if (!prepared.has_value()) {
    return outputs;
  }

  AppendControlPoint(prepared->scan_pose);

  PoseState scan_pose = prepared->scan_pose;
  PoseState odom_pose = prepared->odom_pose;
  if (trajectory_.Size() >= config_.ct_min_control_points) {
    scan_pose = SmoothState(scan_pose);
    odom_pose = SmoothState(odom_pose);
  }

  return core_.FinalizeScan(*prepared, scan_pose, odom_pose);
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
