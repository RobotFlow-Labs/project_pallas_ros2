#include <anima_pallas_ros2/core_runtime.hpp>

#include <rclcpp/time.hpp>

#include <algorithm>
#include <utility>

namespace anima::pallas {

namespace {

Eigen::Vector3d NormalizeNormal(
  const Eigen::Vector3d& normal,
  const Eigen::Vector3d& point_hint)
{
  Eigen::Vector3d normalized = normal;
  if (!normalized.allFinite() || normalized.norm() < 1e-9) {
    if (point_hint.norm() > 1e-9) {
      normalized = -point_hint.normalized();
    } else {
      normalized = Eigen::Vector3d::UnitZ();
    }
  } else {
    normalized.normalize();
  }

  if (point_hint.norm() > 1e-9 && normalized.dot(point_hint) > 0.0) {
    normalized = -normalized;
  }
  return normalized;
}

}  // namespace

PallasCoreRuntime::PallasCoreRuntime(PipelineConfig config)
: config_(std::move(config)),
  ingest_(CloudIngestOptions{
    config_.min_range_m,
    config_.max_range_m,
    true}),
  bootstrap_(
    AttitudeBootstrapOptions{
      config_.stationary_init_sec,
      20,
      config_.max_imu_buffer_size,
      config_.gravity_mps2,
      0.25,
      0.03,
      0.75}),
  tracker_(StrapdownTrackerOptions{
    config_.gravity_mps2,
    config_.max_imu_buffer_size,
    1e-4}),
  sampler_(ScanSamplerOptions{
    config_.scan_voxel_size_m,
    config_.scan_normal_radius_m,
    config_.scan_min_points_for_normal,
    true}),
  surfel_volume_(SurfelVolumeOptions{
    config_.map_voxel_size_m,
    config_.map_max_surfels,
    config_.map_max_age_sec})
{}

void PallasCoreRuntime::IngestImu(const ImuSample& sample)
{
  bootstrap_.Push(sample);
  if (!tracker_.IsInitialized()) {
    const auto seed = bootstrap_.TrySeed(0.0);
    if (seed.has_value()) {
      tracker_.Initialize(*seed);
    }
  }
  if (tracker_.IsInitialized()) {
    tracker_.Push(sample);
  }
}

RuntimeOutputs PallasCoreRuntime::IngestScan(const sensor_msgs::msg::PointCloud2& cloud_msg)
{
  RuntimeOutputs outputs;
  outputs.initialized = tracker_.IsInitialized();
  if (!outputs.initialized) {
    return outputs;
  }

  const auto prepared = PrepareScan(cloud_msg);
  if (!prepared.has_value()) {
    return outputs;
  }

  return FinalizeScan(*prepared, prepared->scan_pose, prepared->odom_pose);
}

std::optional<PreparedScan> PallasCoreRuntime::PrepareScan(
  const sensor_msgs::msg::PointCloud2& cloud_msg) const
{
  if (!tracker_.IsInitialized()) {
    return std::nullopt;
  }

  const TimedPointCloud parsed = ingest_.Ingest(cloud_msg);
  if (parsed.empty()) {
    return std::nullopt;
  }

  const SampledScan sampled = sampler_.Sample(parsed);
  if (sampled.empty()) {
    return std::nullopt;
  }

  const double stamp_sec = rclcpp::Time(cloud_msg.header.stamp).seconds();
  return PreparedScan{
    stamp_sec,
    tracker_.StateAt(stamp_sec),
    tracker_.Latest(),
    sampled};
}

RuntimeOutputs PallasCoreRuntime::FinalizeScan(
  const PreparedScan& prepared,
  const PoseState& scan_pose,
  const PoseState& odom_pose)
{
  RuntimeOutputs outputs;
  outputs.initialized = tracker_.IsInitialized();
  if (!outputs.initialized) {
    return outputs;
  }

  const SampledScan transformed = TransformScan(prepared.sampled_scan, scan_pose);
  if (transformed.empty()) {
    return outputs;
  }

  surfel_volume_.Integrate(transformed, prepared.stamp_sec);
  ++scan_count_;

  outputs.ready = true;
  outputs.initialized = true;
  outputs.scan_pose = scan_pose;
  outputs.odom_pose = odom_pose;
  outputs.aligned_scan.reserve(transformed.size());
  for (const auto& sample : transformed) {
    outputs.aligned_scan.push_back(sample.point);
  }

  outputs.published_map =
    (scan_count_ % std::max<std::size_t>(1, config_.map_publish_period_scans) == 0);
  if (outputs.published_map) {
    outputs.map_cloud = surfel_volume_.Cloud();
  }
  return outputs;
}

bool PallasCoreRuntime::IsInitialized() const
{
  return tracker_.IsInitialized();
}

PoseState PallasCoreRuntime::LatestOdom() const
{
  return tracker_.Latest();
}

SampledScan PallasCoreRuntime::TransformScan(
  const SampledScan& scan,
  const PoseState& scan_pose) const
{
  SampledScan transformed;
  transformed.reserve(scan.size());

  const Eigen::Matrix3d sensor_rotation = config_.sensor_to_body.linear();
  for (const auto& sample : scan) {
    SampledPoint transformed_sample = sample;
    transformed_sample.point.xyz =
      TransformPoint(scan_pose, sample.point.xyz, config_.sensor_to_body);
    transformed_sample.normal =
      NormalizeNormal(scan_pose.orientation * (sensor_rotation * sample.normal), transformed_sample.point.xyz);
    transformed.push_back(transformed_sample);
  }

  return transformed;
}

}  // namespace anima::pallas
