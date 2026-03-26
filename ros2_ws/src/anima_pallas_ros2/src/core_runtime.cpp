#include <anima_pallas_ros2/core_runtime.hpp>
#include <anima_pallas_ros2/normal_utils.hpp>

#include <rclcpp/time.hpp>

#include <algorithm>
#include <utility>

namespace anima::pallas {

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
      config_.max_accel_std_mps2,
      config_.max_gyro_std_radps,
      config_.max_accel_norm_error_mps2}),
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
    } else if (bootstrap_.Size() > 200) {
      // Fallback: if bootstrap collected 200+ samples but stationarity checks
      // still fail (e.g. simulated IMU with unusual noise), force-init with
      // identity orientation to avoid indefinite blocking.
      PoseState forced_seed;
      forced_seed.stamp_sec = sample.stamp_sec;
      forced_seed.orientation = Eigen::Quaterniond::Identity();
      tracker_.Initialize(forced_seed);
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

  SampledScan transformed = TransformScan(prepared.sampled_scan, scan_pose);
  if (transformed.empty()) {
    return outputs;
  }

  // Scan-to-map alignment: correct IMU drift using point-to-plane ICP.
  PoseState corrected_scan_pose = scan_pose;
  PoseState corrected_odom_pose = odom_pose;
  if (scan_count_ > 0) {
    const auto alignment = surfel_volume_.AlignScan(transformed);
    if (alignment && alignment->inlier_count > 5) {
      // Estimate gyro bias from rotation drift between scans.
      const Eigen::AngleAxisd aa(alignment->rotation_delta);
      const double scan_dt = 0.15;
      const Eigen::Vector3d implied_gyro_bias =
        (aa.angle() > 1e-6) ? (aa.axis() * aa.angle() / scan_dt) : Eigen::Vector3d::Zero();

      tracker_.ApplyCorrection(
        alignment->position_delta,
        alignment->rotation_delta,
        implied_gyro_bias,
        Eigen::Vector3d::Zero(),
        0.3);

      corrected_scan_pose = tracker_.StateAt(prepared.stamp_sec);
      corrected_odom_pose = tracker_.Latest();
      transformed = TransformScan(prepared.sampled_scan, corrected_scan_pose);
    } else {
      // ICP failed (no inliers — scan drifted too far from map).
      // Hard-reset position back toward map centroid to recover.
      const auto map_cloud = surfel_volume_.Cloud();
      if (!map_cloud.empty()) {
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& pt : map_cloud) {
          centroid += pt.xyz;
        }
        centroid /= static_cast<double>(map_cloud.size());
        const Eigen::Vector3d reset_delta = centroid - scan_pose.position;
        tracker_.ApplyCorrection(
          reset_delta,
          Eigen::Quaterniond::Identity(),
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(),
          1.0);  // full correction — hard reset

        corrected_scan_pose = tracker_.StateAt(prepared.stamp_sec);
        corrected_odom_pose = tracker_.Latest();
        transformed = TransformScan(prepared.sampled_scan, corrected_scan_pose);
      }
    }
  }

  surfel_volume_.Integrate(transformed, prepared.stamp_sec);
  ++scan_count_;

  outputs.ready = true;
  outputs.initialized = true;
  outputs.scan_pose = corrected_scan_pose;
  outputs.odom_pose = corrected_odom_pose;
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
