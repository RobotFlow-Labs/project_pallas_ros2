#include <anima_pallas_ros2/pipeline.hpp>

#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace anima::pallas {

namespace {

struct VoxelKey {
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const VoxelKey& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const
  {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

TimedPointCloud VoxelFilter(const TimedPointCloud& cloud, double voxel_size_m)
{
  if (voxel_size_m <= 0.0) {
    return cloud;
  }

  std::unordered_map<VoxelKey, TimedPoint, VoxelKeyHash> chosen;
  chosen.reserve(cloud.size());

  for (const auto& point : cloud) {
    const VoxelKey key{
      static_cast<std::int64_t>(std::floor(point.xyz.x() / voxel_size_m)),
      static_cast<std::int64_t>(std::floor(point.xyz.y() / voxel_size_m)),
      static_cast<std::int64_t>(std::floor(point.xyz.z() / voxel_size_m))};
    if (chosen.find(key) == chosen.end()) {
      chosen.emplace(key, point);
    }
  }

  TimedPointCloud filtered;
  filtered.reserve(chosen.size());
  for (const auto& [key, point] : chosen) {
    (void)key;
    filtered.push_back(point);
  }
  return filtered;
}

}  // namespace

PallasPipeline::PallasPipeline(PipelineConfig config)
: config_(std::move(config)),
  imu_buffer_(config_.max_imu_buffer_size),
  integrator_(config_.gravity_mps2),
  local_map_(config_.map_voxel_size_m, config_.max_map_scans) {}

void PallasPipeline::IngestImu(const ImuSample& sample)
{
  imu_buffer_.Push(sample);
  if (!integrator_.IsInitialized()) {
    const auto init = imu_buffer_.TryInitialize(config_.stationary_init_sec, 0.0, config_.gravity_mps2);
    if (init.ready) {
      integrator_.Initialize(init.seed_state);
    }
  }
  if (integrator_.IsInitialized()) {
    integrator_.PushImu(sample);
  }
}

PipelineOutputs PallasPipeline::IngestScan(const sensor_msgs::msg::PointCloud2& cloud_msg)
{
  PipelineOutputs outputs;
  outputs.initialized = integrator_.IsInitialized();
  if (!outputs.initialized) {
    return outputs;
  }

  const TimedPointCloud parsed = parser_.Parse(cloud_msg, config_.lidar_type);
  const TimedPointCloud filtered = FilterScan(parsed);
  const double stamp_sec = rclcpp::Time(cloud_msg.header.stamp).seconds();
  const PoseState scan_pose = integrator_.StateAt(stamp_sec);

  TimedPointCloud transformed;
  transformed.reserve(filtered.size());
  for (const auto& point : filtered) {
    TimedPoint transformed_point = point;
    transformed_point.xyz = TransformPoint(scan_pose, point.xyz, config_.sensor_to_body);
    transformed.push_back(transformed_point);
  }

  local_map_.Insert(transformed);
  ++scan_count_;

  outputs.ready = true;
  outputs.initialized = true;
  outputs.scan_pose = scan_pose;
  outputs.odom_pose = integrator_.LatestState();
  outputs.aligned_scan = std::move(transformed);
  outputs.published_map = (scan_count_ % std::max<std::size_t>(1, config_.map_publish_period_scans) == 0);
  if (outputs.published_map) {
    outputs.map_cloud = local_map_.Cloud();
  }
  return outputs;
}

bool PallasPipeline::IsInitialized() const
{
  return integrator_.IsInitialized();
}

PoseState PallasPipeline::LatestOdom() const
{
  return integrator_.LatestState();
}

TimedPointCloud PallasPipeline::FilterScan(const TimedPointCloud& scan) const
{
  TimedPointCloud clipped;
  clipped.reserve(scan.size());
  for (const auto& point : scan) {
    const double norm = point.xyz.norm();
    if (norm < config_.min_range_m || norm > config_.max_range_m) {
      continue;
    }
    clipped.push_back(point);
  }
  return VoxelFilter(clipped, config_.scan_voxel_size_m);
}

}  // namespace anima::pallas
