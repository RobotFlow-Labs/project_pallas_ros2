#include <anima_pallas_ros2/local_map.hpp>

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

TimedPointCloud Voxelize(const TimedPointCloud& cloud, double voxel_size_m)
{
  struct Accumulator {
    Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
    double intensity_sum{0.0};
    double time_sum{0.0};
    std::size_t count{0};
  };

  std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash> voxels;
  voxels.reserve(cloud.size());

  for (const auto& point : cloud) {
    const VoxelKey key{
      static_cast<std::int64_t>(std::floor(point.xyz.x() / voxel_size_m)),
      static_cast<std::int64_t>(std::floor(point.xyz.y() / voxel_size_m)),
      static_cast<std::int64_t>(std::floor(point.xyz.z() / voxel_size_m))};
    auto& acc = voxels[key];
    acc.sum += point.xyz;
    acc.intensity_sum += point.intensity;
    acc.time_sum += point.relative_time_sec;
    ++acc.count;
  }

  TimedPointCloud filtered;
  filtered.reserve(voxels.size());
  for (const auto& [key, acc] : voxels) {
    (void)key;
    TimedPoint point;
    const double inv = 1.0 / static_cast<double>(acc.count);
    point.xyz = acc.sum * inv;
    point.intensity = static_cast<float>(acc.intensity_sum * inv);
    point.relative_time_sec = acc.time_sum * inv;
    filtered.push_back(point);
  }
  return filtered;
}

}  // namespace

LocalMap::LocalMap(double voxel_size_m, std::size_t max_scans)
: voxel_size_m_(voxel_size_m), max_scans_(max_scans) {}

void LocalMap::Insert(const TimedPointCloud& transformed_scan)
{
  scans_.push_back(transformed_scan);
  while (scans_.size() > max_scans_) {
    scans_.pop_front();
  }
  Rebuild();
}

TimedPointCloud LocalMap::Cloud() const
{
  return cached_cloud_;
}

void LocalMap::Rebuild()
{
  TimedPointCloud merged;
  std::size_t total = 0;
  for (const auto& scan : scans_) {
    total += scan.size();
  }
  merged.reserve(total);
  for (const auto& scan : scans_) {
    merged.insert(merged.end(), scan.begin(), scan.end());
  }
  cached_cloud_ = Voxelize(merged, voxel_size_m_);
}

}  // namespace anima::pallas
