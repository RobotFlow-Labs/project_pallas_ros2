#include <anima_pallas_ros2/scan_sampler.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>

namespace anima::pallas {

namespace {

struct VoxelKey {
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator<(const VoxelKey& other) const
  {
    if (x != other.x) {
      return x < other.x;
    }
    if (y != other.y) {
      return y < other.y;
    }
    return z < other.z;
  }

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

struct VoxelAccumulator {
  Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
  double intensity_sum{0.0};
  double time_sum{0.0};
  std::size_t count{0};
};

VoxelKey MakeKey(const Eigen::Vector3d& point, double voxel_size_m)
{
  return {
    static_cast<std::int64_t>(std::floor(point.x() / voxel_size_m)),
    static_cast<std::int64_t>(std::floor(point.y() / voxel_size_m)),
    static_cast<std::int64_t>(std::floor(point.z() / voxel_size_m))};
}

Eigen::Vector3d FallbackNormal(const Eigen::Vector3d& point)
{
  if (point.norm() > 1e-9) {
    return -point.normalized();
  }
  return Eigen::Vector3d::UnitZ();
}

}  // namespace

ScanSampler::ScanSampler(ScanSamplerOptions options)
: options_(std::move(options))
{
  if (options_.voxel_size_m < 0.0) {
    options_.voxel_size_m = 0.0;
  }
  if (options_.normal_radius_m < 0.0) {
    options_.normal_radius_m = 0.0;
  }
  if (options_.min_points_for_normal < 3) {
    options_.min_points_for_normal = 3;
  }
}

const ScanSamplerOptions& ScanSampler::options() const
{
  return options_;
}

SampledScan ScanSampler::Sample(const TimedPointCloud& cloud) const
{
  SampledScan samples;
  if (cloud.empty()) {
    return samples;
  }

  struct VoxelSample {
    VoxelKey key;
    SampledPoint sample;
  };

  std::vector<VoxelSample> voxel_samples;
  voxel_samples.reserve(cloud.size());

  if (options_.voxel_size_m <= 0.0) {
    for (std::size_t idx = 0; idx < cloud.size(); ++idx) {
      voxel_samples.push_back({
        VoxelKey{static_cast<std::int64_t>(idx), 0, 0},
        SampledPoint{cloud[idx], Eigen::Vector3d::UnitZ(), 1}});
    }
  } else {
    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
    voxels.reserve(cloud.size());

    for (const auto& point : cloud) {
      if (!point.xyz.allFinite()) {
        continue;
      }
      const VoxelKey key = MakeKey(point.xyz, options_.voxel_size_m);
      auto& acc = voxels[key];
      acc.sum += point.xyz;
      acc.intensity_sum += point.intensity;
      acc.time_sum += point.relative_time_sec;
      ++acc.count;
    }

    voxel_samples.reserve(voxels.size());
    for (const auto& [key, acc] : voxels) {
      if (acc.count == 0) {
        continue;
      }
      TimedPoint point;
      const double inv = 1.0 / static_cast<double>(acc.count);
      point.xyz = acc.sum * inv;
      point.intensity = static_cast<float>(acc.intensity_sum * inv);
      point.relative_time_sec = acc.time_sum * inv;
      voxel_samples.push_back({key, SampledPoint{point, Eigen::Vector3d::UnitZ(), acc.count}});
    }

    std::sort(
      voxel_samples.begin(), voxel_samples.end(),
      [](const VoxelSample& lhs, const VoxelSample& rhs) { return lhs.key < rhs.key; });
  }

  samples.reserve(voxel_samples.size());
  for (const auto& voxel_sample : voxel_samples) {
    samples.push_back(voxel_sample.sample);
  }

  if (samples.empty()) {
    return samples;
  }

  const double radius = options_.normal_radius_m > 0.0
    ? options_.normal_radius_m
    : std::max(0.30, options_.voxel_size_m * 2.0);
  const double radius_sq = radius * radius;

  for (std::size_t i = 0; i < samples.size(); ++i) {
    const Eigen::Vector3d& center = samples[i].point.xyz;
    double total_weight = 0.0;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    std::size_t neighbor_count = 0;

    for (std::size_t j = 0; j < samples.size(); ++j) {
      const Eigen::Vector3d diff = samples[j].point.xyz - center;
      if (diff.squaredNorm() > radius_sq) {
        continue;
      }
      const double weight = static_cast<double>(std::max<std::size_t>(1, samples[j].support));
      total_weight += weight;
      mean += weight * samples[j].point.xyz;
      ++neighbor_count;
    }

    if (neighbor_count < options_.min_points_for_normal || total_weight <= 0.0) {
      samples[i].normal = FallbackNormal(center);
      continue;
    }

    mean /= total_weight;

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (std::size_t j = 0; j < samples.size(); ++j) {
      const Eigen::Vector3d diff = samples[j].point.xyz - center;
      if (diff.squaredNorm() > radius_sq) {
        continue;
      }
      const double weight = static_cast<double>(std::max<std::size_t>(1, samples[j].support));
      const Eigen::Vector3d centered = samples[j].point.xyz - mean;
      covariance += weight * centered * centered.transpose();
    }
    covariance /= total_weight;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      samples[i].normal = FallbackNormal(center);
      continue;
    }

    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    if (!normal.allFinite() || normal.norm() < 1e-9) {
      samples[i].normal = FallbackNormal(center);
      continue;
    }

    normal.normalize();
    if (options_.orient_normals_toward_origin && center.norm() > 1e-9 && normal.dot(center) > 0.0) {
      normal = -normal;
    }
    samples[i].normal = normal;
  }

  return samples;
}

TimedPointCloud ScanSampler::Voxelize(const TimedPointCloud& cloud) const
{
  const SampledScan sampled = Sample(cloud);
  TimedPointCloud result;
  result.reserve(sampled.size());
  for (const auto& sample : sampled) {
    result.push_back(sample.point);
  }
  return result;
}

}  // namespace anima::pallas
