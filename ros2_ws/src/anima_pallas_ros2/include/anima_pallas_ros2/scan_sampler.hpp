#pragma once

#include <anima_pallas_ros2/types.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace anima::pallas {

struct SampledPoint {
  TimedPoint point{};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  std::size_t support{0};
};

using SampledScan = std::vector<SampledPoint>;

struct ScanSamplerOptions {
  double voxel_size_m{0.20};
  double normal_radius_m{0.30};
  std::size_t min_points_for_normal{6};
  bool orient_normals_toward_origin{true};
};

class ScanSampler {
public:
  explicit ScanSampler(ScanSamplerOptions options = {});

  const ScanSamplerOptions& options() const;
  SampledScan Sample(const TimedPointCloud& cloud) const;
  TimedPointCloud Voxelize(const TimedPointCloud& cloud) const;

private:
  ScanSamplerOptions options_;
};

}  // namespace anima::pallas
