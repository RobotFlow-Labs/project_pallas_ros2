#pragma once

#include <anima_pallas_ros2/scan_sampler.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace anima::pallas {

struct Surfel {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  float intensity{0.0F};
  double stamp_sec{0.0};
  double weight{0.0};
  std::size_t support{0};
  double radius_m{0.0};
};

struct SurfelVolumeOptions {
  double voxel_size_m{0.20};
  std::size_t max_surfels{50000};
  double max_age_sec{60.0};
};

class SurfelVolume {
public:
  explicit SurfelVolume(SurfelVolumeOptions options = {});

  const SurfelVolumeOptions& options() const;
  void Reset();
  void Integrate(const SampledScan& scan, double stamp_sec);

  std::size_t Size() const;
  std::vector<Surfel> Snapshot() const;
  TimedPointCloud Cloud() const;
  sensor_msgs::msg::PointCloud2 ToPointCloud2(
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp) const;

private:
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
    std::size_t operator()(const VoxelKey& key) const;
  };

  struct Entry {
    Surfel surfel;
    std::uint64_t sequence{0};
  };

  VoxelKey MakeKey(const Eigen::Vector3d& point) const;
  void CullExpired(double stamp_sec);
  void PruneToLimit();

  SurfelVolumeOptions options_;
  std::uint64_t sequence_{0};
  std::unordered_map<VoxelKey, Entry, VoxelKeyHash> surfels_;
};

}  // namespace anima::pallas
