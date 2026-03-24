#pragma once

#include <anima_pallas_ros2/types.hpp>

#include <cstddef>
#include <deque>

namespace anima::pallas {

class LocalMap {
public:
  LocalMap(double voxel_size_m, std::size_t max_scans);

  void Insert(const TimedPointCloud& transformed_scan);
  TimedPointCloud Cloud() const;

private:
  void Rebuild();

  double voxel_size_m_{0.2};
  std::size_t max_scans_{20};
  std::deque<TimedPointCloud> scans_;
  TimedPointCloud cached_cloud_;
};

}  // namespace anima::pallas
