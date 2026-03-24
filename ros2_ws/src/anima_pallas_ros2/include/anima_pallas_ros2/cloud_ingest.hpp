#pragma once

#include <anima_pallas_ros2/types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <limits>
#include <string>

namespace anima::pallas {

struct CloudIngestOptions {
  double min_range_m{0.0};
  double max_range_m{std::numeric_limits<double>::infinity()};
  bool normalize_relative_time{true};
};

class CloudIngest {
public:
  explicit CloudIngest(CloudIngestOptions options = {});

  const CloudIngestOptions& options() const;
  TimedPointCloud Ingest(const sensor_msgs::msg::PointCloud2& cloud_msg) const;

private:
  CloudIngestOptions options_;
};

}  // namespace anima::pallas
