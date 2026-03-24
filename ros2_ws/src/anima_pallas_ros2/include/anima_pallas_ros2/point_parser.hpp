#pragma once

#include <anima_pallas_ros2/types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

namespace anima::pallas {

class PointParser {
public:
  TimedPointCloud Parse(
    const sensor_msgs::msg::PointCloud2& cloud_msg,
    const std::string& lidar_type) const;
};

}  // namespace anima::pallas
