#pragma once

#include <anima_pallas_ros2/config.hpp>
#include <anima_pallas_ros2/imu_buffer.hpp>
#include <anima_pallas_ros2/local_map.hpp>
#include <anima_pallas_ros2/motion_integrator.hpp>
#include <anima_pallas_ros2/point_parser.hpp>
#include <anima_pallas_ros2/types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace anima::pallas {

struct PipelineOutputs {
  bool ready{false};
  bool initialized{false};
  bool published_map{false};
  PoseState scan_pose{};
  PoseState odom_pose{};
  TimedPointCloud aligned_scan{};
  TimedPointCloud map_cloud{};
};

class PallasPipeline {
public:
  explicit PallasPipeline(PipelineConfig config);

  void IngestImu(const ImuSample& sample);
  PipelineOutputs IngestScan(const sensor_msgs::msg::PointCloud2& cloud_msg);

  bool IsInitialized() const;
  PoseState LatestOdom() const;

private:
  TimedPointCloud FilterScan(const TimedPointCloud& scan) const;

  PipelineConfig config_;
  PointParser parser_;
  ImuBuffer imu_buffer_;
  MotionIntegrator integrator_;
  LocalMap local_map_;
  std::size_t scan_count_{0};
};

}  // namespace anima::pallas
