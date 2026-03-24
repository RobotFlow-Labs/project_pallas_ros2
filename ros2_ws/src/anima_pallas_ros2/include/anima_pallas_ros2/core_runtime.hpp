#pragma once

#include <anima_pallas_ros2/attitude_bootstrap.hpp>
#include <anima_pallas_ros2/cloud_ingest.hpp>
#include <anima_pallas_ros2/config.hpp>
#include <anima_pallas_ros2/scan_sampler.hpp>
#include <anima_pallas_ros2/strapdown_tracker.hpp>
#include <anima_pallas_ros2/surfel_volume.hpp>
#include <anima_pallas_ros2/types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace anima::pallas {

struct RuntimeOutputs {
  bool ready{false};
  bool initialized{false};
  bool published_map{false};
  PoseState scan_pose{};
  PoseState odom_pose{};
  TimedPointCloud aligned_scan{};
  TimedPointCloud map_cloud{};
};

class PallasCoreRuntime {
public:
  explicit PallasCoreRuntime(PipelineConfig config);

  void IngestImu(const ImuSample& sample);
  RuntimeOutputs IngestScan(const sensor_msgs::msg::PointCloud2& cloud_msg);

  bool IsInitialized() const;
  PoseState LatestOdom() const;

private:
  SampledScan TransformScan(const SampledScan& scan, const PoseState& scan_pose) const;

  PipelineConfig config_;
  CloudIngest ingest_;
  AttitudeBootstrap bootstrap_;
  StrapdownTracker tracker_;
  ScanSampler sampler_;
  SurfelVolume surfel_volume_;
  std::size_t scan_count_{0};
};

}  // namespace anima::pallas
