#pragma once

#include <anima_pallas_ros2/config.hpp>
#include <anima_pallas_ros2/types.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace anima::pallas {

namespace detail {

inline geometry_msgs::msg::PoseStamped ToPoseStamped(
  const PoseState& state,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.pose.position.x = state.position.x();
  msg.pose.position.y = state.position.y();
  msg.pose.position.z = state.position.z();
  msg.pose.orientation.w = state.orientation.w();
  msg.pose.orientation.x = state.orientation.x();
  msg.pose.orientation.y = state.orientation.y();
  msg.pose.orientation.z = state.orientation.z();
  return msg;
}

inline nav_msgs::msg::Odometry ToOdometry(
  const PoseState& state,
  const std::string& odom_frame,
  const std::string& child_frame,
  const builtin_interfaces::msg::Time& stamp)
{
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = odom_frame;
  msg.child_frame_id = child_frame;
  msg.header.stamp = stamp;
  msg.pose.pose = ToPoseStamped(state, odom_frame, stamp).pose;
  msg.twist.twist.linear.x = state.velocity.x();
  msg.twist.twist.linear.y = state.velocity.y();
  msg.twist.twist.linear.z = state.velocity.z();
  return msg;
}

inline geometry_msgs::msg::TransformStamped ToTransform(
  const PoseState& state,
  const std::string& parent_frame,
  const std::string& child_frame,
  const builtin_interfaces::msg::Time& stamp)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = parent_frame;
  msg.child_frame_id = child_frame;
  msg.header.stamp = stamp;
  msg.transform.translation.x = state.position.x();
  msg.transform.translation.y = state.position.y();
  msg.transform.translation.z = state.position.z();
  msg.transform.rotation.w = state.orientation.w();
  msg.transform.rotation.x = state.orientation.x();
  msg.transform.rotation.y = state.orientation.y();
  msg.transform.rotation.z = state.orientation.z();
  return msg;
}

inline sensor_msgs::msg::PointCloud2 ToPointCloud2(
  const TimedPointCloud& cloud,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.height = 1;
  msg.width = static_cast<std::uint32_t>(cloud.size());
  msg.is_dense = false;
  msg.is_bigendian = false;

  msg.fields.resize(6);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 16;
  msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.fields[5].name = "time";
  msg.fields[5].offset = 20;
  msg.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[5].count = 1;

  msg.point_step = 24;
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);

  for (std::size_t i = 0; i < cloud.size(); ++i) {
    std::uint8_t* point_data = msg.data.data() + i * msg.point_step;
    const float x = static_cast<float>(cloud[i].xyz.x());
    const float y = static_cast<float>(cloud[i].xyz.y());
    const float z = static_cast<float>(cloud[i].xyz.z());
    const float intensity = cloud[i].intensity;
    const std::uint16_t ring = cloud[i].ring;
    const float time = static_cast<float>(cloud[i].relative_time_sec);

    std::memcpy(point_data + 0, &x, sizeof(x));
    std::memcpy(point_data + 4, &y, sizeof(y));
    std::memcpy(point_data + 8, &z, sizeof(z));
    std::memcpy(point_data + 12, &intensity, sizeof(intensity));
    std::memcpy(point_data + 16, &ring, sizeof(ring));
    point_data[18] = 0;
    point_data[19] = 0;
    std::memcpy(point_data + 20, &time, sizeof(time));
  }

  return msg;
}

inline Eigen::Vector3d VectorFromParameter(const std::vector<double>& values)
{
  if (values.size() < 3) {
    return Eigen::Vector3d::Zero();
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

}  // namespace detail

template <typename RuntimeT>
class PallasRuntimeNode : public rclcpp::Node {
public:
  PallasRuntimeNode(const std::string& node_name, const std::string& default_output_prefix)
  : rclcpp::Node(node_name)
  {
    const PipelineConfig config = LoadConfig();
    runtime_ = std::make_unique<RuntimeT>(config);

    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/points_raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    pose_topic_ = declare_parameter<std::string>("pose_topic", default_output_prefix + "/pose");
    odom_topic_ = declare_parameter<std::string>("odom_topic", default_output_prefix + "/odom");
    map_topic_ = declare_parameter<std::string>("map_topic", default_output_prefix + "/local_map");
    aligned_scan_topic_ = declare_parameter<std::string>(
      "aligned_scan_topic", default_output_prefix + "/aligned_scan");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "imu");

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 50);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable());
    aligned_scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(aligned_scan_topic_, 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PallasRuntimeNode::OnImu, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PallasRuntimeNode::OnCloud, this, std::placeholders::_1));
  }

private:
  PipelineConfig LoadConfig()
  {
    PipelineConfig cfg;
    cfg.lidar_type = declare_parameter<std::string>("lidar_type", "generic");
    cfg.stationary_init_sec = declare_parameter<double>("stationary_init_sec", 2.0);
    cfg.gravity_mps2 = declare_parameter<double>("gravity_mps2", 9.80665);
    cfg.min_range_m = declare_parameter<double>("min_range_m", 1.0);
    cfg.max_range_m = declare_parameter<double>("max_range_m", 200.0);
    cfg.scan_voxel_size_m = declare_parameter<double>("scan_voxel_size_m", 0.10);
    cfg.scan_normal_radius_m = declare_parameter<double>("scan_normal_radius_m", 0.30);
    cfg.scan_min_points_for_normal = static_cast<std::size_t>(
      declare_parameter<int>("scan_min_points_for_normal", 6));
    cfg.map_voxel_size_m = declare_parameter<double>("map_voxel_size_m", 0.20);
    cfg.map_max_surfels = static_cast<std::size_t>(
      declare_parameter<int>("map_max_surfels", 50000));
    cfg.map_max_age_sec = declare_parameter<double>("map_max_age_sec", 60.0);
    cfg.max_imu_buffer_size = static_cast<std::size_t>(
      declare_parameter<int>("max_imu_buffer_size", 20000));
    cfg.map_publish_period_scans = static_cast<std::size_t>(
      declare_parameter<int>("map_publish_period_scans", 1));
    cfg.ct_min_control_points = static_cast<std::size_t>(
      declare_parameter<int>("ct_min_control_points", 4));
    cfg.ct_max_control_points = static_cast<std::size_t>(
      declare_parameter<int>("ct_max_control_points", 64));

    const auto translation = declare_parameter<std::vector<double>>(
      "sensor_to_body_translation", {0.0, 0.0, 0.0});
    const auto rpy = declare_parameter<std::vector<double>>(
      "sensor_to_body_rpy", {0.0, 0.0, 0.0});

    cfg.sensor_to_body = MakeIsometry(
      detail::VectorFromParameter(translation),
      detail::VectorFromParameter(rpy));
    return cfg;
  }

  void OnImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    ImuSample sample;
    sample.stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
    sample.accel = Eigen::Vector3d(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);
    sample.gyro = Eigen::Vector3d(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);

    runtime_->IngestImu(sample);
    if (!runtime_->IsInitialized()) {
      return;
    }

    const PoseState odom_state = runtime_->LatestOdom();
    odom_pub_->publish(detail::ToOdometry(odom_state, odom_frame_, base_frame_, msg->header.stamp));
    tf_broadcaster_->sendTransform(
      detail::ToTransform(odom_state, odom_frame_, base_frame_, msg->header.stamp));
  }

  void OnCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const RuntimeOutputs outputs = runtime_->IngestScan(*msg);
    if (!outputs.ready) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PALLAS waiting for stationary IMU initialization before processing scans.");
      return;
    }

    pose_pub_->publish(detail::ToPoseStamped(outputs.scan_pose, odom_frame_, msg->header.stamp));
    aligned_scan_pub_->publish(
      detail::ToPointCloud2(outputs.aligned_scan, odom_frame_, msg->header.stamp));
    if (outputs.published_map) {
      map_pub_->publish(detail::ToPointCloud2(outputs.map_cloud, odom_frame_, msg->header.stamp));
    }
  }

  std::string pointcloud_topic_;
  std::string imu_topic_;
  std::string pose_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string aligned_scan_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  std::unique_ptr<RuntimeT> runtime_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace anima::pallas
