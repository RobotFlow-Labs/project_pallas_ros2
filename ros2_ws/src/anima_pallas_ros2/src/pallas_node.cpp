#include <anima_pallas_ros2/config.hpp>
#include <anima_pallas_ros2/pipeline.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace anima::pallas {

namespace {

geometry_msgs::msg::PoseStamped ToPoseStamped(
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

nav_msgs::msg::Odometry ToOdometry(
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

geometry_msgs::msg::TransformStamped ToTransform(
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

sensor_msgs::msg::PointCloud2 ToPointCloud2(
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

  msg.fields.resize(5);
  const std::array<std::string, 5> names{"x", "y", "z", "intensity", "time"};
  for (std::size_t i = 0; i < msg.fields.size(); ++i) {
    msg.fields[i].name = names[i];
    msg.fields[i].offset = static_cast<std::uint32_t>(i * sizeof(float));
    msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[i].count = 1;
  }
  msg.point_step = 5 * sizeof(float);
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);

  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const float values[5] = {
      static_cast<float>(cloud[i].xyz.x()),
      static_cast<float>(cloud[i].xyz.y()),
      static_cast<float>(cloud[i].xyz.z()),
      cloud[i].intensity,
      static_cast<float>(cloud[i].relative_time_sec)};
    std::memcpy(msg.data.data() + i * msg.point_step, values, sizeof(values));
  }

  return msg;
}

}  // namespace

class PallasNode : public rclcpp::Node {
public:
  PallasNode()
  : Node("anima_pallas_node"),
    pipeline_(LoadConfig())
  {
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/livox/lidar");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/pallas/pose");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/pallas/odom");
    map_topic_ = declare_parameter<std::string>("map_topic", "/pallas/local_map");
    aligned_scan_topic_ = declare_parameter<std::string>("aligned_scan_topic", "/pallas/aligned_scan");
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
      std::bind(&PallasNode::OnImu, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PallasNode::OnCloud, this, std::placeholders::_1));
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
    cfg.map_voxel_size_m = declare_parameter<double>("map_voxel_size_m", 0.20);
    cfg.max_imu_buffer_size = static_cast<std::size_t>(
      declare_parameter<int>("max_imu_buffer_size", 20000));
    cfg.max_map_scans = static_cast<std::size_t>(declare_parameter<int>("max_map_scans", 20));
    cfg.map_publish_period_scans = static_cast<std::size_t>(
      declare_parameter<int>("map_publish_period_scans", 1));

    const auto translation = declare_parameter<std::vector<double>>(
      "sensor_to_body_translation", {0.0, 0.0, 0.0});
    const auto rpy = declare_parameter<std::vector<double>>(
      "sensor_to_body_rpy", {0.0, 0.0, 0.0});

    cfg.sensor_to_body = MakeIsometry(
      Eigen::Vector3d(translation[0], translation[1], translation[2]),
      Eigen::Vector3d(rpy[0], rpy[1], rpy[2]));
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

    pipeline_.IngestImu(sample);
    if (!pipeline_.IsInitialized()) {
      return;
    }

    const PoseState odom_state = pipeline_.LatestOdom();
    odom_pub_->publish(ToOdometry(odom_state, odom_frame_, base_frame_, msg->header.stamp));
    tf_broadcaster_->sendTransform(ToTransform(odom_state, odom_frame_, base_frame_, msg->header.stamp));
  }

  void OnCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const PipelineOutputs outputs = pipeline_.IngestScan(*msg);
    if (!outputs.ready) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PALLAS waiting for stationary IMU initialization before processing scans.");
      return;
    }

    pose_pub_->publish(ToPoseStamped(outputs.scan_pose, odom_frame_, msg->header.stamp));
    aligned_scan_pub_->publish(ToPointCloud2(outputs.aligned_scan, odom_frame_, msg->header.stamp));
    if (outputs.published_map) {
      map_pub_->publish(ToPointCloud2(outputs.map_cloud, odom_frame_, msg->header.stamp));
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

  PallasPipeline pipeline_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace anima::pallas

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<anima::pallas::PallasNode>());
  rclcpp::shutdown();
  return 0;
}
