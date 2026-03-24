#include <anima_pallas_ros2/point_parser.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace anima::pallas {

namespace {

const sensor_msgs::msg::PointField* FindField(
  const sensor_msgs::msg::PointCloud2& cloud_msg,
  const std::initializer_list<std::string>& names)
{
  for (const auto& name : names) {
    for (const auto& field : cloud_msg.fields) {
      if (field.name == name) {
        return &field;
      }
    }
  }
  return nullptr;
}

double ReadField(
  const sensor_msgs::msg::PointCloud2& cloud_msg,
  const sensor_msgs::msg::PointField& field,
  std::size_t point_index)
{
  const std::size_t offset = point_index * cloud_msg.point_step + field.offset;
  const auto* ptr = cloud_msg.data.data() + offset;

  switch (field.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32:
      return *reinterpret_cast<const float*>(ptr);
    case sensor_msgs::msg::PointField::FLOAT64:
      return *reinterpret_cast<const double*>(ptr);
    case sensor_msgs::msg::PointField::UINT32:
      return static_cast<double>(*reinterpret_cast<const std::uint32_t*>(ptr));
    case sensor_msgs::msg::PointField::INT32:
      return static_cast<double>(*reinterpret_cast<const std::int32_t*>(ptr));
    case sensor_msgs::msg::PointField::UINT16:
      return static_cast<double>(*reinterpret_cast<const std::uint16_t*>(ptr));
    case sensor_msgs::msg::PointField::INT16:
      return static_cast<double>(*reinterpret_cast<const std::int16_t*>(ptr));
    default:
      return 0.0;
  }
}

double NormalizeRelativeTime(double raw_time)
{
  double value = raw_time;
  if (value > 1e9) {
    value *= 1e-9;
  } else if (value > 1e6) {
    value *= 1e-6;
  } else if (value > 1e3) {
    value *= 1e-3;
  }
  return value;
}

}  // namespace

TimedPointCloud PointParser::Parse(
  const sensor_msgs::msg::PointCloud2& cloud_msg,
  const std::string& /*lidar_type*/) const
{
  TimedPointCloud points;
  const auto* field_x = FindField(cloud_msg, {"x"});
  const auto* field_y = FindField(cloud_msg, {"y"});
  const auto* field_z = FindField(cloud_msg, {"z"});
  const auto* field_i = FindField(cloud_msg, {"intensity", "reflectivity"});
  const auto* field_t = FindField(cloud_msg, {"timestamp", "time", "t", "offset_time"});

  if (!field_x || !field_y || !field_z || cloud_msg.point_step == 0) {
    return points;
  }

  const std::size_t num_points =
    cloud_msg.point_step > 0 ? cloud_msg.data.size() / cloud_msg.point_step : 0;
  points.reserve(num_points);

  for (std::size_t idx = 0; idx < num_points; ++idx) {
    TimedPoint point;
    point.xyz.x() = ReadField(cloud_msg, *field_x, idx);
    point.xyz.y() = ReadField(cloud_msg, *field_y, idx);
    point.xyz.z() = ReadField(cloud_msg, *field_z, idx);
    if (!std::isfinite(point.xyz.x()) || !std::isfinite(point.xyz.y()) ||
      !std::isfinite(point.xyz.z()))
    {
      continue;
    }

    if (field_i) {
      point.intensity = static_cast<float>(ReadField(cloud_msg, *field_i, idx));
    }
    if (field_t) {
      point.relative_time_sec = NormalizeRelativeTime(ReadField(cloud_msg, *field_t, idx));
    }
    points.push_back(point);
  }

  return points;
}

}  // namespace anima::pallas
