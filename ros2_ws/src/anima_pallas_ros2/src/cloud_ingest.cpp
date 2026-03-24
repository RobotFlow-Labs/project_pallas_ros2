#include <anima_pallas_ros2/cloud_ingest.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace anima::pallas {

namespace {

bool HostIsBigEndian()
{
  const std::uint16_t value = 0x0102;
  return reinterpret_cast<const std::uint8_t*>(&value)[0] == 0x01;
}

template <typename T>
T ReadScalar(const std::uint8_t* ptr, bool swap_bytes)
{
  static_assert(std::is_trivially_copyable<T>::value, "PointCloud2 scalar must be trivially copyable");

  T value{};
  std::memcpy(&value, ptr, sizeof(T));
  if (swap_bytes) {
    std::array<std::uint8_t, sizeof(T)> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(T));
    std::reverse(bytes.begin(), bytes.end());
    std::memcpy(&value, bytes.data(), sizeof(T));
  }
  return value;
}

const sensor_msgs::msg::PointField* FindField(
  const sensor_msgs::msg::PointCloud2& cloud_msg,
  const std::string& name)
{
  for (const auto& field : cloud_msg.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

const sensor_msgs::msg::PointField* FindField(
  const sensor_msgs::msg::PointCloud2& cloud_msg,
  const std::vector<std::string>& names)
{
  for (const auto& name : names) {
    if (const auto* field = FindField(cloud_msg, name)) {
      return field;
    }
  }
  return nullptr;
}

std::size_t DatatypeSize(std::uint8_t datatype)
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:
      return 1;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:
      return 2;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32:
      return 4;
    case sensor_msgs::msg::PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

std::optional<double> ReadFieldValue(
  const sensor_msgs::msg::PointCloud2& cloud_msg,
  const sensor_msgs::msg::PointField& field,
  std::size_t point_offset)
{
  const std::size_t raw_offset = point_offset + field.offset;
  const std::size_t value_size = DatatypeSize(field.datatype);
  if (value_size == 0 || raw_offset > cloud_msg.data.size() ||
    value_size > cloud_msg.data.size() - raw_offset)
  {
    return std::nullopt;
  }

  const bool swap_bytes = cloud_msg.is_bigendian != HostIsBigEndian();
  const std::uint8_t* ptr = cloud_msg.data.data() + raw_offset;

  switch (field.datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return static_cast<double>(ReadScalar<std::int8_t>(ptr, false));
    case sensor_msgs::msg::PointField::UINT8:
      return static_cast<double>(ReadScalar<std::uint8_t>(ptr, false));
    case sensor_msgs::msg::PointField::INT16:
      return static_cast<double>(ReadScalar<std::int16_t>(ptr, swap_bytes));
    case sensor_msgs::msg::PointField::UINT16:
      return static_cast<double>(ReadScalar<std::uint16_t>(ptr, swap_bytes));
    case sensor_msgs::msg::PointField::INT32:
      return static_cast<double>(ReadScalar<std::int32_t>(ptr, swap_bytes));
    case sensor_msgs::msg::PointField::UINT32:
      return static_cast<double>(ReadScalar<std::uint32_t>(ptr, swap_bytes));
    case sensor_msgs::msg::PointField::FLOAT32:
      return static_cast<double>(ReadScalar<float>(ptr, swap_bytes));
    case sensor_msgs::msg::PointField::FLOAT64:
      return ReadScalar<double>(ptr, swap_bytes);
    default:
      return std::nullopt;
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

CloudIngest::CloudIngest(CloudIngestOptions options)
: options_(std::move(options)) {}

const CloudIngestOptions& CloudIngest::options() const
{
  return options_;
}

TimedPointCloud CloudIngest::Ingest(const sensor_msgs::msg::PointCloud2& cloud_msg) const
{
  TimedPointCloud points;
  if (cloud_msg.point_step == 0 || cloud_msg.data.empty()) {
    return points;
  }

  const auto* field_x = FindField(cloud_msg, "x");
  const auto* field_y = FindField(cloud_msg, "y");
  const auto* field_z = FindField(cloud_msg, "z");
  const auto* field_i = FindField(cloud_msg, {"intensity", "reflectivity", "remission", "signal"});
  const auto* field_r = FindField(cloud_msg, {"ring", "line", "channel"});
  const auto* field_t = FindField(
    cloud_msg,
    {"timestamp", "time", "t", "offset_time", "relative_time"});

  if (!field_x || !field_y || !field_z) {
    return points;
  }

  const bool organized =
    cloud_msg.height > 0 && cloud_msg.width > 0 &&
    cloud_msg.row_step >= cloud_msg.point_step * cloud_msg.width;

  std::size_t estimated_points = 0;
  if (organized) {
    estimated_points = static_cast<std::size_t>(cloud_msg.height) * cloud_msg.width;
  } else {
    estimated_points = cloud_msg.data.size() / cloud_msg.point_step;
  }
  points.reserve(estimated_points);

  const auto ingest_at_offset = [&](std::size_t point_offset) {
    const auto x = ReadFieldValue(cloud_msg, *field_x, point_offset);
    const auto y = ReadFieldValue(cloud_msg, *field_y, point_offset);
    const auto z = ReadFieldValue(cloud_msg, *field_z, point_offset);
    if (!x || !y || !z) {
      return;
    }

    TimedPoint point;
    point.xyz = Eigen::Vector3d(*x, *y, *z);
    if (!point.xyz.allFinite()) {
      return;
    }

    const double range = point.xyz.norm();
    if (range < options_.min_range_m || range > options_.max_range_m) {
      return;
    }

    if (field_i) {
      const auto intensity = ReadFieldValue(cloud_msg, *field_i, point_offset);
      if (intensity) {
        point.intensity = static_cast<float>(*intensity);
      }
    }

    if (field_r) {
      const auto raw_ring = ReadFieldValue(cloud_msg, *field_r, point_offset);
      if (raw_ring) {
        const long long clamped_ring = std::clamp(
          static_cast<long long>(std::llround(*raw_ring)),
          0LL,
          static_cast<long long>(std::numeric_limits<std::uint16_t>::max()));
        point.ring = static_cast<std::uint16_t>(clamped_ring);
      }
    }

    if (field_t) {
      const auto raw_time = ReadFieldValue(cloud_msg, *field_t, point_offset);
      if (raw_time) {
        point.relative_time_sec = options_.normalize_relative_time
          ? NormalizeRelativeTime(*raw_time)
          : *raw_time;
      }
    }

    points.push_back(point);
  };

  if (organized) {
    for (std::uint32_t row = 0; row < cloud_msg.height; ++row) {
      const std::size_t row_offset = static_cast<std::size_t>(row) * cloud_msg.row_step;
      for (std::uint32_t col = 0; col < cloud_msg.width; ++col) {
        const std::size_t point_offset = row_offset + static_cast<std::size_t>(col) * cloud_msg.point_step;
        if (point_offset + cloud_msg.point_step > cloud_msg.data.size()) {
          continue;
        }
        ingest_at_offset(point_offset);
      }
    }
  } else {
    for (std::size_t idx = 0; idx < estimated_points; ++idx) {
      const std::size_t point_offset = idx * cloud_msg.point_step;
      if (point_offset + cloud_msg.point_step > cloud_msg.data.size()) {
        break;
      }
      ingest_at_offset(point_offset);
    }
  }

  if (field_t && !points.empty()) {
    double min_time = points.front().relative_time_sec;
    for (const auto& point : points) {
      min_time = std::min(min_time, point.relative_time_sec);
    }
    for (auto& point : points) {
      point.relative_time_sec -= min_time;
    }
  }

  return points;
}

}  // namespace anima::pallas
