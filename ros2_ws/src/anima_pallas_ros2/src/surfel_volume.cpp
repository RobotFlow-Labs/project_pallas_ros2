#include <anima_pallas_ros2/surfel_volume.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <utility>

namespace anima::pallas {

namespace {

Eigen::Vector3d FallbackNormal(const Eigen::Vector3d& point)
{
  if (point.norm() > 1e-9) {
    return -point.normalized();
  }
  return Eigen::Vector3d::UnitZ();
}

std::size_t SupportWeight(std::size_t support)
{
  return std::max<std::size_t>(1, support);
}

}  // namespace

std::size_t SurfelVolume::VoxelKeyHash::operator()(const VoxelKey& key) const
{
  std::size_t seed = std::hash<std::int64_t>{}(key.x);
  seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

SurfelVolume::SurfelVolume(SurfelVolumeOptions options)
: options_(std::move(options))
{
  if (options_.voxel_size_m <= 0.0) {
    options_.voxel_size_m = 0.20;
  }
}

const SurfelVolumeOptions& SurfelVolume::options() const
{
  return options_;
}

void SurfelVolume::Reset()
{
  surfels_.clear();
  sequence_ = 0;
}

SurfelVolume::VoxelKey SurfelVolume::MakeKey(const Eigen::Vector3d& point) const
{
  const double voxel = options_.voxel_size_m > 0.0 ? options_.voxel_size_m : 0.20;
  return {
    static_cast<std::int64_t>(std::floor(point.x() / voxel)),
    static_cast<std::int64_t>(std::floor(point.y() / voxel)),
    static_cast<std::int64_t>(std::floor(point.z() / voxel))};
}

void SurfelVolume::CullExpired(double stamp_sec)
{
  if (options_.max_age_sec <= 0.0) {
    return;
  }

  const double oldest_allowed = stamp_sec - options_.max_age_sec;
  for (auto it = surfels_.begin(); it != surfels_.end();) {
    if (it->second.surfel.stamp_sec < oldest_allowed) {
      it = surfels_.erase(it);
    } else {
      ++it;
    }
  }
}

void SurfelVolume::PruneToLimit()
{
  if (options_.max_surfels == 0 || surfels_.size() <= options_.max_surfels) {
    return;
  }

  while (surfels_.size() > options_.max_surfels) {
    auto oldest = surfels_.begin();
    for (auto it = std::next(surfels_.begin()); it != surfels_.end(); ++it) {
      if (it->second.surfel.stamp_sec < oldest->second.surfel.stamp_sec) {
        oldest = it;
        continue;
      }
      if (it->second.surfel.stamp_sec == oldest->second.surfel.stamp_sec &&
        it->second.sequence < oldest->second.sequence)
      {
        oldest = it;
      }
    }
    surfels_.erase(oldest);
  }
}

void SurfelVolume::Integrate(const SampledScan& scan, double stamp_sec)
{
  CullExpired(stamp_sec);

  for (const auto& sample : scan) {
    if (!sample.point.xyz.allFinite() || !sample.normal.allFinite()) {
      continue;
    }

    const Eigen::Vector3d normal_in = sample.normal.norm() > 1e-9
      ? sample.normal.normalized()
      : FallbackNormal(sample.point.xyz);
    if (!normal_in.allFinite()) {
      continue;
    }

    const VoxelKey key = MakeKey(sample.point.xyz);
    auto& entry = surfels_[key];
    const double w_new = static_cast<double>(SupportWeight(sample.support));
    const double point_stamp = stamp_sec + sample.point.relative_time_sec;

    if (entry.surfel.weight <= 0.0) {
      entry.surfel.position = sample.point.xyz;
      entry.surfel.normal = normal_in;
      entry.surfel.intensity = sample.point.intensity;
      entry.surfel.stamp_sec = point_stamp;
      entry.surfel.weight = w_new;
      entry.surfel.support = SupportWeight(sample.support);
      entry.surfel.radius_m = 0.5 * options_.voxel_size_m;
      entry.sequence = ++sequence_;
      continue;
    }

    Eigen::Vector3d blended_normal = normal_in;
    if (blended_normal.dot(entry.surfel.normal) < 0.0) {
      blended_normal = -blended_normal;
    }

    const double w_old = entry.surfel.weight;
    const double w_total = w_old + w_new;
    entry.surfel.position = (entry.surfel.position * w_old + sample.point.xyz * w_new) / w_total;
    entry.surfel.normal = entry.surfel.normal * w_old + blended_normal * w_new;
    if (entry.surfel.normal.norm() < 1e-9) {
      entry.surfel.normal = blended_normal;
    }
    entry.surfel.normal.normalize();
    if (entry.surfel.normal.dot(entry.surfel.position) > 0.0) {
      entry.surfel.normal = -entry.surfel.normal;
    }
    entry.surfel.intensity = static_cast<float>(
      (static_cast<double>(entry.surfel.intensity) * w_old +
      static_cast<double>(sample.point.intensity) * w_new) / w_total);
    entry.surfel.stamp_sec = point_stamp;
    entry.surfel.weight = w_total;
    entry.surfel.support += SupportWeight(sample.support);

    const double distance_to_mean = (sample.point.xyz - entry.surfel.position).norm();
    entry.surfel.radius_m = std::max(entry.surfel.radius_m, distance_to_mean);
    entry.sequence = ++sequence_;
  }

  CullExpired(stamp_sec);
  PruneToLimit();
}

std::size_t SurfelVolume::Size() const
{
  return surfels_.size();
}

std::vector<Surfel> SurfelVolume::Snapshot() const
{
  std::vector<Surfel> surfels;
  surfels.reserve(surfels_.size());
  for (const auto& [key, entry] : surfels_) {
    (void)key;
    surfels.push_back(entry.surfel);
  }

  std::sort(
    surfels.begin(), surfels.end(),
    [](const Surfel& lhs, const Surfel& rhs) {
      if (lhs.stamp_sec != rhs.stamp_sec) {
        return lhs.stamp_sec < rhs.stamp_sec;
      }
      if (lhs.position.x() != rhs.position.x()) {
        return lhs.position.x() < rhs.position.x();
      }
      if (lhs.position.y() != rhs.position.y()) {
        return lhs.position.y() < rhs.position.y();
      }
      return lhs.position.z() < rhs.position.z();
    });
  return surfels;
}

TimedPointCloud SurfelVolume::Cloud() const
{
  const std::vector<Surfel> surfels = Snapshot();
  TimedPointCloud cloud;
  cloud.reserve(surfels.size());
  for (const auto& surfel : surfels) {
    TimedPoint point;
    point.xyz = surfel.position;
    point.intensity = surfel.intensity;
    point.relative_time_sec = surfel.stamp_sec;
    cloud.push_back(point);
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 SurfelVolume::ToPointCloud2(
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp) const
{
  const std::vector<Surfel> surfels = Snapshot();

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.height = 1;
  msg.width = static_cast<std::uint32_t>(surfels.size());
  msg.is_dense = false;
  msg.is_bigendian = false;

  msg.fields.resize(10);
  const std::array<std::string, 10> names{
    "x", "y", "z", "intensity", "normal_x", "normal_y", "normal_z", "weight", "time", "support"};
  for (std::size_t i = 0; i < msg.fields.size(); ++i) {
    msg.fields[i].name = names[i];
    msg.fields[i].offset = static_cast<std::uint32_t>(i * sizeof(float));
    msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[i].count = 1;
  }

  msg.point_step = static_cast<std::uint32_t>(msg.fields.size() * sizeof(float));
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);

  for (std::size_t i = 0; i < surfels.size(); ++i) {
    const float values[10] = {
      static_cast<float>(surfels[i].position.x()),
      static_cast<float>(surfels[i].position.y()),
      static_cast<float>(surfels[i].position.z()),
      surfels[i].intensity,
      static_cast<float>(surfels[i].normal.x()),
      static_cast<float>(surfels[i].normal.y()),
      static_cast<float>(surfels[i].normal.z()),
      static_cast<float>(surfels[i].weight),
      static_cast<float>(surfels[i].stamp_sec),
      static_cast<float>(surfels[i].support)};
    std::memcpy(msg.data.data() + i * msg.point_step, values, sizeof(values));
  }

  return msg;
}

}  // namespace anima::pallas
