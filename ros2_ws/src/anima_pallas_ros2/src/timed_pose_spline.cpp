#include <anima_pallas_ros2/timed_pose_spline.hpp>

#include <algorithm>
#include <cmath>

namespace anima::pallas {

void TimedPoseSpline::Clear()
{
  control_points_.clear();
}

bool TimedPoseSpline::Empty() const
{
  return control_points_.empty();
}

std::size_t TimedPoseSpline::Size() const
{
  return control_points_.size();
}

const std::vector<TimedPoseSpline::ControlPoint>& TimedPoseSpline::ControlPoints() const
{
  return control_points_;
}

double TimedPoseSpline::ClampUnitInterval(double u)
{
  return std::clamp(u, 0.0, 1.0);
}

bool TimedPoseSpline::InsertControlPoint(const ControlPoint& control_point)
{
  ControlPoint normalized = control_point;
  normalized.orientation = detail::NormalizeQuaternion(normalized.orientation);

  const auto comparator = [](const ControlPoint& point, double stamp_sec) {
    return point.stamp_sec < stamp_sec;
  };

  const auto it = std::lower_bound(
    control_points_.begin(), control_points_.end(), normalized.stamp_sec, comparator);

  if (it != control_points_.end() && std::abs(it->stamp_sec - normalized.stamp_sec) <= 1e-12) {
    *it = normalized;
  } else {
    control_points_.insert(it, normalized);
  }

  return true;
}

bool TimedPoseSpline::EraseControlPoint(double stamp_sec)
{
  const auto it = std::lower_bound(
    control_points_.begin(), control_points_.end(), stamp_sec,
    [](const ControlPoint& point, double value) { return point.stamp_sec < value; });

  if (it == control_points_.end() || std::abs(it->stamp_sec - stamp_sec) > 1e-12) {
    return false;
  }

  control_points_.erase(it);
  return true;
}

std::vector<Eigen::Quaterniond> TimedPoseSpline::BuildAlignedOrientations(
  const ControlPointList& control_points)
{
  std::vector<Eigen::Quaterniond> orientations;
  orientations.reserve(control_points.size());
  if (control_points.empty()) {
    return orientations;
  }

  orientations.push_back(detail::NormalizeQuaternion(control_points.front().orientation));
  for (std::size_t index = 1; index < control_points.size(); ++index) {
    Eigen::Quaterniond orientation = detail::NormalizeQuaternion(control_points[index].orientation);
    orientation = detail::AlignQuaternion(orientations.back(), orientation);
    orientations.push_back(orientation);
  }

  return orientations;
}

std::vector<Eigen::Vector3d> TimedPoseSpline::BuildLinearTangents(
  const ControlPointList& control_points)
{
  std::vector<Eigen::Vector3d> tangents;
  tangents.reserve(control_points.size());
  for (std::size_t index = 0; index < control_points.size(); ++index) {
    tangents.push_back(EstimateLinearTangent(control_points, index));
  }
  return tangents;
}

Eigen::Vector3d TimedPoseSpline::EstimateLinearTangent(
  const ControlPointList& control_points,
  std::size_t index)
{
  if (control_points.size() < 2) {
    return Eigen::Vector3d::Zero();
  }

  if (index == 0) {
    const double dt = control_points[1].stamp_sec - control_points[0].stamp_sec;
    if (dt <= 1e-12) {
      return Eigen::Vector3d::Zero();
    }
    return (control_points[1].position - control_points[0].position) / dt;
  }

  if (index + 1 == control_points.size()) {
    const double dt = control_points[index].stamp_sec - control_points[index - 1].stamp_sec;
    if (dt <= 1e-12) {
      return Eigen::Vector3d::Zero();
    }
    return (control_points[index].position - control_points[index - 1].position) / dt;
  }

  const double dt_prev = control_points[index].stamp_sec - control_points[index - 1].stamp_sec;
  const double dt_next = control_points[index + 1].stamp_sec - control_points[index].stamp_sec;
  if (dt_prev <= 1e-12 || dt_next <= 1e-12) {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d slope_prev =
    (control_points[index].position - control_points[index - 1].position) / dt_prev;
  const Eigen::Vector3d slope_next =
    (control_points[index + 1].position - control_points[index].position) / dt_next;
  return (dt_next * slope_prev + dt_prev * slope_next) / (dt_prev + dt_next);
}

Eigen::Quaterniond TimedPoseSpline::EstimateSquadTangent(
  const std::vector<Eigen::Quaterniond>& orientations,
  std::size_t index)
{
  if (orientations.empty()) {
    return Eigen::Quaterniond::Identity();
  }

  const Eigen::Quaterniond& current = orientations[index];
  if (orientations.size() == 1) {
    return current;
  }

  if (index == 0) {
    return detail::NormalizeQuaternion(
      current * detail::QuaternionExp(
                 -0.5 * detail::QuaternionLog(current.conjugate() * orientations[1])));
  }

  if (index + 1 == orientations.size()) {
    return detail::NormalizeQuaternion(
      current * detail::QuaternionExp(
                 -0.5 * detail::QuaternionLog(current.conjugate() * orientations[index - 1])));
  }

  const Eigen::Vector3d log_prev = detail::QuaternionLog(current.conjugate() * orientations[index - 1]);
  const Eigen::Vector3d log_next = detail::QuaternionLog(current.conjugate() * orientations[index + 1]);
  return detail::NormalizeQuaternion(
    current * detail::QuaternionExp(-0.25 * (log_prev + log_next)));
}

std::size_t TimedPoseSpline::ClampSegmentIndex(
  const ControlPointList& control_points,
  double stamp_sec)
{
  if (control_points.size() <= 1) {
    return 0;
  }

  if (stamp_sec <= control_points.front().stamp_sec) {
    return 0;
  }

  if (stamp_sec >= control_points.back().stamp_sec) {
    return control_points.size() - 2;
  }

  const auto upper = std::lower_bound(
    control_points.begin(), control_points.end(), stamp_sec,
    [](const ControlPoint& point, double value) { return point.stamp_sec < value; });
  if (upper == control_points.begin()) {
    return 0;
  }

  return static_cast<std::size_t>(std::distance(control_points.begin(), upper) - 1);
}

double TimedPoseSpline::SegmentParameter(
  const ControlPointList& control_points,
  std::size_t segment_index,
  double stamp_sec)
{
  if (control_points.size() <= 1) {
    return 0.0;
  }

  const double t0 = control_points[segment_index].stamp_sec;
  const double t1 = control_points[segment_index + 1].stamp_sec;
  const double dt = std::max(t1 - t0, 1e-12);
  return ClampUnitInterval((stamp_sec - t0) / dt);
}

TimedPoseSpline::PoseSample TimedPoseSpline::EvaluateSegmentPose(
  const ControlPointList& control_points,
  const std::vector<Eigen::Quaterniond>& orientations,
  const std::vector<Eigen::Vector3d>& linear_tangents,
  std::size_t segment_index,
  double stamp_sec)
{
  PoseSample sample;
  sample.stamp_sec = stamp_sec;
  sample.position = EvaluateSegmentPosition(control_points, linear_tangents, segment_index, stamp_sec);
  sample.orientation = EvaluateSegmentOrientation(control_points, orientations, segment_index, stamp_sec);
  return sample;
}

Eigen::Vector3d TimedPoseSpline::EvaluateSegmentPosition(
  const ControlPointList& control_points,
  const std::vector<Eigen::Vector3d>& linear_tangents,
  std::size_t segment_index,
  double stamp_sec)
{
  const double u = SegmentParameter(control_points, segment_index, stamp_sec);
  const double dt = std::max(
    control_points[segment_index + 1].stamp_sec - control_points[segment_index].stamp_sec, 1e-12);
  return detail::HermitePosition(
    control_points[segment_index].position,
    linear_tangents[segment_index],
    control_points[segment_index + 1].position,
    linear_tangents[segment_index + 1],
    dt,
    u);
}

Eigen::Vector3d TimedPoseSpline::EvaluateSegmentLinearVelocity(
  const ControlPointList& control_points,
  const std::vector<Eigen::Vector3d>& linear_tangents,
  std::size_t segment_index,
  double stamp_sec)
{
  const double u = SegmentParameter(control_points, segment_index, stamp_sec);
  const double dt = std::max(
    control_points[segment_index + 1].stamp_sec - control_points[segment_index].stamp_sec, 1e-12);
  return detail::HermiteVelocity(
    control_points[segment_index].position,
    linear_tangents[segment_index],
    control_points[segment_index + 1].position,
    linear_tangents[segment_index + 1],
    dt,
    u);
}

Eigen::Vector3d TimedPoseSpline::EvaluateSegmentLinearAcceleration(
  const ControlPointList& control_points,
  const std::vector<Eigen::Vector3d>& linear_tangents,
  std::size_t segment_index,
  double stamp_sec)
{
  const double u = SegmentParameter(control_points, segment_index, stamp_sec);
  const double dt = std::max(
    control_points[segment_index + 1].stamp_sec - control_points[segment_index].stamp_sec, 1e-12);
  return detail::HermiteAcceleration(
    control_points[segment_index].position,
    linear_tangents[segment_index],
    control_points[segment_index + 1].position,
    linear_tangents[segment_index + 1],
    dt,
    u);
}

Eigen::Quaterniond TimedPoseSpline::EvaluateSegmentOrientation(
  const ControlPointList& control_points,
  const std::vector<Eigen::Quaterniond>& orientations,
  std::size_t segment_index,
  double stamp_sec)
{
  const double u = SegmentParameter(control_points, segment_index, stamp_sec);
  const Eigen::Quaterniond q0 = orientations[segment_index];
  const Eigen::Quaterniond q1 = orientations[segment_index + 1];
  const Eigen::Quaterniond s0 = EstimateSquadTangent(orientations, segment_index);
  const Eigen::Quaterniond s1 = EstimateSquadTangent(orientations, segment_index + 1);

  const Eigen::Quaterniond qa = detail::SlerpShortest(q0, q1, u);
  const Eigen::Quaterniond qb = detail::SlerpShortest(s0, s1, u);
  const Eigen::Quaterniond orientation = detail::SlerpShortest(qa, qb, 2.0 * u * (1.0 - u));
  return detail::NormalizeQuaternion(orientation);
}

std::optional<TimedPoseSpline::PoseSample> TimedPoseSpline::EvaluatePose(double stamp_sec) const
{
  if (control_points_.empty()) {
    return std::nullopt;
  }

  if (control_points_.size() == 1) {
    PoseSample sample;
    sample.stamp_sec = stamp_sec;
    sample.position = control_points_.front().position;
    sample.orientation = detail::NormalizeQuaternion(control_points_.front().orientation);
    return sample;
  }

  const std::size_t segment_index = ClampSegmentIndex(control_points_, stamp_sec);
  const auto orientations = BuildAlignedOrientations(control_points_);
  const auto linear_tangents = BuildLinearTangents(control_points_);
  return EvaluateSegmentPose(
    control_points_, orientations, linear_tangents, segment_index, stamp_sec);
}

std::optional<Eigen::Vector3d> TimedPoseSpline::EvaluatePosition(double stamp_sec) const
{
  const auto pose = EvaluatePose(stamp_sec);
  if (!pose.has_value()) {
    return std::nullopt;
  }
  return pose->position;
}

std::optional<Eigen::Quaterniond> TimedPoseSpline::EvaluateOrientation(double stamp_sec) const
{
  const auto pose = EvaluatePose(stamp_sec);
  if (!pose.has_value()) {
    return std::nullopt;
  }
  return pose->orientation;
}

std::optional<Eigen::Vector3d> TimedPoseSpline::EvaluateLinearVelocity(double stamp_sec) const
{
  if (control_points_.empty()) {
    return std::nullopt;
  }
  if (control_points_.size() == 1) {
    return Eigen::Vector3d::Zero();
  }

  const std::size_t segment_index = ClampSegmentIndex(control_points_, stamp_sec);
  const auto linear_tangents = BuildLinearTangents(control_points_);
  return EvaluateSegmentLinearVelocity(control_points_, linear_tangents, segment_index, stamp_sec);
}

std::optional<Eigen::Vector3d> TimedPoseSpline::EvaluateLinearAcceleration(double stamp_sec) const
{
  if (control_points_.empty()) {
    return std::nullopt;
  }
  if (control_points_.size() == 1) {
    return Eigen::Vector3d::Zero();
  }

  const std::size_t segment_index = ClampSegmentIndex(control_points_, stamp_sec);
  const auto linear_tangents = BuildLinearTangents(control_points_);
  return EvaluateSegmentLinearAcceleration(control_points_, linear_tangents, segment_index, stamp_sec);
}

double TimedPoseSpline::AngularDifferenceStep(
  const ControlPointList& control_points,
  double stamp_sec)
{
  if (control_points.size() < 2) {
    return 1e-3;
  }

  const std::size_t segment_index = ClampSegmentIndex(control_points, stamp_sec);
  const double dt = std::max(
    control_points[segment_index + 1].stamp_sec - control_points[segment_index].stamp_sec, 1e-12);
  return std::clamp(0.25 * dt, 1e-6, 1e-3);
}

Eigen::Vector3d TimedPoseSpline::FiniteDifferenceAngularVelocity(
  const ControlPointList& control_points,
  double stamp_sec)
{
  if (control_points.size() < 2) {
    return Eigen::Vector3d::Zero();
  }

  const auto orientations = BuildAlignedOrientations(control_points);
  const double step = AngularDifferenceStep(control_points, stamp_sec);

  const std::size_t seg_minus = ClampSegmentIndex(control_points, stamp_sec - step);
  const auto orientation_minus = EvaluateSegmentOrientation(
    control_points, orientations, seg_minus, stamp_sec - step);

  const std::size_t seg_plus = ClampSegmentIndex(control_points, stamp_sec + step);
  const auto orientation_plus = EvaluateSegmentOrientation(
    control_points, orientations, seg_plus, stamp_sec + step);

  Eigen::Quaterniond aligned_plus = orientation_plus;
  if (orientation_minus.coeffs().dot(aligned_plus.coeffs()) < 0.0) {
    aligned_plus.coeffs() *= -1.0;
  }

  const Eigen::Quaterniond delta = orientation_minus.conjugate() * aligned_plus;
  return detail::QuaternionLog(delta) / (2.0 * step);
}

std::optional<Eigen::Vector3d> TimedPoseSpline::EvaluateAngularVelocity(double stamp_sec) const
{
  if (control_points_.empty()) {
    return std::nullopt;
  }
  if (control_points_.size() == 1) {
    return Eigen::Vector3d::Zero();
  }

  return FiniteDifferenceAngularVelocity(control_points_, stamp_sec);
}

std::optional<Eigen::Vector3d> TimedPoseSpline::EvaluateAngularAcceleration(double stamp_sec) const
{
  if (control_points_.empty()) {
    return std::nullopt;
  }
  if (control_points_.size() == 1) {
    return Eigen::Vector3d::Zero();
  }

  const double step = AngularDifferenceStep(control_points_, stamp_sec);
  const Eigen::Vector3d omega_minus = FiniteDifferenceAngularVelocity(control_points_, stamp_sec - step);
  const Eigen::Vector3d omega_plus = FiniteDifferenceAngularVelocity(control_points_, stamp_sec + step);
  return (omega_plus - omega_minus) / (2.0 * step);
}

std::optional<TimedPoseSpline::DerivativeSample> TimedPoseSpline::EvaluateDerivatives(
  double stamp_sec) const
{
  const auto velocity = EvaluateLinearVelocity(stamp_sec);
  const auto acceleration = EvaluateLinearAcceleration(stamp_sec);
  const auto angular_velocity = EvaluateAngularVelocity(stamp_sec);
  const auto angular_acceleration = EvaluateAngularAcceleration(stamp_sec);
  if (!velocity.has_value() || !acceleration.has_value() ||
      !angular_velocity.has_value() || !angular_acceleration.has_value()) {
    return std::nullopt;
  }

  DerivativeSample sample;
  sample.linear_velocity = *velocity;
  sample.linear_acceleration = *acceleration;
  sample.angular_velocity = *angular_velocity;
  sample.angular_acceleration = *angular_acceleration;
  return sample;
}

}  // namespace anima::pallas
