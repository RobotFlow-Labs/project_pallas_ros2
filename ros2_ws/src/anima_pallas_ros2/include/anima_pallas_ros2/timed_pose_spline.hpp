#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace anima::pallas {

namespace detail {

inline Eigen::Quaterniond NormalizeQuaternion(Eigen::Quaterniond q)
{
  const double norm = q.norm();
  if (norm <= 0.0) {
    return Eigen::Quaterniond::Identity();
  }
  q.coeffs() /= norm;
  return q;
}

inline Eigen::Quaterniond AlignQuaternion(
  const Eigen::Quaterniond& reference,
  Eigen::Quaterniond candidate)
{
  if (reference.coeffs().dot(candidate.coeffs()) < 0.0) {
    candidate.coeffs() *= -1.0;
  }
  return candidate;
}

inline Eigen::Quaterniond QuaternionExp(const Eigen::Vector3d& omega)
{
  const double angle = omega.norm();
  if (angle <= 1e-12) {
    return NormalizeQuaternion(Eigen::Quaterniond(
      1.0, 0.5 * omega.x(), 0.5 * omega.y(), 0.5 * omega.z()));
  }

  const Eigen::Vector3d axis = omega / angle;
  const double half_angle = 0.5 * angle;
  return Eigen::Quaterniond(
    std::cos(half_angle),
    axis.x() * std::sin(half_angle),
    axis.y() * std::sin(half_angle),
    axis.z() * std::sin(half_angle));
}

inline Eigen::Vector3d QuaternionLog(Eigen::Quaterniond q)
{
  q = NormalizeQuaternion(q);
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }

  const Eigen::Vector3d vec(q.x(), q.y(), q.z());
  const double vec_norm = vec.norm();
  if (vec_norm <= 1e-12) {
    return 2.0 * vec;
  }

  const double angle = 2.0 * std::atan2(vec_norm, q.w());
  return vec * (angle / vec_norm);
}

inline Eigen::Quaterniond SlerpShortest(
  const Eigen::Quaterniond& lhs,
  const Eigen::Quaterniond& rhs,
  double u)
{
  const double alpha = std::clamp(u, 0.0, 1.0);
  Eigen::Quaterniond aligned_rhs = AlignQuaternion(lhs, rhs);
  const double dot = lhs.coeffs().dot(aligned_rhs.coeffs());
  if (dot > 0.9995) {
    Eigen::Quaterniond blended;
    blended.coeffs() = (1.0 - alpha) * lhs.coeffs() + alpha * aligned_rhs.coeffs();
    return NormalizeQuaternion(blended);
  }

  Eigen::Quaterniond result = lhs.slerp(alpha, aligned_rhs);
  return NormalizeQuaternion(result);
}

inline Eigen::Vector3d HermitePosition(
  const Eigen::Vector3d& p0,
  const Eigen::Vector3d& v0,
  const Eigen::Vector3d& p1,
  const Eigen::Vector3d& v1,
  double dt,
  double u)
{
  const double uu = std::clamp(u, 0.0, 1.0);
  const double uu2 = uu * uu;
  const double uu3 = uu2 * uu;

  const double h00 = 2.0 * uu3 - 3.0 * uu2 + 1.0;
  const double h10 = uu3 - 2.0 * uu2 + uu;
  const double h01 = -2.0 * uu3 + 3.0 * uu2;
  const double h11 = uu3 - uu2;
  return h00 * p0 + h10 * dt * v0 + h01 * p1 + h11 * dt * v1;
}

inline Eigen::Vector3d HermiteVelocity(
  const Eigen::Vector3d& p0,
  const Eigen::Vector3d& v0,
  const Eigen::Vector3d& p1,
  const Eigen::Vector3d& v1,
  double dt,
  double u)
{
  const double uu = std::clamp(u, 0.0, 1.0);
  const double uu2 = uu * uu;

  const double dh00 = 6.0 * uu2 - 6.0 * uu;
  const double dh10 = 3.0 * uu2 - 4.0 * uu + 1.0;
  const double dh01 = -dh00;
  const double dh11 = 3.0 * uu2 - 2.0 * uu;
  return (dh00 * p0 + dh10 * dt * v0 + dh01 * p1 + dh11 * dt * v1) / std::max(dt, 1e-12);
}

inline Eigen::Vector3d HermiteAcceleration(
  const Eigen::Vector3d& p0,
  const Eigen::Vector3d& v0,
  const Eigen::Vector3d& p1,
  const Eigen::Vector3d& v1,
  double dt,
  double u)
{
  const double uu = std::clamp(u, 0.0, 1.0);
  const double d2h00 = 12.0 * uu - 6.0;
  const double d2h10 = 6.0 * uu - 4.0;
  const double d2h01 = -d2h00;
  const double d2h11 = 6.0 * uu - 2.0;
  const double dt2 = std::max(dt * dt, 1e-24);
  return (d2h00 * p0 + d2h10 * dt * v0 + d2h01 * p1 + d2h11 * dt * v1) / dt2;
}

}  // namespace detail

class TimedPoseSpline {
public:
  struct ControlPoint {
    double stamp_sec{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  };

  struct PoseSample {
    double stamp_sec{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  };

  struct DerivativeSample {
    Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};
  };

  TimedPoseSpline() = default;

  void Clear();
  bool Empty() const;
  std::size_t Size() const;
  const std::vector<ControlPoint>& ControlPoints() const;

  bool InsertControlPoint(const ControlPoint& control_point);
  bool Insert(const ControlPoint& control_point) { return InsertControlPoint(control_point); }

  bool EraseControlPoint(double stamp_sec);
  bool Remove(double stamp_sec) { return EraseControlPoint(stamp_sec); }

  std::optional<PoseSample> EvaluatePose(double stamp_sec) const;
  std::optional<PoseSample> Interpolate(double stamp_sec) const { return EvaluatePose(stamp_sec); }

  std::optional<Eigen::Vector3d> EvaluatePosition(double stamp_sec) const;
  std::optional<Eigen::Quaterniond> EvaluateOrientation(double stamp_sec) const;
  std::optional<Eigen::Vector3d> EvaluateLinearVelocity(double stamp_sec) const;
  std::optional<Eigen::Vector3d> EvaluateLinearAcceleration(double stamp_sec) const;
  std::optional<Eigen::Vector3d> EvaluateAngularVelocity(double stamp_sec) const;
  std::optional<Eigen::Vector3d> EvaluateAngularAcceleration(double stamp_sec) const;
  std::optional<DerivativeSample> EvaluateDerivatives(double stamp_sec) const;

private:
  using ControlPointList = std::vector<ControlPoint>;

  static double ClampUnitInterval(double u);
  static std::vector<Eigen::Quaterniond> BuildAlignedOrientations(
    const ControlPointList& control_points);
  static std::vector<Eigen::Vector3d> BuildLinearTangents(
    const ControlPointList& control_points);
  static Eigen::Vector3d EstimateLinearTangent(
    const ControlPointList& control_points,
    std::size_t index);
  static Eigen::Quaterniond EstimateSquadTangent(
    const std::vector<Eigen::Quaterniond>& orientations,
    std::size_t index);
  static std::size_t ClampSegmentIndex(
    const ControlPointList& control_points,
    double stamp_sec);
  static double SegmentParameter(
    const ControlPointList& control_points,
    std::size_t segment_index,
    double stamp_sec);
  static PoseSample EvaluateSegmentPose(
    const ControlPointList& control_points,
    const std::vector<Eigen::Quaterniond>& orientations,
    const std::vector<Eigen::Vector3d>& linear_tangents,
    std::size_t segment_index,
    double stamp_sec);
  static Eigen::Vector3d EvaluateSegmentPosition(
    const ControlPointList& control_points,
    const std::vector<Eigen::Vector3d>& linear_tangents,
    std::size_t segment_index,
    double stamp_sec);
  static Eigen::Vector3d EvaluateSegmentLinearVelocity(
    const ControlPointList& control_points,
    const std::vector<Eigen::Vector3d>& linear_tangents,
    std::size_t segment_index,
    double stamp_sec);
  static Eigen::Vector3d EvaluateSegmentLinearAcceleration(
    const ControlPointList& control_points,
    const std::vector<Eigen::Vector3d>& linear_tangents,
    std::size_t segment_index,
    double stamp_sec);
  static Eigen::Quaterniond EvaluateSegmentOrientation(
    const ControlPointList& control_points,
    const std::vector<Eigen::Quaterniond>& orientations,
    std::size_t segment_index,
    double stamp_sec);
  static Eigen::Vector3d FiniteDifferenceAngularVelocity(
    const ControlPointList& control_points,
    double stamp_sec);
  static double AngularDifferenceStep(
    const ControlPointList& control_points,
    double stamp_sec);

  ControlPointList control_points_;
};

inline void TimedPoseSpline::Clear()
{
  control_points_.clear();
}

inline bool TimedPoseSpline::Empty() const
{
  return control_points_.empty();
}

inline std::size_t TimedPoseSpline::Size() const
{
  return control_points_.size();
}

inline const std::vector<TimedPoseSpline::ControlPoint>& TimedPoseSpline::ControlPoints() const
{
  return control_points_;
}

inline double TimedPoseSpline::ClampUnitInterval(double u)
{
  return std::clamp(u, 0.0, 1.0);
}

inline bool TimedPoseSpline::InsertControlPoint(const ControlPoint& control_point)
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

inline bool TimedPoseSpline::EraseControlPoint(double stamp_sec)
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

inline std::vector<Eigen::Quaterniond> TimedPoseSpline::BuildAlignedOrientations(
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

inline std::vector<Eigen::Vector3d> TimedPoseSpline::BuildLinearTangents(
  const ControlPointList& control_points)
{
  std::vector<Eigen::Vector3d> tangents;
  tangents.reserve(control_points.size());
  for (std::size_t index = 0; index < control_points.size(); ++index) {
    tangents.push_back(EstimateLinearTangent(control_points, index));
  }
  return tangents;
}

inline Eigen::Vector3d TimedPoseSpline::EstimateLinearTangent(
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

inline Eigen::Quaterniond TimedPoseSpline::EstimateSquadTangent(
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

inline std::size_t TimedPoseSpline::ClampSegmentIndex(
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

inline double TimedPoseSpline::SegmentParameter(
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

inline TimedPoseSpline::PoseSample TimedPoseSpline::EvaluateSegmentPose(
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

inline Eigen::Vector3d TimedPoseSpline::EvaluateSegmentPosition(
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

inline Eigen::Vector3d TimedPoseSpline::EvaluateSegmentLinearVelocity(
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

inline Eigen::Vector3d TimedPoseSpline::EvaluateSegmentLinearAcceleration(
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

inline Eigen::Quaterniond TimedPoseSpline::EvaluateSegmentOrientation(
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

inline std::optional<TimedPoseSpline::PoseSample> TimedPoseSpline::EvaluatePose(double stamp_sec) const
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

inline std::optional<Eigen::Vector3d> TimedPoseSpline::EvaluatePosition(double stamp_sec) const
{
  const auto pose = EvaluatePose(stamp_sec);
  if (!pose.has_value()) {
    return std::nullopt;
  }
  return pose->position;
}

inline std::optional<Eigen::Quaterniond> TimedPoseSpline::EvaluateOrientation(double stamp_sec) const
{
  const auto pose = EvaluatePose(stamp_sec);
  if (!pose.has_value()) {
    return std::nullopt;
  }
  return pose->orientation;
}

inline std::optional<Eigen::Vector3d> TimedPoseSpline::EvaluateLinearVelocity(double stamp_sec) const
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

inline std::optional<Eigen::Vector3d> TimedPoseSpline::EvaluateLinearAcceleration(double stamp_sec) const
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

inline double TimedPoseSpline::AngularDifferenceStep(
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

inline Eigen::Vector3d TimedPoseSpline::FiniteDifferenceAngularVelocity(
  const ControlPointList& control_points,
  double stamp_sec)
{
  if (control_points.empty()) {
    return Eigen::Vector3d::Zero();
  }
  if (control_points.size() == 1) {
    return Eigen::Vector3d::Zero();
  }

  TimedPoseSpline spline;
  spline.control_points_ = control_points;
  const double step = AngularDifferenceStep(control_points, stamp_sec);
  const auto orientation_minus = spline.EvaluateOrientation(stamp_sec - step);
  const auto orientation_plus = spline.EvaluateOrientation(stamp_sec + step);
  if (!orientation_minus.has_value() || !orientation_plus.has_value()) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Quaterniond aligned_plus = *orientation_plus;
  if (orientation_minus->coeffs().dot(aligned_plus.coeffs()) < 0.0) {
    aligned_plus.coeffs() *= -1.0;
  }

  const Eigen::Quaterniond delta = orientation_minus->conjugate() * aligned_plus;
  return detail::QuaternionLog(delta) / (2.0 * step);
}

inline std::optional<Eigen::Vector3d> TimedPoseSpline::EvaluateAngularVelocity(double stamp_sec) const
{
  if (control_points_.empty()) {
    return std::nullopt;
  }
  if (control_points_.size() == 1) {
    return Eigen::Vector3d::Zero();
  }

  return FiniteDifferenceAngularVelocity(control_points_, stamp_sec);
}

inline std::optional<Eigen::Vector3d> TimedPoseSpline::EvaluateAngularAcceleration(double stamp_sec) const
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

inline std::optional<TimedPoseSpline::DerivativeSample> TimedPoseSpline::EvaluateDerivatives(
  double stamp_sec) const
{
  const auto position = EvaluateLinearVelocity(stamp_sec);
  const auto acceleration = EvaluateLinearAcceleration(stamp_sec);
  const auto angular_velocity = EvaluateAngularVelocity(stamp_sec);
  const auto angular_acceleration = EvaluateAngularAcceleration(stamp_sec);
  if (!position.has_value() || !acceleration.has_value() ||
      !angular_velocity.has_value() || !angular_acceleration.has_value()) {
    return std::nullopt;
  }

  DerivativeSample sample;
  sample.linear_velocity = *position;
  sample.linear_acceleration = *acceleration;
  sample.angular_velocity = *angular_velocity;
  sample.angular_acceleration = *angular_acceleration;
  return sample;
}

}  // namespace anima::pallas
