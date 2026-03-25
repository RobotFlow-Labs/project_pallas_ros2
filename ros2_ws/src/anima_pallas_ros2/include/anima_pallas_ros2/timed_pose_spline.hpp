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

}  // namespace anima::pallas
