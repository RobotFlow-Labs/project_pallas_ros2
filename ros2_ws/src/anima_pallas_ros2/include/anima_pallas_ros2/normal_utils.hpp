#pragma once

#include <Eigen/Core>

namespace anima::pallas {

/// Return a fallback normal pointing toward the origin if the point is
/// far enough away, otherwise return +Z.
inline Eigen::Vector3d FallbackNormal(const Eigen::Vector3d& point)
{
  if (point.norm() > 1e-9) {
    return -point.normalized();
  }
  return Eigen::Vector3d::UnitZ();
}

/// Normalize a surface normal.  Falls back to FallbackNormal when the
/// input is degenerate, and optionally flips toward the origin.
inline Eigen::Vector3d NormalizeNormal(
  const Eigen::Vector3d& normal,
  const Eigen::Vector3d& point_hint)
{
  Eigen::Vector3d normalized = normal;
  if (!normalized.allFinite() || normalized.norm() < 1e-9) {
    normalized = FallbackNormal(point_hint);
  } else {
    normalized.normalize();
  }

  if (point_hint.norm() > 1e-9 && normalized.dot(point_hint) > 0.0) {
    normalized = -normalized;
  }
  return normalized;
}

}  // namespace anima::pallas
