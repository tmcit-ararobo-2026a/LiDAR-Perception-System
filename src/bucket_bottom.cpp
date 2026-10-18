// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <random>

#include "lidar_perception_system/opening_tracker.hpp"

namespace lidar_perception_system::opening {
namespace {
constexpr double pi = 3.14159265358979323846;
using XY = Eigen::Vector2d;
double cross(const XY& a, const XY& b) { return a.x() * b.y() - a.y() * b.x(); }
std::vector<XY> hull(std::vector<XY> p) {
  std::sort(p.begin(), p.end(), [](const XY& a, const XY& b) {
    return a.x() == b.x() ? a.y() < b.y() : a.x() < b.x();
  });
  std::vector<XY> out;
  for (const auto& q : p) {
    while (out.size() > 1 && cross(out.back() - out[out.size() - 2], q - out.back()) <= 0)
      out.pop_back();
    out.push_back(q);
  }
  const auto lower = out.size();
  for (int i = static_cast<int>(p.size()) - 2; i >= 0; --i) {
    while (out.size() > lower && cross(out.back() - out[out.size() - 2], p[i] - out.back()) <= 0)
      out.pop_back();
    out.push_back(p[i]);
  }
  if (!out.empty()) out.pop_back();
  return out;
}
double span(std::vector<double> a) {
  if (a.size() < 2) return 0;
  std::sort(a.begin(), a.end());
  double gap = a.front() + 2 * pi - a.back();
  for (size_t i = 1; i < a.size(); ++i) gap = std::max(gap, a[i] - a[i - 1]);
  return 2 * pi - gap;
}
// Cluster the entire slice before fitting: never carve an arbitrary disk out of
// a large robot deck. Spatial hashing bounds neighbor searches on dense scans.
std::vector<Points> patches(const Points& points, double cell) {
  using Key = std::pair<int, int>;
  std::map<Key, std::vector<size_t>> bins;
  for (size_t i = 0; i < points.size(); ++i)
    bins[{static_cast<int>(std::floor(points[i].x() / cell)),
          static_cast<int>(std::floor(points[i].y() / cell))}]
        .push_back(i);
  std::vector<bool> seen(points.size());
  std::vector<Points> out;
  for (size_t i = 0; i < points.size(); ++i) {
    if (seen[i]) continue;
    std::vector<size_t> pending{i};
    seen[i] = true;
    Points group;
    while (!pending.empty()) {
      const auto n = pending.back();
      pending.pop_back();
      group.push_back(points[n]);
      const int x = static_cast<int>(std::floor(points[n].x() / cell));
      const int y = static_cast<int>(std::floor(points[n].y() / cell));
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy) {
          auto it = bins.find({x + dx, y + dy});
          if (it == bins.end()) continue;
          for (const auto k : it->second) {
            if (!seen[k] && (points[k].head<2>() - points[n].head<2>()).norm() <= cell) {
              seen[k] = true;
              pending.push_back(k);
            }
          }
        }
    }
    out.push_back(std::move(group));
  }
  return out;
}
std::optional<Bottom> fit_patch(const Points& p, const Points& all, const Points& current,
                                const Config& cfg) {
  const auto& c = cfg.bottom;
  if (static_cast<int>(p.size()) < c.min_plane_points) return {};
  Point mean = Point::Zero();
  for (const auto& v : p) mean += v;
  mean /= p.size();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& v : p) covariance += (v - mean) * (v - mean).transpose();
  covariance /= p.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
  if (eig.info() != Eigen::Success || eig.eigenvalues()[1] < 0.00035) return {};
  Point normal = eig.eigenvectors().col(0);
  if (normal.z() < 0) normal = -normal;
  const double rmse = std::sqrt(std::max(0., eig.eigenvalues()[0]));
  if (normal.z() < std::cos(c.max_tilt_rad) || rmse > c.max_plane_rmse) return {};
  std::vector<XY> xy;
  XY lo = p.front().head<2>(), hi = lo;
  for (const auto& v : p) {
    xy.push_back(v.head<2>());
    lo = lo.cwiseMin(v.head<2>());
    hi = hi.cwiseMax(v.head<2>());
  }
  if ((hi - lo).maxCoeff() > 2 * c.max_radius + c.plane_distance * 2 ||
      (hi - lo).minCoeff() < c.min_radius)
    return {};
  auto edge = hull(xy);
  if (edge.size() < 5) return {};
  double area = 0;
  for (size_t i = 0; i < edge.size(); ++i) area += cross(edge[i], edge[(i + 1) % edge.size()]);
  area = std::abs(area) * .5;
  std::mt19937 rng(0x20261018);
  std::optional<Bottom> best;
  for (int iteration = 0; iteration < cfg.ransac_iterations; ++iteration) {
    const XY& a = edge[rng() % edge.size()];
    const XY& b = edge[rng() % edge.size()];
    const XY& d = edge[rng() % edge.size()];
    Eigen::Matrix2d m;
    m.row(0) = 2 * (b - a);
    m.row(1) = 2 * (d - a);
    if (std::abs(m.determinant()) < 1e-6) continue;
    XY center =
        m.inverse() * XY(b.squaredNorm() - a.squaredNorm(), d.squaredNorm() - a.squaredNorm());
    const double radius = (center - a).norm();
    if (radius < c.min_radius || radius > c.max_radius ||
        (center - mean.head<2>()).norm() > radius * .7 || area / (pi * radius * radius) < .35)
      continue;
    std::vector<double> angles;
    double error = 0;
    int boundary = 0, interior = 0, outside = 0;
    for (const auto& v : edge) {
      const XY delta = v - center;
      const double e = std::abs(delta.norm() - radius);
      if (e <= c.plane_distance) {
        ++boundary;
        error += e * e;
        angles.push_back(std::atan2(delta.y(), delta.x()));
      }
    }
    if (boundary < 5 || boundary < static_cast<int>(edge.size() * .65) || span(angles) < 2.1)
      continue;
    for (const auto& v : p) {
      const double r = (v.head<2>() - center).norm();
      interior += r < radius * .65;
      outside += r > radius + c.plane_distance;
    }
    if (interior < 4 || outside > static_cast<int>(p.size() * .1)) continue;
    const double z = mean.z() - (normal.head<2>().dot(center - mean.head<2>())) / normal.z();
    if (z + c.bucket_height < cfg.min_opening_z - c.height_tolerance ||
        z + c.bucket_height > cfg.max_opening_z + c.height_tolerance)
      continue;
    // Side returns must extend upward at a compatible radius over several
    // heights. A bare disk, narrow mast, or a solid horizontal deck is
    // insufficient.
    int side = 0, solid = 0, continuing = 0;
    double low = 1e9, high = -1e9;
    std::array<bool, 5> levels{};
    for (const auto& v : all) {
      const double dz = v.z() - z;
      const double radial = (v.head<2>() - center).norm();
      if (dz > c.bucket_height + c.height_tolerance && dz < c.bucket_height + .20 &&
          radial > radius * .65 && radial < cfg.nominal_radius + c.side_tolerance)
        ++continuing;
      if (dz < .04 || dz > c.bucket_height + c.height_tolerance) continue;
      const double expected =
          radius + (cfg.nominal_radius - radius) * std::min(1., dz / c.bucket_height);
      if (std::abs(radial - expected) <= c.side_tolerance) {
        ++side;
        low = std::min(low, dz);
        high = std::max(high, dz);
        levels[std::clamp(static_cast<int>(dz / c.bucket_height * 5), 0, 4)] = true;
      }
      solid += dz > .08 && radial < radius * .45;
    }
    if (side < c.min_side_points || high - low < c.min_side_span || high < c.bucket_height - .08 ||
        std::count(levels.begin(), levels.end(), true) < 2 || solid >= 4 || continuing >= 4)
      continue;
    int fresh = 0;
    for (const auto& v : current) {
      if (std::abs(normal.dot(v - Point(center.x(), center.y(), z))) <= c.plane_distance &&
          (v.head<2>() - center).norm() <= radius + c.plane_distance)
        ++fresh;
    }
    if (fresh < c.min_current_points) continue;
    Bottom result;
    result.center = {center.x(), center.y(), z};
    result.normal = normal;
    result.radius = radius;
    result.plane_rmse = rmse;
    result.boundary_residual = std::sqrt(error / boundary);
    result.plane_points = p.size() - outside;
    result.current_points = fresh;
    result.side_points = side;
    result.score = std::log1p(result.plane_points) + std::min(1., area / (pi * radius * radius)) +
                   span(angles) / (2 * pi) - rmse / c.max_plane_rmse -
                   result.boundary_residual / c.plane_distance;
    if (!best || result.score > best->score) best = result;
  }
  return best;
}
}  // namespace

std::optional<Bottom> fit_bucket_bottom(const Points& accumulated, const Points& current,
                                        const Config& cfg) {
  cfg.validate();
  const auto& c = cfg.bottom;
  // Input can be called directly by offline users as well as through Tracker.
  Points finite, fresh;
  for (const auto& p : accumulated)
    if (p.allFinite() && p.cwiseAbs().maxCoeff() <= 200) finite.push_back(p);
  for (const auto& p : current)
    if (p.allFinite() && p.cwiseAbs().maxCoeff() <= 200) fresh.push_back(p);
  if (finite.size() > static_cast<size_t>(cfg.max_model_points) || fresh.empty()) return {};
  std::vector<Bottom> candidates;
  const double lower = cfg.min_opening_z - c.bucket_height - c.height_tolerance;
  const double upper = cfg.max_opening_z - c.bucket_height + c.height_tolerance;
  for (double z = lower; z <= upper; z += c.plane_distance) {
    Points slice;
    for (const auto& p : finite)
      if (std::abs(p.z() - z) <= c.plane_distance) slice.push_back(p);
    if (slice.size() < static_cast<size_t>(c.min_plane_points)) continue;
    for (const auto& patch : patches(slice, c.connection_m)) {
      auto fit = fit_patch(patch, finite, fresh, cfg);
      if (!fit) continue;
      bool merged = false;
      for (auto& old : candidates) {
        if ((old.center.head<2>() - fit->center.head<2>()).norm() < .08 &&
            std::abs(old.center.z() - fit->center.z()) < .04) {
          if (fit->score > old.score) old = *fit;
          merged = true;
          break;
        }
      }
      if (!merged) candidates.push_back(*fit);
    }
  }
  if (candidates.empty()) return {};
  std::sort(candidates.begin(), candidates.end(),
            [](const Bottom& a, const Bottom& b) { return a.score > b.score; });
  if (candidates.size() > 1 && candidates[1].score > candidates[0].score - .7) return {};
  return candidates.front();
}
std::optional<Bottom> track_bucket_bottom(const Bottom& learned, const Points& current,
                                          const Config& cfg) {
  if (!learned.center.allFinite() || !learned.normal.allFinite() || learned.normal.norm() < .99 ||
      learned.radius < cfg.bottom.min_radius || learned.radius > cfg.bottom.max_radius ||
      current.size() > static_cast<size_t>(cfg.max_model_points))
    return {};
  Points support;
  for (const auto& p : current) {
    if (p.allFinite() && (p.head<2>() - learned.center.head<2>()).norm() < learned.radius &&
        std::abs(learned.normal.dot(p - learned.center)) <= cfg.height_change_m)
      support.push_back(p);
  }
  if (support.size() < static_cast<size_t>(cfg.bottom.min_current_points)) return {};
  Point mean = Point::Zero();
  for (const auto& p : support) mean += p;
  mean /= support.size();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& p : support) covariance += (p - mean) * (p - mean).transpose();
  covariance /= support.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
  if (eig.info() != Eigen::Success || eig.eigenvalues()[1] < .0001) return {};
  Point normal = eig.eigenvectors().col(0);
  if (normal.z() < 0) normal = -normal;
  const double rmse = std::sqrt(std::max(0., eig.eigenvalues()[0]));
  if (normal.z() < std::cos(cfg.bottom.max_tilt_rad) ||
      normal.dot(learned.normal) < std::cos(cfg.bottom.max_tilt_rad) ||
      rmse > cfg.bottom.max_plane_rmse)
    return {};
  Bottom tracked = learned;
  // Partial returns cannot identify a new disk center/radius. Correct only the
  // measured plane height/normal, keeping the previously recognized local XY.
  tracked.center.z() =
      mean.z() - normal.head<2>().dot(learned.center.head<2>() - mean.head<2>()) / normal.z();
  if (std::abs(tracked.center.z() - learned.center.z()) > cfg.height_change_m) return {};
  tracked.normal = normal;
  tracked.plane_rmse = rmse;
  tracked.plane_points = tracked.current_points = static_cast<int>(support.size());
  tracked.side_points = 0;
  return tracked;
}
}  // namespace lidar_perception_system::opening
