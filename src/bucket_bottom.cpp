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
namespace {
struct ShapeAxis {
  Point origin, normal, x, y;
  Point local(const Point& p) const {
    const Point d = p - origin;
    return {d.dot(x), d.dot(y), d.dot(normal)};
  }
  Point world(const XY& p) const { return origin + x * p.x() + y * p.y(); }
};
ShapeAxis shape_axis(const Point& origin, const Point& normal) {
  const Point x = (Point::UnitX() - normal * normal.x()).normalized();
  return {origin, normal, x, normal.cross(x)};
}
double shape_radius(double bottom_radius, double z, const Config& cfg) {
  return bottom_radius + (cfg.nominal_radius - bottom_radius) *
                             std::clamp(z / cfg.bottom.bucket_height, 0., 1.);
}
double shape_tolerance(const Config& cfg) {
  return std::min(cfg.bottom.plane_distance, cfg.bottom.side_tolerance * .5);
}
struct ShapeEvidence {
  int sides{0}, plane{0}, solid{0}, continuing{0}, shell{0};
  double squared{0}, low{1e9}, high{-1e9}, arc{0};
};
ShapeEvidence shape_evidence(const Points &points, const XY &center,
                             double radius, const Config &cfg) {
  ShapeEvidence e;
  std::vector<double> angles;
  for (const auto &p : points) {
    const XY d = p.head<2>() - center;
    const double r = d.norm(), z = p.z();
    if (std::abs(z) <= cfg.bottom.plane_distance &&
        r <= radius + cfg.bottom.plane_distance)
      ++e.plane;
    if (z > .08 && z < cfg.bottom.bucket_height + cfg.bottom.height_tolerance &&
        r < shape_radius(radius, z, cfg) * .75)
      ++e.solid;
    if (z > cfg.bottom.bucket_height + cfg.bottom.height_tolerance &&
        z < cfg.bottom.bucket_height + .20 && r > radius * .6 &&
        r < cfg.nominal_radius + cfg.bottom.side_tolerance)
      ++e.continuing;
    if (z < .035 || z > cfg.bottom.bucket_height + cfg.bottom.height_tolerance)
      continue;
    if (r < cfg.nominal_radius + cfg.bottom.side_tolerance)
      ++e.shell;
    const double error = r - shape_radius(radius, z, cfg);
    if (std::abs(error) > shape_tolerance(cfg))
      continue;
    ++e.sides;
    e.squared += error * error;
    e.low = std::min(e.low, z);
    e.high = std::max(e.high, z);
    angles.push_back(std::atan2(d.y(), d.x()));
  }
  e.arc = span(std::move(angles));
  return e;
}
std::vector<XY> shape_centers(const Point &a, const Point &b, double radius,
                              const Config &cfg) {
  const XY d = b.head<2>() - a.head<2>();
  const double distance = d.norm();
  if (distance < .06)
    return {};
  const double r1 = shape_radius(radius, a.z(), cfg),
               r2 = shape_radius(radius, b.z(), cfg);
  const double along =
      (r1 * r1 - r2 * r2 + distance * distance) / (2 * distance);
  const double height2 = r1 * r1 - along * along;
  if (height2 < 0)
    return {};
  const XY u = d / distance, v(-u.y(), u.x());
  const XY midpoint = a.head<2>() + u * along;
  return {midpoint + v * std::sqrt(height2), midpoint - v * std::sqrt(height2)};
}
bool refine_shape(const Points &side, XY &center, double &radius,
                  const Config &cfg, bool fit_radius) {
  for (int iteration = 0; iteration < 8; ++iteration) {
    Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
    Point g = Point::Zero();
    int used = 0;
    for (const auto &p : side) {
      const XY d = center - p.head<2>();
      const double norm = d.norm();
      if (norm < 1e-6)
        continue;
      const double error = norm - shape_radius(radius, p.z(), cfg);
      if (std::abs(error) > cfg.bottom.side_tolerance)
        continue;
      const Point j(
          d.x() / norm, d.y() / norm,
          fit_radius
              ? -(1 - std::clamp(p.z() / cfg.bottom.bucket_height, 0., 1.))
              : 0.);
      const double weight =
          std::min(1., shape_tolerance(cfg) / std::max(1e-9, std::abs(error)));
      h.noalias() += weight * j * j.transpose();
      g.noalias() += weight * j * error;
      ++used;
    }
    if (used < (fit_radius ? 4 : 3))
      return false;
    if (!fit_radius)
      h(2, 2) = 1;
    if (std::abs(h.determinant()) < 1e-8)
      return false;
    Point step = -h.ldlt().solve(g);
    if (!step.allFinite())
      return false;
    step /= std::max(1., step.norm() / .025);
    center += step.head<2>();
    if (fit_radius)
      radius = std::clamp(radius + step.z(), cfg.bottom.min_radius,
                          cfg.bottom.max_radius);
    if (step.norm() < 1e-5)
      break;
  }
  return true;
}
std::optional<Bottom> fit_sparse_patch(const Points &plane, const Points &all,
                                       const Points &current,
                                       const Config &cfg) {
  const auto &c = cfg.bottom;
  if (plane.size() < static_cast<size_t>(std::max(6, c.min_plane_points / 3)))
    return {};
  Point mean = Point::Zero();
  for (const auto &p : plane)
    mean += p;
  mean /= plane.size();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto &p : plane)
    covariance += (p - mean) * (p - mean).transpose();
  covariance /= plane.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
  if (eig.info() != Eigen::Success || eig.eigenvalues()[1] < .0002)
    return {};
  Point normal = eig.eigenvectors().col(0);
  if (normal.z() < 0)
    normal = -normal;
  const double plane_rmse = std::sqrt(std::max(0., eig.eigenvalues()[0]));
  if (normal.z() < std::cos(c.max_tilt_rad) || plane_rmse > c.max_plane_rmse)
    return {};
  const auto axis = shape_axis(mean, normal);
  Points floor, context, fresh, side;
  XY lo = XY::Constant(1e9), hi = -lo;
  // A quantized height slice can leave a few noisy disk points while excluding
  // a coplanar large deck. Grow the entire component on the fitted plane
  // before evaluating its footprint; never carve a disk from that deck.
  Points coplanar, component;
  for (const auto &p : all)
    if (std::abs(normal.dot(p - mean)) <= c.plane_distance)
      coplanar.push_back(p);
  for (auto &group :
       patches(coplanar, std::max(c.connection_m, c.min_radius))) {
    if (std::any_of(group.begin(), group.end(), [&](const Point &p) {
          return (p - plane.front()).squaredNorm() < 1e-12;
        })) {
      component = std::move(group);
      break;
    }
  }
  if (component.empty())
    return {};
  double plane_squared = 0;
  for (const auto &p : component) {
    const auto q = axis.local(p);
    floor.push_back(q);
    lo = lo.cwiseMin(q.head<2>());
    hi = hi.cwiseMax(q.head<2>());
    plane_squared += q.z() * q.z();
  }
  const double supported_plane_rmse =
      std::sqrt(plane_squared / component.size());
  if (supported_plane_rmse > c.max_plane_rmse)
    return {};
  if ((hi - lo).maxCoeff() > 2 * c.max_radius + c.plane_distance * 2 ||
      (hi - lo).minCoeff() < c.min_radius * .7)
    return {};
  for (const auto &p : all) {
    const auto q = axis.local(p);
    if (q.head<2>().norm() > 2 * c.max_radius + .05 || q.z() < -.03 ||
        q.z() > c.bucket_height + .20)
      continue;
    context.push_back(q);
    if (q.z() >= .035 && q.z() <= c.bucket_height + c.height_tolerance)
      side.push_back(q);
  }
  if (side.size() < static_cast<size_t>(std::max(10, c.min_side_points)))
    return {};
  for (const auto &p : current)
    fresh.push_back(axis.local(p));
  std::optional<Bottom> best;
  std::mt19937 rng(0x20261026);
  for (int ri = 0; ri <= 6; ++ri) {
    const double seed_radius =
        c.min_radius + (c.max_radius - c.min_radius) * ri / 6.;
    for (int k = 0; k < std::min(cfg.ransac_iterations, 40); ++k) {
      const auto a = side[rng() % side.size()], b = side[rng() % side.size()];
      for (auto center : shape_centers(a, b, seed_radius, cfg)) {
        if (center.norm() > c.max_radius)
          continue;
        double radius = seed_radius;
        if (!refine_shape(side, center, radius, cfg, true) ||
            center.norm() > radius * .9)
          continue;
        int interior = 0, outside = 0;
        for (const auto &p : floor) {
          const double r = (p.head<2>() - center).norm();
          interior += r < radius * .7;
          outside += r > radius + c.plane_distance;
        }
        if (interior < 3 || outside > static_cast<int>(floor.size() * .1))
          continue;
        const auto evidence = shape_evidence(context, center, radius, cfg);
        const auto now = shape_evidence(fresh, center, radius, cfg);
        if (evidence.sides < std::max(10, c.min_side_points) ||
            evidence.sides < .65 * evidence.shell ||
            evidence.arc < cfg.min_arc ||
            evidence.high - evidence.low <
                std::max(c.min_side_span, c.bucket_height * .65) ||
            evidence.high < c.bucket_height - .08 || evidence.solid >= 4 ||
            evidence.continuing >= 4 ||
            now.plane + now.sides < c.min_current_points || now.sides < 2 ||
            now.sides < .6 * now.shell || now.arc < .6 || now.solid >= 4 ||
            now.continuing >= 4)
          continue;
        const double residual = std::sqrt(evidence.squared / evidence.sides);
        if (residual > shape_tolerance(cfg))
          continue;
        Bottom result;
        result.center = axis.world(center);
        if (result.center.z() + c.bucket_height <
                cfg.min_opening_z - c.height_tolerance ||
            result.center.z() + c.bucket_height >
                cfg.max_opening_z + c.height_tolerance)
          continue;
        result.normal = normal;
        result.radius = radius;
        result.plane_rmse = std::max(plane_rmse, supported_plane_rmse);
        result.boundary_residual = residual;
        result.plane_points = floor.size() - outside;
        result.current_points = now.plane + now.sides;
        result.side_points = evidence.sides;
        result.score = std::log1p(result.plane_points) +
                       std::log1p(evidence.sides) + evidence.arc / pi -
                       residual / shape_tolerance(cfg) -
                       result.plane_rmse / c.max_plane_rmse;
        if (!best || result.score > best->score)
          best = result;
      }
    }
  }
  return best;
}
} // namespace

std::optional<Bottom> fit_bucket_shape(const Points &accumulated,
                                       const Points &current,
                                       const Config &cfg) {
  cfg.validate();
  if (!cfg.bottom.sparse_shape_enabled || current.empty() ||
      current.size() > static_cast<size_t>(cfg.max_model_points) ||
      accumulated.size() > static_cast<size_t>(cfg.max_model_points))
    return {};
  Points finite, fresh;
  for (const auto &p : accumulated)
    if (p.allFinite() && p.cwiseAbs().maxCoeff() <= 200)
      finite.push_back(p);
  for (const auto &p : current)
    if (p.allFinite() && p.cwiseAbs().maxCoeff() <= 200)
      fresh.push_back(p);
  std::vector<Bottom> candidates;
  const double lower = cfg.min_opening_z - cfg.bottom.bucket_height -
                       cfg.bottom.height_tolerance;
  const double upper = cfg.max_opening_z - cfg.bottom.bucket_height +
                       cfg.bottom.height_tolerance;
  for (double z = lower; z <= upper; z += cfg.bottom.plane_distance) {
    Points slice;
    for (const auto &p : finite)
      if (std::abs(p.z() - z) <= cfg.bottom.plane_distance)
        slice.push_back(p);
    for (const auto &patch : patches(
             slice, std::max(cfg.bottom.connection_m, cfg.bottom.min_radius))) {
      auto fit = fit_sparse_patch(patch, finite, fresh, cfg);
      if (!fit)
        continue;
      bool merged = false;
      for (auto &old : candidates) {
        if ((old.center - fit->center).norm() < .05) {
          if (fit->score > old.score)
            old = *fit;
          merged = true;
          break;
        }
      }
      if (!merged)
        candidates.push_back(*fit);
    }
  }
  if (candidates.empty())
    return {};
  std::sort(candidates.begin(), candidates.end(),
            [](const auto &a, const auto &b) { return a.score > b.score; });
  if (candidates.size() > 1 && candidates[1].score > candidates[0].score - .7)
    return {};
  return candidates.front();
}
std::optional<Bottom> track_bucket_shape(const Bottom &learned,
                                         const Points &current,
                                         const Config &cfg) {
  cfg.validate();
  if (!cfg.bottom.sparse_shape_enabled || !learned.center.allFinite() ||
      !learned.normal.allFinite() ||
      std::abs(learned.normal.norm() - 1.) > .01 ||
      learned.normal.z() < std::cos(cfg.bottom.max_tilt_rad) ||
      learned.radius < cfg.bottom.min_radius ||
      learned.radius > cfg.bottom.max_radius ||
      current.size() > static_cast<size_t>(cfg.max_model_points))
    return {};
  const auto axis = shape_axis(learned.center, learned.normal);
  const double correction_limit =
      std::min({.30, cfg.crop_margin, cfg.association_m});
  Points context, side;
  for (const auto &p : current) {
    if (!p.allFinite())
      continue;
    const auto q = axis.local(p);
    if (q.head<2>().norm() >
        cfg.nominal_radius + cfg.bottom.side_tolerance + correction_limit)
      continue;
    context.push_back(q);
    if (q.z() > .035 &&
        q.z() < cfg.bottom.bucket_height + cfg.bottom.height_tolerance)
      side.push_back(q);
  }
  if (side.size() < static_cast<size_t>(cfg.bottom.shape_tracking_min_points))
    return {};
  std::vector<XY> seeds{XY::Zero()};
  std::mt19937 rng(0x20261027);
  for (int i = 0; i < 32; ++i) {
    const auto &a = side[rng() % side.size()];
    const auto &b = side[rng() % side.size()];
    for (const auto &seed : shape_centers(a, b, learned.radius, cfg))
      if (seed.norm() <= correction_limit)
        seeds.push_back(seed);
  }
  std::optional<Bottom> best;
  double best_quality = std::numeric_limits<double>::infinity();
  bool ambiguous = false;
  for (auto center : seeds) {
    double radius = learned.radius;
    if (!refine_shape(side, center, radius, cfg, false) ||
        center.norm() > correction_limit)
      continue;
    const auto now = shape_evidence(context, center, radius, cfg);
    if (now.sides < cfg.bottom.shape_tracking_min_points ||
        now.sides < .65 * now.shell || now.arc < 1.05 ||
        now.high - now.low < cfg.bottom.min_side_span || now.solid >= 4 ||
        now.continuing >= 4)
      continue;
    const double residual = std::sqrt(now.squared / now.sides);
    // Three points can update an established shape only with tighter residuals.
    // A large correction must be supported by a broad current frustum, not a
    // few points from an adjacent fixture.
    if (now.sides < 4 && residual > shape_tolerance(cfg) * .5)
      continue;
    if (center.norm() > .10 &&
        (now.sides < std::max(6, cfg.bottom.min_current_points) ||
         now.arc < cfg.min_arc ||
         now.high - now.low < std::max(cfg.bottom.min_side_span,
                                       cfg.bottom.bucket_height * .65)))
      continue;
    const double quality =
        residual + shape_tolerance(cfg) * (1. - static_cast<double>(now.sides) /
                                                    std::max(1, now.shell));
    const Point fitted_center = axis.world(center);
    const bool different = best && (best->center - fitted_center).norm() > .04;
    if (different && std::abs(quality - best_quality) < .002)
      ambiguous = true;
    if (quality < best_quality) {
      if (quality + .002 < best_quality)
        ambiguous = false;
      Bottom result = learned;
      result.center = fitted_center;
      result.current_points = result.side_points = now.sides;
      result.plane_points = now.plane;
      result.boundary_residual = residual;
      best = result;
      best_quality = quality;
    }
  }
  if (ambiguous)
    return {};
  return best;
}

} // namespace lidar_perception_system::opening
