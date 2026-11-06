// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
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
  if (c.horizontal_only) normal = Point::UnitZ();
  const double rmse = std::sqrt(std::max(0., c.horizontal_only ? covariance(2, 2)
                                                                         : eig.eigenvalues()[0]));
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
        std::abs((cfg.bottom.horizontal_only ? p.z() - learned.center.z()
                                              : learned.normal.dot(p - learned.center))) <= cfg.bottom.plane_distance)
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
  if (cfg.bottom.horizontal_only) normal = Point::UnitZ();
  const double rmse = std::sqrt(std::max(0., cfg.bottom.horizontal_only ? covariance(2, 2)
                                                                                  : eig.eigenvalues()[0]));
  if (normal.z() < std::cos(cfg.bottom.max_tilt_rad) ||
      normal.dot(learned.normal) < std::cos(cfg.bottom.max_tilt_rad) ||
      rmse > cfg.bottom.max_plane_rmse)
    return {};
  Bottom tracked = learned;
  // Partial returns cannot identify a new disk center/radius. Correct only the
  // measured plane height/normal, keeping the previously recognized local XY.
  tracked.center.z() =
      mean.z() - normal.head<2>().dot(learned.center.head<2>() - mean.head<2>()) / normal.z();
  // A few nearby returns cannot establish a lift: a side wall or bracket can
  // form a plausible small plane within height_change_m. Only update the
  // learned floor within its plane tolerance; larger lifts need full geometry.
  if (std::abs(tracked.center.z() - learned.center.z()) > cfg.bottom.plane_distance) return {};
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
  int sides{0}, plane{0}, solid{0}, cap{0}, continuing{0}, shell{0};
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
    if (z > cfg.bottom.bucket_height - .035 &&
        z < cfg.bottom.bucket_height + cfg.bottom.height_tolerance &&
        r < shape_radius(radius, z, cfg) * .75)
      ++e.cap;
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
  if (c.horizontal_only) normal = Point::UnitZ();
  const double plane_rmse = std::sqrt(std::max(0., c.horizontal_only ? covariance(2, 2)
                                                                               : eig.eigenvalues()[0]));
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
  // Refinement cost grows with the wall returns; a bounded sample keeps its
  // accuracy. The final evidence below still counts every accumulated point.
  constexpr size_t max_refine_points = 150;
  if (side.size() > max_refine_points) {
    Points sampled;
    sampled.reserve(max_refine_points);
    for (size_t i = 0; i < max_refine_points; ++i)
      sampled.push_back(side[i * side.size() / max_refine_points]);
    side = std::move(sampled);
  }
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
            center.norm() > radius * 1.5)
          continue;
        int interior = 0, outside = 0;
        for (const auto &p : floor) {
          const double r = (p.head<2>() - center).norm();
          interior += r < radius * .7;
          outside += r > radius + c.plane_distance;
        }
        // A small coplanar mounting plate can extend beyond the disk. The
        // whole-component envelope above still rejects large decks; require
        // most of this component to support the disk and validate its sides.
        if (interior < 3 || outside > static_cast<int>(floor.size() * .5))
          continue;
        const auto evidence = shape_evidence(context, center, radius, cfg);
        const auto now = shape_evidence(fresh, center, radius, cfg);
        if (evidence.sides < std::max(10, c.min_side_points) ||
            evidence.sides < .5 * evidence.shell ||
            evidence.arc < cfg.min_arc ||
            evidence.high - evidence.low <
                std::max(c.min_side_span, c.bucket_height * .65) ||
            // Registration noise and internal returns grow with accumulation
            // density. Reject solid objects by their support ratio as well as
            // a minimum count, instead of a fixed four-point limit.
            evidence.high < c.bucket_height - .08 ||
            evidence.solid >= std::max(4., .15 * evidence.sides) ||
            evidence.cap >= 4 ||
            evidence.continuing >= 4 ||
            now.plane + now.sides < c.min_current_points || now.sides < 2 ||
            // One scan sees a narrow arc; the accumulated shell above must
            // still span min_arc and the full vertical extent.
            now.sides < .3 * now.shell || now.arc < .3 || now.solid >= 4 ||
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
                                       const Config &cfg,
                                       std::optional<double> expected_z) {
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
  double lower = cfg.min_opening_z - cfg.bottom.bucket_height -
                 cfg.bottom.height_tolerance;
  double upper = cfg.max_opening_z - cfg.bottom.bucket_height +
                 cfg.bottom.height_tolerance;
  if (expected_z) {
    lower = std::max(lower, *expected_z - cfg.height_change_m);
    upper = std::min(upper, *expected_z + cfg.height_change_m);
  }
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
  Point normal = learned.normal;
  if (cfg.bottom.horizontal_only) normal = Point::UnitZ();
  const auto axis = shape_axis(learned.center, normal);
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
      result.normal = normal;
      result.center = fitted_center;
      // Side-only registration observes XY, not a new floor height. Moving
      // along the learned (noisy/tilted) plane would turn XY corrections into
      // artificial vertical motion and accumulate height drift.
      result.center.z() = learned.center.z();
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

namespace {
// Largest standard deviation of the estimated center (metres).
constexpr double top_max_uncertainty_m = .04;
// Share of the band allowed outside the wall, and share of its height the returns must cover.
constexpr double top_max_outside_ratio = .2, top_min_span_ratio = .5;
// Smallest standard deviation of the returns around the center along their widest direction.
constexpr double top_min_width_m = .06;
// Wall returns this recent make the fit a current observation (seconds).
constexpr double top_fresh_s = .5;
// The rim height comes from returns this recent (seconds), if there are this many of them.
constexpr double top_rim_window_s = 1.0;
constexpr int top_rim_min_points = 15;
// This many returns further than this above the estimated rim mean it is not the rim.
constexpr double top_rim_margin_m = .06, top_rim_reach_m = .3;
constexpr int top_rim_max_above = 3;
struct TopSample {
  Point p;
  double last;
};
// Merge repeated returns and drop those not seen again later: passing hands, thrown cloths
// and people do not stay in place in the robot frame. Republished scan windows would count
// several times otherwise.
std::vector<TopSample> lasting_samples(const std::deque<TimedPoints>& scans) {
  constexpr double fine_m = .01, coarse_m = .06, min_seen_s = .4;
  struct Cell {
    Point sum{0, 0, 0};
    int count{0};
    double first{std::numeric_limits<double>::infinity()}, last{-first};
  };
  const auto key = [](const Point& p, double size) {
    return std::array<int, 3>{static_cast<int>(std::floor(p.x() / size)),
                              static_cast<int>(std::floor(p.y() / size)),
                              static_cast<int>(std::floor(p.z() / size))};
  };
  std::map<std::array<int, 3>, Cell> fine, coarse;
  for (const auto& scan : scans)
    for (const auto& p : scan.points) {
      if (!p.allFinite()) continue;
      for (auto* group : {&fine, &coarse}) {
        auto& cell = (*group)[key(p, group == &fine ? fine_m : coarse_m)];
        cell.sum += p;
        ++cell.count;
        cell.first = std::min(cell.first, scan.stamp);
        cell.last = std::max(cell.last, scan.stamp);
      }
    }
  std::vector<TopSample> out;
  out.reserve(fine.size());
  for (const auto& item : fine) {
    const Point p = item.second.sum / item.second.count;
    const auto& seen = coarse.at(key(p, coarse_m));
    if (seen.last - seen.first >= min_seen_s) out.push_back({p, item.second.last});
  }
  return out;
}
}  // namespace

std::optional<Bottom> fit_bucket_top(const std::deque<TimedPoints>& scans,
                                     const TopFitContext& context, const Config& cfg,
                                     TopFitStats* stats) {
  TopFitStats local;
  TopFitStats& info = stats ? *stats : local;
  const auto reject = [&](const char* reason) -> std::optional<Bottom> {
    info.reason = reason;
    return std::nullopt;
  };
  const auto& c = cfg.bottom;
  const double sigma = std::max(context.sigma, 1e-3);
  auto samples = lasting_samples(scans);
  if (static_cast<int>(samples.size()) < c.top_min_points) return reject("too few lasting returns");
  std::sort(samples.begin(), samples.end(),
            [](const TopSample& a, const TopSample& b) { return a.p.z() > b.p.z(); });
  double floor_z = 0;
  bool floor_known = false;
  if (context.expected_floor_z) {
    floor_z = *context.expected_floor_z;
    floor_known = true;
    info.top_z = floor_z + c.bucket_height;
    info.seen_floor_z = floor_z;
  } else {
    // Cloths hanging over the rim and stray returns must not define the height. Wall returns are
    // spread evenly over the band, so the height below which most of them lie locates the rim.
    // A bucket may be lifting, so the newest returns decide when there are enough of them.
    constexpr double rim_share = .1;
    const auto rim_height = [&](double since) -> std::optional<double> {
      std::vector<double> heights;
      for (const auto& s : samples)
        if (s.last >= since) heights.push_back(s.p.z());
      if (heights.size() < 3) return std::nullopt;
      const double highest = heights[2];
      const size_t in_band = std::count_if(heights.begin(), heights.end(), [&](double z) {
        return z >= highest - c.top_band_m;
      });
      if (static_cast<int>(in_band) < top_rim_min_points) return std::nullopt;
      return heights[std::min(heights.size() - 1, static_cast<size_t>(rim_share * in_band))] +
             rim_share * c.top_band_m;
    };
    auto recent_top = rim_height(context.stamp - top_rim_window_s);
    if (!recent_top) recent_top = rim_height(-std::numeric_limits<double>::infinity());
    if (!recent_top) return reject("too few returns at the top");
    const double seen_top = *recent_top;
    info.top_z = seen_top;
    // A sparse rim does not last long enough to be kept, and the top of the robot below it then
    // looks like the rim. Returns above the estimate, kept or not, show that it is too low.
    std::vector<double> band_x, band_y;
    for (const auto& s : samples)
      if (s.p.z() >= seen_top - c.top_band_m && s.p.z() <= seen_top) {
        band_x.push_back(s.p.x());
        band_y.push_back(s.p.y());
      }
    int above = 0;
    if (!band_x.empty()) {
      std::nth_element(band_x.begin(), band_x.begin() + band_x.size() / 2, band_x.end());
      std::nth_element(band_y.begin(), band_y.begin() + band_y.size() / 2, band_y.end());
      const XY middle(band_x[band_x.size() / 2], band_y[band_y.size() / 2]);
      for (const auto& scan : scans) {
        if (scan.stamp < context.stamp - top_rim_window_s) continue;
        for (const auto& p : scan.points)
          above += p.allFinite() && p.z() > seen_top + top_rim_margin_m &&
                   (p.head<2>() - middle).norm() < top_rim_reach_m;
      }
    }
    // A short view cuts off the rim, so the highest return says nothing about the bucket height.
    if (above >= top_rim_max_above ||
        (context.view_ceiling_z && seen_top + .03 >= *context.view_ceiling_z))
      return reject("top is at the edge of the view");
    floor_z = seen_top - c.bucket_height;
    info.seen_floor_z = floor_z;
  }
  const double top_z = floor_z + c.bucket_height;
  if (top_z < cfg.min_opening_z - c.height_tolerance ||
      top_z > cfg.max_opening_z + c.height_tolerance)
    return reject("top outside the rulebook height");
  std::vector<TopSample> band;
  for (const auto& s : samples)
    if (s.p.z() >= top_z - c.top_band_m && s.p.z() <= top_z + .05) band.push_back(s);
  info.band_points = static_cast<int>(band.size());
  if (static_cast<int>(band.size()) < c.top_min_points) return reject("too few returns in the band");
  // Bounded work: a near robot returns far more than this estimate can use.
  constexpr size_t max_band_points = 150;
  std::vector<TopSample> used;
  const size_t stride = std::max<size_t>(1, (band.size() + max_band_points - 1) / max_band_points);
  for (size_t i = 0; i < band.size(); i += stride) used.push_back(band[i]);
  XY seed = XY::Zero();
  for (const auto& s : used) seed += s.p.head<2>();
  seed /= used.size();
  const auto radius_at = [&](const TopSample& s) {
    return shape_radius(c.floor_radius, s.p.z() - floor_z, cfg);
  };
  // A transparent wall returns weakly, and the near and far walls are too close for the sensor
  // to separate, so returns appear on the wall and anywhere between the walls. Outside the wall
  // they are unexplained: only the bucket may occupy this band.
  const double wall_sigma = std::max(sigma, .012);
  const double wall_norm = 1. / (std::sqrt(2 * pi) * wall_sigma);
  constexpr double wall_share = .45, inner_share = .45, stray_density = .1 / (.6 * .6);
  const auto log_likelihood = [&](const XY& center) {
    double sum = 0;
    for (const auto& s : used) {
      const double r = radius_at(s);
      const double e = (s.p.head<2>() - center).norm() - r;
      const double ring = wall_norm * std::exp(-.5 * e * e / (wall_sigma * wall_sigma)) / (2 * pi * r);
      const double inner = 1. / (1. + std::exp(1.7 * e / wall_sigma)) / (pi * r * r);
      sum += std::log(wall_share * ring + inner_share * inner + stray_density);
    }
    return sum;
  };
  // The likelihood is broad and uneven when returns are few, so its peak jumps between scans.
  // Its mean does not, and its spread tells whether the returns locate the bucket at all.
  constexpr double step = .015, temper = .5;
  const int cells = static_cast<int>(std::ceil((cfg.nominal_radius + .03) / step));
  std::vector<std::pair<double, XY>> grid;
  grid.reserve((2 * cells + 1) * (2 * cells + 1));
  double peak = -std::numeric_limits<double>::infinity();
  for (int ix = -cells; ix <= cells; ++ix)
    for (int iy = -cells; iy <= cells; ++iy) {
      const XY candidate = seed + XY(ix * step, iy * step);
      const double value = log_likelihood(candidate);
      grid.emplace_back(value, candidate);
      peak = std::max(peak, value);
    }
  double total = 0;
  XY center = XY::Zero();
  for (const auto& cell : grid) {
    const double weight = std::exp(temper * (cell.first - peak));
    total += weight;
    center += weight * cell.second;
  }
  center /= total;
  Eigen::Matrix2d spread = Eigen::Matrix2d::Zero();
  for (const auto& cell : grid) {
    const XY delta = cell.second - center;
    spread += std::exp(temper * (cell.first - peak)) * delta * delta.transpose();
  }
  spread /= total;
  const double uncertainty =
      std::sqrt(Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(spread).eigenvalues()[1]);
  info.uncertainty = uncertainty;
  if (uncertainty > top_max_uncertainty_m) return reject("returns do not locate a center");
  int inliers = 0, outside = 0, fresh = 0;
  double squared = 0, low = std::numeric_limits<double>::infinity(), high = -low;
  std::array<bool, 24> sectors{};
  Eigen::Matrix2d scatter = Eigen::Matrix2d::Zero();
  for (const auto& s : band) {
    const XY d = s.p.head<2>() - center;
    const double r = radius_at(s);
    const double e = d.norm() - r;
    if (e > 3 * wall_sigma) {
      ++outside;
      continue;
    }
    scatter += d * d.transpose();
    low = std::min(low, s.p.z());
    high = std::max(high, s.p.z());
    if (d.norm() > .5 * r)
      sectors[std::clamp(static_cast<int>((std::atan2(d.y(), d.x()) + pi) / (2 * pi) * sectors.size()),
                         0, static_cast<int>(sectors.size()) - 1)] = true;
    if (std::abs(e) <= 2 * wall_sigma) {
      ++inliers;
      squared += e * e;
    }
    fresh += s.last >= context.stamp - top_fresh_s;
  }
  info.inliers = inliers;
  info.outside = outside;
  info.fresh = fresh;
  info.arc = std::count(sectors.begin(), sectors.end(), true) * 2 * pi / sectors.size();
  // A filled or a half-seen disk of the bucket's size spreads at least this much across its
  // widest direction. A post, a head or a stray patch at the top does not.
  const double width = std::sqrt(
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(scatter / std::max<size_t>(1, band.size() - outside))
          .eigenvalues()[1]);
  info.width = width;
  if (outside > top_max_outside_ratio * band.size()) return reject("band is wider than the bucket");
  if (inliers < c.top_min_inliers) return reject("too few wall returns");
  if (info.arc < c.top_min_arc) return reject("wall arc too short");
  if (width < top_min_width_m) return reject("band is narrower than the bucket");
  if (high - low < top_min_span_ratio * c.top_band_m) return reject("wall height span too short");
  if (fresh < 2) return reject("no current wall returns");
  Bottom result;
  result.center = {center.x(), center.y(), floor_z};
  result.normal = Point::UnitZ();
  result.radius = c.floor_radius;
  result.boundary_residual = std::sqrt(squared / inliers);
  result.current_points = fresh;
  result.side_points = inliers;
  result.score = std::log1p(inliers) + info.arc / pi - static_cast<double>(outside) / band.size();
  info.reason = floor_known ? "fitted with known floor" : "fitted";
  return result;
}

} // namespace lidar_perception_system::opening
