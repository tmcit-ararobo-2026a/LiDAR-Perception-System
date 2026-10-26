// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
#include "lidar_perception_system/opening_tracker.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace lidar_perception_system::opening {
namespace {
constexpr double pi = 3.14159265358979323846;
struct Key {
  int x, y, z;
  bool operator==(const Key& b) const { return x == b.x && y == b.y && z == b.z; }
};
struct Hash {
  size_t operator()(const Key& k) const {
    size_t h = std::hash<int>{}(k.x);
    h ^= std::hash<int>{}(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};
Key key(const Point& p, double size) {
  return {static_cast<int>(std::floor(p.x() / size)), static_cast<int>(std::floor(p.y() / size)),
          static_cast<int>(std::floor(p.z() / size))};
}
Points voxelize(const Points& input, double size, size_t limit) {
  if (limit == 0 || input.empty()) return {};
  struct Cell {
    Point sum{0, 0, 0};
    int count{0};
  };
  std::unordered_map<Key, Cell, Hash> cells;
  for (const auto& p : input) {
    auto& c = cells[key(p, size)];
    c.sum += p;
    ++c.count;
  }
  std::vector<std::pair<Key, Point>> ordered;
  ordered.reserve(cells.size());
  for (const auto& item : cells) {
    ordered.push_back({item.first, item.second.sum / item.second.count});
  }
  // Stable spatial ordering also makes decimation independent of hash iteration
  // order.
  std::sort(ordered.begin(), ordered.end(), [](const auto& a, const auto& b) {
    return std::tie(a.first.x, a.first.y, a.first.z) < std::tie(b.first.x, b.first.y, b.first.z);
  });
  Points output;
  const size_t stride = std::max(size_t{1}, (ordered.size() + limit - 1) / limit);
  for (size_t i = 0; i < ordered.size(); i += stride) {
    output.push_back(ordered[i].second);
  }
  return output;
}
double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double arc_span(std::vector<double> a) {
  if (a.size() < 2) {
    return 0;
  }
  std::sort(a.begin(), a.end());
  double gap = a.front() + 2 * pi - a.back();
  for (size_t i = 1; i < a.size(); ++i) {
    gap = std::max(gap, a[i] - a[i - 1]);
  }
  return 2 * pi - gap;
}
double median(std::vector<double> a) {
  auto middle = a.begin() + a.size() / 2;
  std::nth_element(a.begin(), middle, a.end());
  return *middle;
}
Points local_points(const Points& p, const Pose& pose) {
  Points output;
  output.reserve(p.size());
  for (const auto& v : p) {
    output.push_back(pose.local(v));
  }
  return output;
}
}  // namespace

Point Pose::world(const Point& p) const {
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {c * p.x() - s * p.y() + xy.x(), s * p.x() + c * p.y() + xy.y(), p.z()};
}
Point Pose::local(const Point& p) const {
  const double c = std::cos(yaw), s = std::sin(yaw), x = p.x() - xy.x(), y = p.y() - xy.y();
  return {c * x + s * y, -s * x + c * y, p.z()};
}
bool Config::inside_field(const Point& p) const {
  return p.allFinite() && p.x() >= field_min_x && p.x() <= field_max_x && p.y() >= field_min_y &&
         p.y() <= field_max_y;
}
void Config::validate() const {
  for (double v : {min_x,
                   max_x,
                   min_y,
                   max_y,
                   field_min_x,
                   field_max_x,
                   field_min_y,
                   field_max_y,
                   min_z,
                   max_z,
                   self_radius,
                   body_min_z,
                   body_max_z,
                   tracking_body_max_z,
                   cluster_resolution,
                   min_span,
                   max_span,
                   max_body_side,
                   body_size_tolerance,
                   crop_margin,
                   body_voxel,
                   opening_voxel,
                   window_s,
                   correspondence_m,
                   max_registration_rmse,
                   min_overlap,
                   min_support_spread_ratio,
                   min_opening_z,
                   max_opening_z,
                   min_radius,
                   max_radius,
                   nominal_radius,
                   slice_m,
                   max_residual,
                   max_plane_rmse,
                   min_arc,
                   height_change_m,
                   min_lip_width,
                   prediction_s,
                   reset_s,
                   body_recovery_s,
                   reidentification_s,
                   reidentification_m,
                   association_m}) {
    if (!std::isfinite(v)) {
      throw std::invalid_argument("Opening config must be finite");
    }
  }
  for (double v :
       {bottom.bucket_height, bottom.height_tolerance, bottom.min_radius, bottom.max_radius,
        bottom.plane_distance, bottom.max_plane_rmse, bottom.max_tilt_rad, bottom.connection_m,
        bottom.side_tolerance, bottom.min_side_span, bottom.max_body_inference_s}) {
    if (!std::isfinite(v)) throw std::invalid_argument("Bottom config must be finite");
  }
  if (bottom.bucket_height < .1 || bottom.bucket_height > .5 || bottom.height_tolerance < 0 ||
      bottom.height_tolerance > .08 || bottom.min_radius < .02 ||
      bottom.max_radius <= bottom.min_radius || bottom.max_radius > max_radius ||
      bottom.plane_distance < .003 || bottom.plane_distance > .025 || bottom.max_plane_rmse <= 0 ||
      bottom.max_plane_rmse > bottom.plane_distance || bottom.max_tilt_rad <= 0 ||
      bottom.max_tilt_rad > .35 || bottom.connection_m < .01 || bottom.connection_m > .08 ||
      bottom.side_tolerance <= 0 || bottom.side_tolerance > .05 || bottom.min_side_span < .05 ||
      bottom.min_side_span > bottom.bucket_height || bottom.min_plane_points < 6 ||
      bottom.min_plane_points > max_model_points || bottom.min_current_points < 3 ||
      bottom.min_side_points < 3 || bottom.max_body_inference_s < 0 ||
      bottom.max_body_inference_s > 10 || bottom.shape_tracking_min_points < 3 ||
      bottom.shape_tracking_min_points > max_model_points) {
    throw std::invalid_argument("Invalid bottom geometry bounds");
  }
  if (field_min_x >= field_max_x || field_min_y >= field_max_y ||
      std::max({std::abs(field_min_x), std::abs(field_max_x), std::abs(field_min_y),
                std::abs(field_max_y)}) > 100 ||
      max_body_side <= 0 || max_body_side > 3 || body_size_tolerance < 0 ||
      body_size_tolerance > .2 || min_x >= max_x || min_y >= max_y || min_z >= max_z ||
      min_support_spread_ratio < 0 || min_support_spread_ratio > 1 ||
      std::max({std::abs(min_x), std::abs(max_x), std::abs(min_y), std::abs(max_y), std::abs(min_z),
                std::abs(max_z)}) > 100 ||
      body_min_z < min_z || body_min_z >= body_max_z || body_max_z >= min_opening_z ||
      tracking_body_max_z < body_max_z || tracking_body_max_z > max_z ||
      min_opening_z >= max_opening_z || max_opening_z > max_z || self_radius < 0 ||
      crop_margin < 0 || min_span <= 0 || max_span < min_span || cluster_resolution < 0.01 ||
      body_voxel < 0.005 || opening_voxel < 0.001 || min_radius <= 0 || min_radius >= max_radius ||
      nominal_radius < min_radius || nominal_radius > max_radius || slice_m < 0.002 ||
      max_opening_z - min_opening_z > 5 || max_residual <= 0 || max_plane_rmse <= 0 ||
      min_arc <= 0 || min_arc > 2 * pi || height_change_m <= 0 || min_lip_width < 0 ||
      min_lip_width > max_residual || window_s <= 0 || window_s > 5 || prediction_s <= 0 ||
      prediction_s > reset_s || reset_s > 10 || body_recovery_s < 0 || body_recovery_s > reset_s ||
      reidentification_s < 0 || reidentification_s > 30 || reidentification_m <= 0 ||
      reidentification_m > 10 ||
      correspondence_m <= 0 || correspondence_m > 1 || max_registration_rmse <= 0 ||
      min_overlap <= 0 || min_overlap > 1 || association_m <= 0 || icp_iterations < 1 ||
      icp_iterations > 100 || ransac_iterations < 1 || ransac_iterations > 1000 ||
      min_body_points < 3 || min_rim_points < 3 || min_current_points < 3 || min_side_points < 1 ||
      confirm_hits < 1 || max_body_points < min_body_points || max_body_points > 20000 ||
      max_model_points < max_body_points || max_model_points > 200000 ||
      max_frame_points < max_model_points || max_frame_points > 1000000) {
    throw std::invalid_argument("Invalid opening tracker bounds or limits");
  }
}
Tracker::Tracker(Config config) : cfg_(config) { cfg_.validate(); }
void Tracker::reset() {
  clear_model();
  lost_model_.reset();
  acquisition_.reset();
}
void Tracker::clear_model() {
  frames_.clear();
  selected_.clear();
  accumulated_.clear();
  reference_body_.clear();
  reference_upper_.clear();
  have_body_ = false;
  identified_ = false;
  motion_seeded_ = false;
  confirmed_ = false;
  height_pending_ = false;
  hits_ = 0;
  last_opening_.reset();
  last_bottom_.reset();
  learned_bottom_.reset();
  learned_bottom_stamp_ = 0;
  bottom_hits_ = 0;
  bottom_pending_shape_ = false;
  bottom_stamp_ = 0;
  body_stamp_ = 0;
  acquired_stamp_ = 0;
  rejection_until_ = 0;
  rejected_center_.reset();
  input_stamp_ = -1;
  observation_stamp_ = 0;
  body_velocity_.setZero();
  velocity_.setZero();
  yaw_rate_ = 0;
}
void Tracker::set_exclusions(const std::vector<Eigen::Vector3d>& masks) {
  exclusions_ = masks;
  acquisition_.set_exclusions(masks);
}

bool Tracker::body_size_valid(const Points& points) const {
  if (points.empty()) return false;
  const size_t lower = std::count_if(points.begin(), points.end(), [&](const Point& p) {
    return p.z() >= cfg_.body_min_z && p.z() <= cfg_.body_max_z;
  });
  if (lower < 3) return false;
  Eigen::Vector2d lower_lo = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d lower_hi = -lower_lo;
  for (const auto& p : points) {
    if (p.z() < cfg_.body_min_z || p.z() > cfg_.body_max_z) continue;
    lower_lo = lower_lo.cwiseMin(p.head<2>());
    lower_hi = lower_hi.cwiseMax(p.head<2>());
  }
  if ((lower_hi - lower_lo).norm() < .10) return false;
  // Search square orientations, not map-axis widths. A legal 1.2 m square
  // spans 1.70 m in map X/Y at 45 degrees. One-degree sampling contributes
  // at most 11 mm at the default size, covered by the measurement tolerance.
  const double limit = cfg_.max_body_side + cfg_.body_size_tolerance;
  for (int degrees = 0; degrees < 90; ++degrees) {
    const double a = degrees * pi / 180, c = std::cos(a), s = std::sin(a);
    Eigen::Vector2d lo = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector2d hi = -lo;
    for (const auto& p : points) {
      if (p.z() < cfg_.body_min_z || p.z() > cfg_.body_max_z) continue;
      const Eigen::Vector2d q(c * p.x() + s * p.y(), -s * p.x() + c * p.y());
      lo = lo.cwiseMin(q);
      hi = hi.cwiseMax(q);
      if ((hi - lo).maxCoeff() > limit) break;
    }
    if ((hi - lo).maxCoeff() <= limit) return true;
  }
  return false;
}

std::vector<Tracker::Cluster> Tracker::clusters(const Points& points) const {
  std::unordered_map<Key, Points, Hash> cells;
  for (const auto& p : points) {
    if (p.z() < cfg_.body_min_z || p.z() > cfg_.body_max_z) {
      continue;
    }
    auto k = key(p, cfg_.cluster_resolution);
    k.z = 0;
    cells[k].push_back(p);
  }
  std::vector<Key> starts;
  for (const auto& c : cells) {
    starts.push_back(c.first);
  }
  std::sort(starts.begin(), starts.end(),
            [](const auto& a, const auto& b) { return std::tie(a.x, a.y) < std::tie(b.x, b.y); });
  std::unordered_set<Key, Hash> visited;
  std::vector<Cluster> result;
  for (const auto& start : starts) {
    if (!visited.insert(start).second) {
      continue;
    }
    std::vector<Key> todo{start};
    Cluster c;
    c.center.setZero();
    c.min_x = c.min_y = std::numeric_limits<double>::infinity();
    c.max_x = c.max_y = -c.min_x;
    for (size_t i = 0; i < todo.size(); ++i) {
      for (const auto& p : cells.at(todo[i])) {
        c.body.push_back(p);
        c.center += p.head<2>();
        c.min_x = std::min(c.min_x, p.x());
        c.max_x = std::max(c.max_x, p.x());
        c.min_y = std::min(c.min_y, p.y());
        c.max_y = std::max(c.max_y, p.y());
      }
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          const Key n{todo[i].x + dx, todo[i].y + dy, 0};
          if (cells.count(n) && visited.insert(n).second) {
            todo.push_back(n);
          }
        }
      }
    }
    if (static_cast<int>(c.body.size()) < cfg_.min_body_points ||
        c.max_x - c.min_x < cfg_.min_span || c.max_y - c.min_y < cfg_.min_span ||
        c.max_x - c.min_x > cfg_.max_span || c.max_y - c.min_y > cfg_.max_span) {
      continue;
    }
    if (!body_size_valid(c.body)) continue;
    c.center /= c.body.size();
    c.body = voxelize(c.body, cfg_.body_voxel, cfg_.max_body_points);
    result.push_back(std::move(c));
  }
  return result;
}

bool Tracker::register_body(const Points& input, const Pose& initial, Pose& output, double& rmse,
                            double& overlap, Points* supported, bool use_upper) const {
  Points lower, reference;
  for (const auto& p : input)
    if (use_upper || p.z() <= cfg_.body_max_z) lower.push_back(p);
  for (const auto& p : reference_body_)
    if (use_upper || p.z() <= cfg_.body_max_z) reference.push_back(p);
  if (use_upper)
    reference.insert(reference.end(), reference_upper_.begin(), reference_upper_.end());
  const Points& body = lower;
  if (reference.empty() || body.empty()) {
    return false;
  }
  std::unordered_map<Key, std::vector<size_t>, Hash> hash;
  for (size_t i = 0; i < reference.size(); ++i) {
    hash[key(reference[i], cfg_.correspondence_m)].push_back(i);
  }
  Pose estimate = initial;
  struct Pair {
    Point p, q, map;
    double squared;
  };
  std::vector<Pair> complete_pairs;
  for (int iteration = 0; iteration <= cfg_.icp_iterations; ++iteration) {
    // A dense wall must not vote hundreds of times for one chassis feature.
    std::vector<std::optional<Pair>> closest(reference.size());
    std::vector<Pair> pairs;
    for (const auto& map : body) {
      const Point p = estimate.local(map);
      const auto k = key(p, cfg_.correspondence_m);
      double best = cfg_.correspondence_m * cfg_.correspondence_m;
      const Point* q = nullptr;
      size_t reference_index = 0;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            auto cell = hash.find({k.x + dx, k.y + dy, k.z + dz});
            if (cell == hash.end()) {
              continue;
            }
            for (size_t i : cell->second) {
              double d = (p - reference[i]).squaredNorm();
              if (d < best) {
                best = d;
                q = &reference[i];
                reference_index = i;
              }
            }
          }
        }
      }
      if (q && (!closest[reference_index] || best < closest[reference_index]->squared)) {
        closest[reference_index] = Pair{p, *q, map, best};
      }
    }
    for (const auto& pair : closest)
      if (pair) pairs.push_back(*pair);
    overlap = static_cast<double>(pairs.size()) /
              std::max(size_t{1}, std::min(body.size(), reference.size()));
    if (overlap < cfg_.min_overlap || pairs.size() < 12) {
      return false;
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const auto& a, const auto& b) { return a.squared < b.squared; });
    // Optimize with trimming, retaining all unique pairs so final close-match
    // coverage is measured independently of the optimizer's trimmed subset.
    double all_squared = 0;
    for (const auto& pair : pairs) all_squared += pair.squared;
    rmse = std::sqrt(all_squared / pairs.size());
    complete_pairs = pairs;
    pairs.resize(std::max(size_t{12}, pairs.size() * 7 / 10));
    Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
    Point g = Point::Zero();
    for (const auto& pair : pairs) {
      const Eigen::Vector2d e = (pair.p - pair.q).head<2>();
      Eigen::Matrix<double, 2, 3> j;
      j << 1, 0, -pair.p.y(), 0, 1, pair.p.x();
      const double w = std::min(1.0, 0.03 / std::max(1e-9, std::sqrt(pair.squared)));
      h.noalias() += w * j.transpose() * j;
      g.noalias() += w * j.transpose() * e;
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(h);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() < 1e-5 ||
        eigen.eigenvalues().minCoeff() / eigen.eigenvalues().maxCoeff() < 1e-4) {
      return false;
    }
    if (iteration == cfg_.icp_iterations) {
      break;
    }
    Point step = -h.ldlt().solve(g);
    if (!step.allFinite()) {
      return false;
    }
    const double scaling = std::max({1.0, step.head<2>().norm() / 0.1, std::abs(step.z()) / 0.15});
    step /= scaling;
    // Apply the local alignment increment; the inverse is the map pose update.
    Pose incremental;
    incremental.xy = step.head<2>();
    incremental.yaw = step.z();
    const Point new_origin = estimate.world(incremental.local(Point::Zero()));
    estimate.xy = new_origin.head<2>();
    estimate.yaw = angle(estimate.yaw - step.z());
    if (step.norm() < 1e-5) {
      break;
    }
  }
  // Reject far pairs individually, then require sufficient close support.
  // An unmatched edge or fixture must not dominate the mean error of the
  // correctly aligned part. Coverage still penalizes periodic chassis aliases.
  complete_pairs.erase(std::remove_if(complete_pairs.begin(), complete_pairs.end(),
                                      [&](const Pair& p) {
                                        return p.squared > cfg_.max_registration_rmse *
                                                               cfg_.max_registration_rmse;
                                      }),
                       complete_pairs.end());
  overlap = static_cast<double>(complete_pairs.size()) /
            std::max(size_t{1}, std::min(body.size(), reference.size()));
  if (complete_pairs.size() < 12 || overlap < cfg_.min_overlap) return false;
  double inlier_squared = 0;
  for (const auto& p : complete_pairs) inlier_squared += p.squared;
  rmse = std::sqrt(inlier_squared / complete_pairs.size());
  if (rmse > cfg_.max_registration_rmse || (estimate.xy - initial.xy).norm() > cfg_.association_m ||
      std::abs(angle(estimate.yaw - initial.yaw)) > 0.65) {
    return false;
  }
  const auto spread = [](const Points& points) {
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& p : points) mean += p.head<2>();
    mean /= points.size();
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const auto& p : points) {
      const Eigen::Vector2d delta = p.head<2>() - mean;
      covariance += delta * delta.transpose();
    }
    covariance /= points.size();
    return Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(covariance).eigenvalues().minCoeff();
  };
  Points support_geometry;
  for (const auto& pair : complete_pairs) support_geometry.push_back(pair.p);
  // A wall can match one visible chassis edge with tiny residual. Require
  // horizontal shape evidence in two directions, independent of wall angle.
  if (spread(support_geometry) < cfg_.min_support_spread_ratio *
                                     std::max(spread(reference), cfg_.body_voxel * cfg_.body_voxel))
    return false;
  if (supported) {
    supported->clear();
    for (const auto& pair : complete_pairs) {
      if (pair.squared <= cfg_.max_registration_rmse * cfg_.max_registration_rmse)
        supported->push_back(pair.map);
    }
    if (supported->size() < 12) return false;
  }
  output = estimate;
  return true;
}

std::optional<Opening> Tracker::fit_opening(const Points& input, const Points& current,
                                            FitStats* stats) const {
  if (stats) *stats = FitStats{};
  const auto points = voxelize(input, cfg_.opening_voxel, cfg_.max_model_points);
  std::optional<Opening> best;
  std::mt19937 rng(0x20261018);
  for (double z = cfg_.min_opening_z; z <= cfg_.max_opening_z + 1e-6; z += cfg_.slice_m / 2) {
    std::vector<size_t> slice;
    for (size_t i = 0; i < points.size(); ++i) {
      if (std::abs(points[i].z() - z) <= cfg_.slice_m / 2) {
        slice.push_back(i);
      }
    }
    if (stats)
      stats->max_slice_points = std::max(stats->max_slice_points, static_cast<int>(slice.size()));
    if (static_cast<int>(slice.size()) < cfg_.min_rim_points) {
      continue;
    }
    for (int iteration = 0; iteration < cfg_.ransac_iterations; ++iteration) {
      const Point &a = points[slice[rng() % slice.size()]],
                  &b = points[slice[rng() % slice.size()]],
                  &c = points[slice[rng() % slice.size()]];
      Eigen::Matrix2d m;
      m.row(0) = 2 * (b - a).head<2>();
      m.row(1) = 2 * (c - a).head<2>();
      if (std::abs(m.determinant()) < 1e-5) {
        continue;
      }
      Eigen::Vector2d rhs(b.head<2>().squaredNorm() - a.head<2>().squaredNorm(),
                          c.head<2>().squaredNorm() - a.head<2>().squaredNorm());
      Eigen::Vector2d center = m.inverse() * rhs;
      double radius = (center - a.head<2>()).norm();
      if (radius < cfg_.min_radius || radius > cfg_.max_radius) {
        continue;
      }
      if (stats) ++stats->radius_hypotheses;
      std::vector<size_t> inliers;
      for (size_t i : slice) {
        if (std::abs((points[i].head<2>() - center).norm() - radius) <= cfg_.max_residual) {
          inliers.push_back(i);
        }
      }
      if (stats)
        stats->max_inliers = std::max(stats->max_inliers, static_cast<int>(inliers.size()));
      if (static_cast<int>(inliers.size()) < cfg_.min_rim_points) {
        continue;
      }
      for (int refine = 0; refine < 5; ++refine) {
        Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
        Point g = Point::Zero();
        for (size_t i : inliers) {
          const Eigen::Vector2d d = center - points[i].head<2>();
          const double norm = d.norm();
          if (norm < 1e-6) {
            continue;
          }
          Point j(d.x() / norm, d.y() / norm, -1);
          const double e = norm - radius;
          h.noalias() += j * j.transpose();
          g += j * e;
        }
        if (std::abs(h.determinant()) < 1e-6) {
          break;
        }
        Point step = h.ldlt().solve(g);
        if (!step.allFinite()) {
          break;
        }
        center -= step.head<2>();
        radius -= step.z();
        if (step.norm() < 1e-5) {
          break;
        }
      }
      if (radius < cfg_.min_radius || radius > cfg_.max_radius) {
        continue;
      }
      std::vector<double> heights, angles, radii;
      double sum = 0;
      for (size_t i : inliers) {
        heights.push_back(points[i].z());
      }
      const double top = median(heights);
      double plane = 0;
      if (top < cfg_.min_opening_z || top > cfg_.max_opening_z) {
        continue;
      }
      for (size_t i : inliers) {
        const auto d = points[i].head<2>() - center;
        const double e = d.norm() - radius;
        sum += e * e;
        radii.push_back(d.norm());
        plane += std::pow(points[i].z() - top, 2);
        angles.push_back(std::atan2(d.y(), d.x()));
      }
      const double residual = std::sqrt(sum / inliers.size()), span = arc_span(angles);
      if (stats) stats->max_arc = std::max(stats->max_arc, span);
      const double plane_rmse = std::sqrt(plane / inliers.size());
      if (residual > cfg_.max_residual || span < cfg_.min_arc || plane_rmse > cfg_.max_plane_rmse) {
        continue;
      }
      if (stats) ++stats->geometry_hypotheses;
      std::sort(radii.begin(), radii.end());
      // A horizontal lip has radial extent. An isolated wall contour alone is
      // not enough evidence that its highest visible return is the true rim.
      if (radii[radii.size() * 9 / 10] - radii[radii.size() / 10] < cfg_.min_lip_width) {
        continue;
      }
      int below = 0, above = 0, interior = 0;
      for (const auto& p : points) {
        const double radial = (p.head<2>() - center).norm();
        if (radial < radius * 0.65 && std::abs(p.z() - top) <= cfg_.slice_m / 2) {
          ++interior;
        }
        if (p.z() < top - 0.03 && p.z() > top - 0.22 && std::abs(radial - radius) < 0.06) {
          ++below;
        }
        if (p.z() > top + cfg_.slice_m / 2 + 0.002 && p.z() < top + 0.15 &&
            std::abs(radial - radius) < 0.025) {
          ++above;
        }
      }
      // A side-wall cross section has continuing shell above it, unlike a top
      // edge.
      if (below < cfg_.min_side_points || above >= 3) {
        continue;
      }
      // A circle fitted through a solid deck is not an opening. Distinct
      // coplanar returns in its central disk contradict a hollow mouth.
      if (interior >= 3) {
        if (stats) ++stats->interior_rejections;
        continue;
      }
      if (stats) ++stats->edge_hypotheses;
      int fresh = 0;
      std::vector<double> fresh_angles;
      std::array<double, 128> fresh_min_radius, fresh_max_radius;
      fresh_min_radius.fill(std::numeric_limits<double>::infinity());
      fresh_max_radius.fill(-std::numeric_limits<double>::infinity());
      for (const auto& p : current) {
        const auto d = p.head<2>() - center;
        if (std::abs(p.z() - top) <= cfg_.slice_m / 2 &&
            std::abs(d.norm() - radius) <= cfg_.max_residual) {
          ++fresh;
          const double a = std::atan2(d.y(), d.x());
          fresh_angles.push_back(a);
          const int bin = std::clamp(static_cast<int>((a + pi) / (2 * pi) * 128), 0, 127);
          fresh_min_radius[bin] = std::min(fresh_min_radius[bin], d.norm());
          fresh_max_radius[bin] = std::max(fresh_max_radius[bin], d.norm());
        }
      }
      if (fresh < cfg_.min_current_points || arc_span(fresh_angles) < 0.35) {
        continue;
      }
      int fresh_lip_bins = 0;
      for (size_t bin = 0; bin < fresh_min_radius.size(); ++bin) {
        fresh_lip_bins += fresh_max_radius[bin] - fresh_min_radius[bin] >= cfg_.min_lip_width;
      }
      // Registration drift can broaden an accumulated wall contour. Require
      // radial lip width in two narrow angular bins of the current scan too.
      if (fresh_lip_bins < 2) {
        continue;
      }
      Opening candidate;
      candidate.center = {center.x(), center.y(), top};
      candidate.radius = radius;
      candidate.residual = residual;
      candidate.plane_rmse = plane_rmse;
      candidate.arc = span;
      candidate.points = static_cast<int>(inliers.size());
      candidate.current_points = fresh;
      candidate.score = 2 * span + std::log1p(candidate.points) - 100 * residual - 100 * plane_rmse;
      const bool same_axis =
          best && (candidate.center.head<2>() - best->center.head<2>()).norm() < 0.04;
      if (!best || (same_axis && candidate.center.z() > best->center.z() + 0.008) ||
          (!(same_axis && best->center.z() > candidate.center.z() + 0.008) &&
           candidate.score > best->score)) {
        best = candidate;
      }
    }
  }
  return best;
}

void Tracker::rebuild(double stamp) {
  while (!frames_.empty() && stamp - frames_.front().stamp > cfg_.window_s) {
    frames_.pop_front();
  }
  Points body, upper;
  for (const auto& frame : frames_) {
    body.insert(body.end(), frame.body.begin(), frame.body.end());
    upper.insert(upper.end(), frame.upper.begin(), frame.upper.end());
  }
  // Keep a last-good registration reference during short occlusions. Its age
  // never counts as a new opening observation, and it is removed by the reset
  // deadline.
  if (!body.empty()) {
    // Preserve learned rigid geometry through partial visibility. New frames
    // contain only unique, close registration support after acquisition.
    if (identified_ || motion_seeded_)
      body.insert(body.end(), reference_body_.begin(), reference_body_.end());
    body.erase(std::remove_if(body.begin(), body.end(),
                              [&](const Point& p) { return !cfg_.inside_field(pose_.world(p)); }),
               body.end());
    Points lower, upper_body;
    for (const auto& p : body) {
      if (p.z() <= cfg_.body_max_z)
        lower.push_back(p);
      else if (!learned_bottom_ ||
               (p.head<2>() - learned_bottom_->center.head<2>()).norm() >= cfg_.max_radius + .04)
        upper_body.push_back(p);
    }
    reference_body_ = voxelize(lower, cfg_.body_voxel, cfg_.max_body_points);
    reference_upper_ =
        voxelize(upper_body, cfg_.body_voxel, std::min(500, cfg_.max_body_points / 4));
  } else {
    reference_upper_.clear();
  }
  upper.erase(std::remove_if(upper.begin(), upper.end(),
                             [&](const Point& p) { return !cfg_.inside_field(pose_.world(p)); }),
              upper.end());
  accumulated_ = voxelize(upper, cfg_.opening_voxel, cfg_.max_model_points);
}
std::optional<Tracker::Reidentification> Tracker::reidentify(
    const std::vector<Cluster>& candidates, double stamp) {
  if (!lost_model_) return {};
  auto& lost = *lost_model_;
  if (stamp < lost.body_stamp || stamp - lost.body_stamp > cfg_.reidentification_s ||
      (cfg_.bottom.max_body_inference_s > 0 &&
       stamp - lost.bottom_stamp > cfg_.bottom.max_body_inference_s)) {
    lost_model_.reset();
    return {};
  }
  // Search the dormant template at most twice a second. A pending match is
  // checked on each scan; no dormant pose or semantic anchor is published.
  if (!lost.hits && stamp - lost.attempt_stamp < .5) return {};
  lost.attempt_stamp = stamp;
  Tracker probe(cfg_);
  probe.reference_body_ = lost.body;
  Point center = Point::Zero();
  Eigen::Vector2d model_lo = lost.body.front().head<2>(), model_hi = model_lo;
  for (const auto& p : lost.body) {
    center += p;
    model_lo = model_lo.cwiseMin(p.head<2>());
    model_hi = model_hi.cwiseMax(p.head<2>());
  }
  center /= lost.body.size();
  const auto required_support =
      std::max({size_t{40}, static_cast<size_t>(cfg_.min_body_points), lost.body.size() / 8});
  std::vector<const Cluster*> ranked;
  for (const auto& c : candidates)
    if ((c.center - lost.pose.world(center).head<2>()).norm() <= cfg_.reidentification_m)
      ranked.push_back(&c);
  std::sort(ranked.begin(), ranked.end(), [&](const Cluster* a, const Cluster* b) {
    return (a->center - lost.pose.world(center).head<2>()).squaredNorm() <
           (b->center - lost.pose.world(center).head<2>()).squaredNorm();
  });
  std::optional<Reidentification> best;
  double score = std::numeric_limits<double>::infinity();
  bool ambiguous = false;
  for (size_t i = 0; i < std::min(size_t{3}, ranked.size()); ++i) {
    const auto& c = *ranked[i];
    if (c.body.size() < required_support) continue;
    std::vector<double> headings;
    if (lost.hits) headings.push_back(lost.pending_pose.yaw);
    // A dormant model is not extrapolated using stale velocity. Search yaw
    // around a cluster centroid, then verify the actual rigid correspondences.
    if (!lost.hits)
      for (int k = 0; k < 24; ++k) headings.push_back(lost.pose.yaw + k * pi / 12);
    for (double heading : headings) {
      Pose seed, fitted;
      seed.yaw = angle(heading);
      seed.xy = c.center - seed.world(center).head<2>();
      Points support;
      double rmse = 0, overlap = 0;
      if (!probe.register_body(c.body, seed, fitted, rmse, overlap, &support) ||
          !body_size_valid(support) || overlap < std::max(.60, cfg_.min_overlap) ||
          support.size() < required_support ||
          (fitted.xy - lost.pose.xy).norm() > cfg_.reidentification_m)
        continue;
      Eigen::Vector2d lo = fitted.local(support.front()).head<2>(), hi = lo;
      for (const auto& p : support) {
        const auto q = fitted.local(p).head<2>().eval();
        lo = lo.cwiseMin(q);
        hi = hi.cwiseMax(q);
      }
      if (((hi - lo).array() < .5 * (model_hi - model_lo).array()).any()) continue;
      const double coverage = static_cast<double>(support.size()) / lost.body.size();
      const double quality = rmse + cfg_.max_registration_rmse * (2 - overlap - coverage);
      const bool different = best && ((best->pose.xy - fitted.xy).norm() > .15 ||
                                      std::abs(angle(best->pose.yaw - fitted.yaw)) > .25);
      if (different && std::abs(quality - score) < .005) ambiguous = true;
      if (quality < score) {
        if (quality + .005 < score) ambiguous = false;
        best = Reidentification{c, fitted, rmse, overlap};
        best->cluster.body = std::move(support);
        score = quality;
      }
    }
  }
  if (!best || ambiguous) {
    lost.hits = 0;
    return {};
  }
  const bool consistent = lost.hits && stamp - lost.pending_stamp <= cfg_.prediction_s &&
                          (best->pose.xy - lost.pending_pose.xy).norm() < cfg_.association_m &&
                          std::abs(angle(best->pose.yaw - lost.pending_pose.yaw)) < .4;
  lost.hits = consistent ? lost.hits + 1 : 1;
  lost.pending_pose = best->pose;
  lost.pending_stamp = stamp;
  if (lost.hits < std::max(3, cfg_.confirm_hits)) return {};
  return best;
}
Result Tracker::missing(double stamp, const std::string& reason, Result result) {
  result.stamp = stamp;
  result.track_id = track_id_;
  result.pose = pose_;
  result.motion_seeded = motion_seeded_;
  result.bottom_observation_stamp = bottom_stamp_;
  result.reason = reason;
  result.observation_stamp = observation_stamp_;
  result.model_points = static_cast<int>(accumulated_.size());
  if (confirmed_ && !height_pending_ && stamp >= observation_stamp_ &&
      stamp - observation_stamp_ < cfg_.prediction_s) {
    result.valid = true;
    result.predicted = true;
    result.position = position_ + velocity_ * (stamp - observation_stamp_);
    if (result.body_valid && last_opening_) {
      // The learned mounting offset rotates with the observed robot. This is
      // still a prediction: body registration must not renew the rim timestamp.
      result.position = pose_.world(last_opening_->center);
      const Point lever = result.position - Point(pose_.xy.x(), pose_.xy.y(), 0);
      result.velocity = Point(body_velocity_.x() - yaw_rate_ * lever.y(),
                              body_velocity_.y() + yaw_rate_ * lever.x(), 0);
    } else {
      result.velocity = velocity_;
    }
    result.position.z() = position_.z();
    result.opening = last_opening_;
    result.position_variance = variance_ + 0.1 * (stamp - observation_stamp_);
    if (!cfg_.inside_field(result.position)) {
      result.valid = result.predicted = false;
      result.opening.reset();
    }
  }
  return result;
}
Result Tracker::process(const Points& map_points, double stamp, const Eigen::Vector2d& sensor_xy) {
  Result result;
  selected_.clear();
  if (!std::isfinite(stamp) || stamp <= 0 || !sensor_xy.allFinite()) {
    return missing(stamp, "invalid input time/pose");
  }
  if (input_stamp_ >= 0 && stamp < input_stamp_) {
    reset();
  }
  if (stamp == input_stamp_) {
    result.reason = "duplicate input";
    result.stamp = stamp;
    return result;
  }
  input_stamp_ = stamp;
  const bool unconfirmed_timeout =
      have_body_ && !identified_ && !motion_seeded_ && stamp - acquired_stamp_ > cfg_.reset_s;
  // Retain an identified robot while its body is still observed, even if the
  // opening is occluded. Its expired opening remains invalid; body registration
  // never renews the opening observation stamp or its prediction allowance.
  if (have_body_ && (stamp - body_stamp_ > cfg_.reset_s || unconfirmed_timeout)) {
    const auto rejected = cluster_center_;
    // A failed template after missing observations is not evidence of a fixed
    // fixture. Do not blacklist a real opponent's last location for another
    // reset interval after the model times out.
    const bool reject =
        unconfirmed_timeout && !height_pending_ && stamp - body_stamp_ <= cfg_.prediction_s;
    if (identified_ && learned_bottom_ && cfg_.reidentification_s > cfg_.reset_s &&
        !reference_body_.empty()) {
      lost_model_ = LostModel{reference_body_, pose_, *learned_bottom_, body_stamp_,
                             learned_bottom_stamp_, -1, -1, track_id_, 0, Pose{}};
    }
    clear_model();
    if (reject) {
      rejected_center_ = rejected;
      rejection_until_ = stamp + cfg_.reset_s;
    }
    input_stamp_ = stamp;
  }
  rebuild(stamp);
  Points filtered;
  filtered.reserve(std::min(map_points.size(), static_cast<size_t>(cfg_.max_frame_points)));
  const auto accepted = [&](const Point& p) {
    return cfg_.inside_field(p) && p.x() >= cfg_.min_x && p.x() <= cfg_.max_x &&
           p.y() >= cfg_.min_y && p.y() <= cfg_.max_y && p.z() >= cfg_.min_z &&
           p.z() <= cfg_.max_z && (p.head<2>() - sensor_xy).norm() >= cfg_.self_radius;
  };
  // Outside returns must not consume the decimation budget of field points.
  const size_t count = std::count_if(map_points.begin(), map_points.end(), accepted);
  const size_t stride =
      std::max(size_t{1}, (count + cfg_.max_frame_points - 1) / cfg_.max_frame_points);
  size_t accepted_index = 0;
  for (const auto& p : map_points) {
    if (accepted(p) && accepted_index++ % stride == 0) filtered.push_back(p);
  }
  auto candidates = clusters(filtered);
  auto reidentified = reidentify(candidates, stamp);
  const double elapsed = have_body_ ? stamp - body_stamp_ : 0;
  std::vector<lidar_perception_system::SupportCandidate> hypotheses;
  for (const auto& c : candidates) {
    hypotheses.push_back({c.center, static_cast<unsigned int>(c.body.size()), 0});
  }
  const std::optional<Eigen::Vector2d> acquisition_hint =
      have_body_ ? std::optional<Eigen::Vector2d>(cluster_center_ + body_velocity_ * elapsed)
                 : std::nullopt;
  const auto motion_hint = acquisition_.observe_candidates(hypotheses, stamp, acquisition_hint);
  result.candidates = static_cast<int>(candidates.size());
  std::vector<Cluster> recovery_candidates;
  if (have_body_ && !reference_body_.empty()) {
    recovery_candidates = std::move(candidates);
    // Sparse scans can split a robot into several connected components. Once
    // acquired, collect its predicted footprint rather than following whichever
    // fragment happens to have the nearest changing centroid.
    Pose predicted = pose_;
    predicted.xy += body_velocity_ * elapsed;
    predicted.yaw = angle(predicted.yaw + yaw_rate_ * elapsed);
    Eigen::Vector2d lo = reference_body_.front().head<2>();
    Eigen::Vector2d hi = lo;
    for (const auto& p : reference_body_) {
      if (p.z() > cfg_.body_max_z) continue;
      lo = lo.cwiseMin(p.head<2>());
      hi = hi.cwiseMax(p.head<2>());
    }
    const double margin = std::min(0.30, 0.10 + 0.20 * elapsed);
    Cluster tracked;
    size_t lower_count = 0;
    tracked.center.setZero();
    tracked.min_x = tracked.min_y = std::numeric_limits<double>::infinity();
    tracked.max_x = tracked.max_y = -tracked.min_x;
    for (const auto& p : filtered) {
      const Point local = predicted.local(p);
      const Point last_local = pose_.local(p);
      const auto inside = [&](const Point& v) {
        return (v.head<2>().array() >= lo.array() - margin).all() &&
               (v.head<2>().array() <= hi.array() + margin).all();
      };
      if (p.z() < cfg_.body_min_z || p.z() > cfg_.tracking_body_max_z ||
          (!inside(local) && !inside(last_local))) {
        continue;
      }
      tracked.body.push_back(p);
      if (p.z() <= cfg_.body_max_z) {
        ++lower_count;
        tracked.center += p.head<2>();
        tracked.min_x = std::min(tracked.min_x, p.x());
        tracked.max_x = std::max(tracked.max_x, p.x());
        tracked.min_y = std::min(tracked.min_y, p.y());
        tracked.max_y = std::max(tracked.max_y, p.y());
      }
    }
    candidates.clear();
    if (static_cast<int>(tracked.body.size()) >= cfg_.min_body_points && lower_count >= 3 &&
        (lower_count >= static_cast<size_t>(cfg_.min_body_points) || !reference_upper_.empty())) {
      tracked.center /= lower_count;
      Points lower, upper;
      for (const auto& p : tracked.body) {
        if (p.z() <= cfg_.body_max_z)
          lower.push_back(p);
        else
          upper.push_back(p);
      }
      tracked.body = voxelize(lower, cfg_.body_voxel, cfg_.max_body_points);
      auto upper_returns =
          voxelize(upper, cfg_.body_voxel, std::min(500, cfg_.max_body_points / 4));
      tracked.body.insert(tracked.body.end(), upper_returns.begin(), upper_returns.end());
      candidates.push_back(std::move(tracked));
    }
  }
  const Eigen::Vector2d hint = cluster_center_ + body_velocity_ * elapsed;
  Cluster* chosen = nullptr;
  double best = -1e9, second = -1e9;
  int best_evidence = -1, eligible = 0;
  bool chosen_motion = false;
  for (auto& c : candidates) {
    bool excluded = false;
    if (!have_body_) {
      if (rejected_center_ && stamp < rejection_until_ &&
          (c.center - *rejected_center_).norm() < 0.4) {
        continue;
      }
      for (const auto& mask : exclusions_) {
        if ((c.center - mask.head<2>()).norm() < mask.z()) {
          excluded = true;
          break;
        }
      }
    }
    if (excluded) {
      continue;
    }
    ++eligible;
    int upper = 0;
    Points upper_context;
    for (const auto& p : filtered) {
      if (p.z() >= cfg_.min_opening_z - cfg_.bottom.bucket_height - cfg_.bottom.height_tolerance &&
          p.x() > c.min_x - cfg_.crop_margin && p.x() < c.max_x + cfg_.crop_margin &&
          p.y() > c.min_y - cfg_.crop_margin && p.y() < c.max_y + cfg_.crop_margin) {
        if (p.z() >= cfg_.min_opening_z) ++upper;
        if (!have_body_) upper_context.push_back(p);
      }
    }
    double score =
        have_body_ ? -(c.center - hint).norm() : std::log1p(c.body.size()) + std::log1p(upper);
    const bool moving = motion_hint && (c.center - motion_hint->xy).norm() < 0.05;
    int evidence = 0;
    if (!have_body_) {
      if (moving) evidence = 1;
      if (const auto opening = fit_opening(upper_context, upper_context)) {
        evidence = 3;
        score += opening->score;
      } else if (const auto bottom = fit_bucket_bottom(upper_context, upper_context, cfg_)) {
        evidence = 2;
        score += bottom->score;
      }
    }
    if (evidence > best_evidence || (evidence == best_evidence && score > best)) {
      second = evidence == best_evidence ? best : -1e9;
      best = score;
      best_evidence = evidence;
      chosen = &c;
      chosen_motion = moving;
    } else if (evidence == best_evidence) {
      second = std::max(second, score);
    }
  }
  // A fast move can leave the old footprint before a velocity is learned.
  // Retain field/size-filtered clusters for short, model-verified recovery.
  if (!chosen && have_body_ && elapsed <= cfg_.body_recovery_s) {
    double nearest = cfg_.association_m + .5 * elapsed;
    for (auto& c : recovery_candidates) {
      const double distance = (c.center - hint).norm();
      if (distance < nearest) {
        nearest = distance;
        chosen = &c;
      }
    }
  }
  if (reidentified) chosen = &reidentified->cluster;
  if (!chosen) {
    if (!confirmed_) {
      hits_ = 0;
    }
    return missing(stamp, "no robot cluster", result);
  }
  if (!reidentified && !have_body_ &&
      ((eligible > 1 && best_evidence == 0) || second > best - 0.5)) {
    return missing(stamp, "ambiguous robot clusters", result);
  }
  Pose updated;
  Points supported_body;
  if (reidentified) {
    const auto recovered = *lost_model_;
    clear_model();
    input_stamp_ = stamp;
    reference_body_ = recovered.body;
    learned_bottom_ = recovered.bottom;
    learned_bottom_stamp_ = recovered.bottom_stamp;
    pose_ = updated = reidentified->pose;
    have_body_ = identified_ = true;
    acquired_stamp_ = stamp;
    track_id_ = recovered.id;
    result.registration_rmse = reidentified->rmse;
    result.overlap = reidentified->overlap;
    result.body_reidentified = true;
    lost_model_.reset();
  } else if (!have_body_) {
    pose_.xy = chosen->center;
    pose_.yaw = 0;
    updated = pose_;
    have_body_ = true;
    motion_seeded_ = chosen_motion;
    acquired_stamp_ = stamp;
    track_id_ = ++next_id_;
    result.overlap = 1;
  } else {
    Pose initial = pose_;
    initial.xy += body_velocity_ * elapsed;
    initial.yaw = angle(initial.yaw + yaw_rate_ * elapsed);
    bool registered = register_body(chosen->body, initial, updated, result.registration_rmse,
                                    result.overlap, &supported_body);
    // At a wall the chassis may stop or turn abruptly. A velocity seed alone
    // can slip one repeating chassis feature ahead of the current returns.
    if ((initial.xy - pose_.xy).norm() > cfg_.body_voxel * .2 ||
        std::abs(angle(initial.yaw - pose_.yaw)) > .01) {
      Pose fitted;
      Points support;
      double rmse = 0, overlap = 0;
      if (register_body(chosen->body, pose_, fitted, rmse, overlap, &support) &&
          (!registered ||
           rmse + cfg_.max_registration_rmse * (1 - overlap) + .0001 <
               result.registration_rmse + cfg_.max_registration_rmse * (1 - result.overlap))) {
        registered = true;
        updated = fitted;
        result.registration_rmse = rmse;
        result.overlap = overlap;
        supported_body = std::move(support);
      }
    }
    // Repeated chassis structure can trap nearest-neighbor ICP one grid period
    // away. Also try centering the two voxelized bodies before registration.
    // The centroid is only a bounded seed; never use it as the final pose.
    Point reference_center = Point::Zero(), observed_center = Point::Zero();
    Points lower_reference;
    size_t observed_lower = 0;
    for (const auto& p : reference_body_)
      if (p.z() <= cfg_.body_max_z) {
        reference_center += p;
        lower_reference.push_back(p);
      }
    for (const auto& p : chosen->body)
      if (p.z() <= cfg_.body_max_z) {
        observed_center += p;
        ++observed_lower;
      }
    reference_center /= std::max(size_t{1}, lower_reference.size());
    observed_center /= std::max(size_t{1}, observed_lower);
    const Eigen::Vector2d correction =
        (observed_center - initial.world(reference_center)).head<2>();
    if (correction.norm() > cfg_.body_voxel * .2 && correction.norm() < cfg_.correspondence_m) {
      Pose alternate = initial, fitted;
      alternate.xy += correction;
      double rmse = 0, overlap = 0;
      Points alternate_support;
      if (register_body(chosen->body, alternate, fitted, rmse, overlap, &alternate_support) &&
          (!registered ||
           (rmse + cfg_.max_registration_rmse * (1 - overlap) + .0001 <
            result.registration_rmse + cfg_.max_registration_rmse * (1 - result.overlap)))) {
        registered = true;
        updated = fitted;
        result.registration_rmse = rmse;
        result.overlap = overlap;
        supported_body = std::move(alternate_support);
      }
    }
    if (observed_lower < static_cast<size_t>(cfg_.min_body_points)) registered = false;
    if ((!registered ||
         (!result.body_upper_assisted && supported_body.size() < .7 * lower_reference.size())) &&
        elapsed <= cfg_.body_recovery_s) {
      // Never reacquire from a centroid alone: each seed must pass the same
      // unique correspondences, residual, overlap and 2D-shape checks as ICP.
      std::vector<Cluster*> ranked;
      for (auto& candidate : recovery_candidates) ranked.push_back(&candidate);
      std::sort(ranked.begin(), ranked.end(), [&](const Cluster* a, const Cluster* b) {
        return (a->center - initial.world(reference_center).head<2>()).squaredNorm() <
               (b->center - initial.world(reference_center).head<2>()).squaredNorm();
      });
      Cluster* recovered = nullptr;
      const auto quality = [&](double rmse, double overlap, size_t support) {
        const double coverage = static_cast<double>(support) / lower_reference.size();
        return rmse + cfg_.max_registration_rmse * (2 - overlap - coverage);
      };
      bool have_solution = registered;
      double recovery_score =
          registered ? quality(result.registration_rmse, result.overlap, supported_body.size())
                     : std::numeric_limits<double>::infinity();
      bool ambiguous = false;
      const size_t limit = std::min(size_t{3}, ranked.size());
      const auto principal_heading = [](const Points& points) -> std::optional<double> {
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (const auto& p : points) mean += p.head<2>();
        mean /= points.size();
        Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
        for (const auto& p : points) {
          const Eigen::Vector2d delta = p.head<2>() - mean;
          covariance += delta * delta.transpose();
        }
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(covariance);
        if (eigen.info() != Eigen::Success ||
            eigen.eigenvalues()[1] < 1.2 * std::max(1e-6, eigen.eigenvalues()[0]))
          return std::nullopt;
        const auto direction = eigen.eigenvectors().col(1);
        return std::atan2(direction.y(), direction.x());
      };
      const auto model_heading = principal_heading(lower_reference);
      Eigen::Vector2d model_lo = lower_reference.front().head<2>(), model_hi = model_lo;
      for (const auto& p : lower_reference) {
        model_lo = model_lo.cwiseMin(p.head<2>());
        model_hi = model_hi.cwiseMax(p.head<2>());
      }
      for (size_t i = 0; i < limit; ++i) {
        auto& c = *ranked[i];
        if ((c.center - initial.world(reference_center).head<2>()).norm() >
            cfg_.association_m + .5 * elapsed)
          continue;
        std::vector<double> turns{0., -.35, .35};
        const auto observed_heading = principal_heading(c.body);
        if (model_heading && observed_heading) {
          const double turn = std::remainder(*observed_heading - *model_heading - initial.yaw, pi);
          if (std::abs(turn) < .85) turns.insert(turns.begin(), turn);
        } else {
          turns.push_back(-.175);
          turns.push_back(.175);
        }
        for (double turn : turns) {
          Pose seed = initial, fitted;
          seed.yaw = angle(seed.yaw + turn);
          seed.xy += c.center - seed.world(reference_center).head<2>();
          Points support;
          double rmse = 0, overlap = 0;
          ++result.recovery_attempts;
          if (!register_body(c.body, seed, fitted, rmse, overlap, &support) ||
              !body_size_valid(support) ||
              (fitted.xy - initial.xy).norm() > cfg_.association_m + .5 * elapsed)
            continue;
          if (registered) {
            Eigen::Vector2d lo = fitted.local(c.body.front()).head<2>(), hi = lo;
            for (const auto& p : c.body) {
              const auto local = fitted.local(p).head<2>().eval();
              lo = lo.cwiseMin(local);
              hi = hi.cwiseMax(local);
            }
            // A partial visible fragment must not displace an already valid
            // local match merely by being centered on the full model.
            if (((hi - lo).array() < .8 * (model_hi - model_lo).array()).any()) continue;
          }
          const double score = quality(rmse, overlap, support.size());
          const bool different = have_solution && ((fitted.xy - updated.xy).norm() > .15 ||
                                                   std::abs(angle(fitted.yaw - updated.yaw)) > .25);
          if (different && std::abs(score - recovery_score) < .005) ambiguous = true;
          if (score + .0001 < recovery_score) {
            if (score + .005 < recovery_score) ambiguous = false;
            recovery_score = score;
            have_solution = true;
            recovered = &c;
            updated = fitted;
            result.registration_rmse = rmse;
            result.overlap = overlap;
            supported_body = std::move(support);
            if (rmse < .003 && overlap > .98 &&
                supported_body.size() > .98 * lower_reference.size())
              break;
          }
        }
      }
      registered = have_solution && !ambiguous;
      if (recovered && registered) {
        chosen = recovered;
        result.body_recovered = true;
        result.body_upper_assisted = false;
      }
    }
    if (!registered && !reference_upper_.empty()) {
      // Upper features supplement an established model only after lower-only
      // ICP fails. Their movement must never spoil a good lower-body match.
      std::vector<Pose> upper_seeds{initial, pose_};
      Point observed_upper = Point::Zero(), model_upper = Point::Zero();
      size_t upper_count = 0;
      for (const auto& p : chosen->body)
        if (p.z() > cfg_.body_max_z) {
          observed_upper += p;
          ++upper_count;
        }
      for (const auto& p : reference_upper_) model_upper += p;
      if (upper_count >= 12 && !reference_upper_.empty()) {
        observed_upper /= upper_count;
        model_upper /= reference_upper_.size();
        Pose centered = initial;
        const Eigen::Vector2d correction = (observed_upper - centered.world(model_upper)).head<2>();
        if (correction.norm() < cfg_.association_m) {
          centered.xy += correction;
          upper_seeds.push_back(centered);
        }
      }
      for (const Pose& seed : upper_seeds) {
        Pose fitted;
        Points support;
        double rmse = 0, overlap = 0;
        if (!register_body(chosen->body, seed, fitted, rmse, overlap, &support, true) ||
            !body_size_valid(support) ||
            std::count_if(support.begin(), support.end(),
                          [&](const Point& p) { return p.z() > cfg_.body_max_z; }) < 6)
          continue;
        if (!registered ||
            rmse + cfg_.max_registration_rmse * (1 - overlap) <
                result.registration_rmse + cfg_.max_registration_rmse * (1 - result.overlap)) {
          registered = true;
          updated = fitted;
          result.registration_rmse = rmse;
          result.overlap = overlap;
          supported_body = std::move(support);
          result.body_upper_assisted = true;
        }
      }
    }
    if (!registered) {
      if (!confirmed_) {
        hits_ = 0;
      }
      return missing(stamp, "robot registration rejected", result);
    }
    chosen->body = std::move(supported_body);
    if (!body_size_valid(chosen->body))
      return missing(stamp, "robot body exceeds size envelope", result);
    if (elapsed > 0) {
      const auto v = (updated.xy - pose_.xy) / elapsed;
      body_velocity_ = 0.5 * body_velocity_ + 0.5 * v;
      yaw_rate_ = 0.5 * yaw_rate_ + 0.5 * angle(updated.yaw - pose_.yaw) / elapsed;
    }
  }
  pose_ = updated;
  motion_seeded_ = motion_seeded_ || chosen_motion;
  body_stamp_ = stamp;
  cluster_center_ = pose_.xy;
  result.body_valid = true;
  selected_.clear();
  for (const auto& p : chosen->body)
    if (p.z() <= cfg_.body_max_z) selected_.push_back(p);
  Point lo = Point::Constant(std::numeric_limits<double>::infinity()), hi = -lo;
  const auto& footprint = reference_body_.empty() ? chosen->body : reference_body_;
  for (const auto& p : footprint) {
    if (p.z() > cfg_.body_max_z) continue;
    const Point local = reference_body_.empty() ? pose_.local(p) : p;
    lo = lo.cwiseMin(local);
    hi = hi.cwiseMax(local);
  }
  for (const auto& p : filtered) {
    if (p.z() >= cfg_.body_min_z && p.z() <= cfg_.body_max_z) continue;
    const Point local = pose_.local(p);
    if ((local.head<2>().array() >= lo.head<2>().array() - cfg_.crop_margin).all() &&
        (local.head<2>().array() <= hi.head<2>().array() + cfg_.crop_margin).all())
      selected_.push_back(p);
  }
  Points upper;
  for (const auto& p : selected_) {
    if (p.z() >= cfg_.min_opening_z - cfg_.bottom.bucket_height - cfg_.bottom.height_tolerance) {
      upper.push_back(pose_.local(p));
    }
  }
  upper = voxelize(upper, cfg_.opening_voxel, cfg_.max_model_points);
  auto instantaneous = fit_opening(upper, upper);
  if (instantaneous && last_opening_ &&
      std::abs(instantaneous->center.z() - last_opening_->center.z()) > cfg_.height_change_m) {
    for (auto& frame : frames_) {
      frame.upper.clear();
    }
    confirmed_ = false;
    hits_ = 0;
    height_pending_ = true;
    acquired_stamp_ = stamp;
    last_opening_.reset();
    ++height_resets_;
  }
  Frame frame;
  frame.stamp = stamp;
  frame.body = local_points(chosen->body, pose_);
  // Learn upper rigid features only while the lower chassis independently
  // supports the pose. Never grow a new upper model from upper-only alignment.
  const auto lower_support =
      std::count_if(chosen->body.begin(), chosen->body.end(),
                    [&](const Point& p) { return p.z() <= cfg_.body_max_z; });
  result.body_upper_assisted = result.body_upper_assisted || lower_support < cfg_.min_body_points;
  if (!result.body_upper_assisted && lower_support >= cfg_.min_body_points) {
    for (const auto& p : filtered) {
      if (p.z() <= cfg_.body_max_z || p.z() > cfg_.tracking_body_max_z) continue;
      const Point local = pose_.local(p);
      if ((local.head<2>().array() < lo.head<2>().array() - .10).any() ||
          (local.head<2>().array() > hi.head<2>().array() + .10).any())
        continue;
      // A learned bucket may lift independently of the chassis. Keep its
      // neighborhood out of the rigid registration reference.
      if (learned_bottom_ &&
          (local.head<2>() - learned_bottom_->center.head<2>()).norm() < cfg_.max_radius + .04)
        continue;
      frame.body.push_back(local);
    }
  }
  frame.upper = upper;
  frames_.push_back(std::move(frame));
  // At most 100 accepted scans per second; protect memory under pathological
  // timestamp rates.
  while (frames_.size() > 100) {
    frames_.pop_front();
  }
  rebuild(stamp);
  const auto detect_bottom = [&]() {
    auto measured = fit_bucket_bottom(accumulated_, upper, cfg_);
    if (measured) return std::make_pair(measured, false);
    return std::make_pair(fit_bucket_shape(accumulated_, upper, cfg_), true);
  };
  auto [bottom, shape_fitted] = detect_bottom();
  if (bottom && !cfg_.inside_field(pose_.world(bottom->center))) bottom.reset();
  if (bottom && last_bottom_ &&
      std::abs(bottom->center.z() - last_bottom_->center.z()) > cfg_.height_change_m) {
    // The bucket may lift relative to the chassis. Do not blend its old and new
    // heights or keep an old opening observation alive through the lift.
    for (auto& previous : frames_) previous.upper.clear();
    frames_.back().upper = upper;
    rebuild(stamp);
    std::tie(bottom, shape_fitted) = detect_bottom();
    if (bottom && !cfg_.inside_field(pose_.world(bottom->center))) bottom.reset();
    bottom_hits_ = 0;
    last_bottom_.reset();
    confirmed_ = false;
    hits_ = 0;
    last_opening_.reset();
    ++height_resets_;
  }
  if (bottom) {
    const bool consistent =
        last_bottom_ && stamp - bottom_stamp_ <= cfg_.prediction_s &&
        (bottom->center.head<2>() - last_bottom_->center.head<2>()).norm() < .08 &&
        std::abs(bottom->center.z() - last_bottom_->center.z()) <= cfg_.height_change_m;
    bottom_hits_ = consistent ? std::min(cfg_.confirm_hits, bottom_hits_ + 1) : 1;
    last_bottom_ = bottom;
    bottom_stamp_ = stamp;
    result.bottom = bottom;
    result.bottom_shape_fitted = shape_fitted;
    bottom_pending_shape_ = shape_fitted;
    result.bottom_valid = bottom_hits_ >= cfg_.confirm_hits;
    if (result.bottom_valid) {
      identified_ = true;
      learned_bottom_ = bottom;
      learned_bottom_stamp_ = stamp;
      result.bottom_tracking_mode = shape_fitted ? BottomMode::ShapeFitted : BottomMode::Direct;
    }
  } else {
    // Sparse scans may provide side evidence between two floor observations.
    // Retain only the unconfirmed hit count for the normal freshness interval;
    // no recognized bottom or new observation stamp is emitted during a miss.
    if (!bottom_pending_shape_ || stamp - bottom_stamp_ > cfg_.prediction_s) bottom_hits_ = 0;
  }
  result.bottom_observation_stamp = bottom_stamp_;
  if (!result.bottom_valid && learned_bottom_) {
    const double age = stamp - learned_bottom_stamp_;
    // Never revive an expired semantic anchor from an arbitrary nearby plane.
    if (age < 0 ||
        (cfg_.bottom.max_body_inference_s > 0 && age > cfg_.bottom.max_body_inference_s)) {
      learned_bottom_.reset();
    } else if (auto plane = track_bucket_bottom(*learned_bottom_, upper, cfg_)) {
      learned_bottom_ = plane;
      learned_bottom_stamp_ = stamp;
      result.bottom_tracking_mode = BottomMode::PlaneTracked;
    } else if (auto shape = track_bucket_shape(*learned_bottom_, upper, cfg_)) {
      learned_bottom_ = shape;
      learned_bottom_stamp_ = stamp;
      result.bottom_tracking_mode = BottomMode::ShapeFitted;
    } else if (!bottom ||
               (bottom->center - learned_bottom_->center).norm() <= cfg_.height_change_m) {
      result.bottom_tracking_mode = BottomMode::BodyInferred;
    }
    // An unconfirmed contradictory candidate suppresses this frame, but must
    // not erase a recognized mount. Replace the anchor only after confirmation.
  }
  if (learned_bottom_ && result.bottom_tracking_mode != BottomMode::None &&
      cfg_.inside_field(pose_.world(learned_bottom_->center))) {
    result.tracked_bottom = learned_bottom_;
    result.bottom_tracking_observation_stamp = learned_bottom_stamp_;
    if (result.bottom_tracking_mode == BottomMode::BodyInferred) {
      result.tracked_bottom->current_points = 0;
      result.tracked_bottom->side_points = 0;
    }
  }
  auto opening = fit_opening(accumulated_, upper, &result.fit);
  if (opening && !cfg_.inside_field(pose_.world(opening->center))) opening.reset();
  if (opening && last_opening_ &&
      std::abs(opening->center.z() - last_opening_->center.z()) > cfg_.height_change_m) {
    for (auto& previous : frames_) {
      previous.upper.clear();
    }
    frames_.back().upper = upper;
    rebuild(stamp);
    opening = fit_opening(accumulated_, upper, &result.fit);
    if (opening && !cfg_.inside_field(pose_.world(opening->center))) opening.reset();
    confirmed_ = false;
    hits_ = 0;
    height_pending_ = true;
    acquired_stamp_ = stamp;
    last_opening_.reset();
    result.tracked_bottom.reset();
    result.bottom_tracking_mode = BottomMode::None;
    ++height_resets_;
  }
  if (!opening) {
    if (!confirmed_) {
      hits_ = 0;
    }
    return missing(stamp, "no fresh opening edge", result);
  }
  if (last_opening_ &&
      (opening->center.head<2>() - last_opening_->center.head<2>()).norm() > 0.15) {
    return missing(stamp, "opening association rejected", result);
  }
  const Point measurement = pose_.world(opening->center);
  const double dt = stamp - observation_stamp_;
  if (confirmed_ && dt > 0 && dt < cfg_.reset_s) {
    const Point v = (measurement - measured_) / dt;
    if (v.head<2>().norm() > 5.0) {
      return missing(stamp, "opening velocity rejected", result);
    }
    velocity_ = 0.5 * velocity_ + 0.5 * v;
    velocity_.z() = 0;
    position_ = 0.6 * measurement + 0.4 * (position_ + velocity_ * dt);
    position_.z() = measurement.z();
  } else {
    position_ = measurement;
    velocity_.setZero();
  }
  measured_ = measurement;
  observation_stamp_ = stamp;
  last_opening_ = opening;
  hits_ = std::min(cfg_.confirm_hits, hits_ + 1);
  confirmed_ = hits_ >= cfg_.confirm_hits;
  if (confirmed_) {
    identified_ = true;
    height_pending_ = false;
  }
  variance_ =
      std::max(0.0004, std::pow(opening->residual, 2) + std::pow(result.registration_rmse, 2));
  result = missing(stamp, confirmed_ ? "opening observed" : "confirming opening", result);
  result.valid = confirmed_;
  result.predicted = false;
  result.position = position_;
  result.velocity = velocity_;
  result.opening = opening;
  result.position_variance = variance_;
  result.confidence =
      std::clamp(opening->arc / (2 * pi) * (1 - opening->residual / cfg_.max_residual), 0.0, 1.0);
  if (!cfg_.inside_field(result.position)) {
    result.valid = false;
    result.reason = "opening outside field";
  }
  return result;
}
}  // namespace lidar_perception_system::opening
