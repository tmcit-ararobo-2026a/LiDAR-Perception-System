// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_map>
#include <vector>

namespace lidar_perception_system::opening {
// Follows the bucket axis in the map frame from every scan, without the pose of the chassis. A
// partial view leaves the heading of the chassis free by tens of degrees while the robot moves, and
// the mount turns with it. Nothing on the robot reaches the top of the bucket, so what hits that
// strip near the robot is the bucket itself, however few returns there are.
//
// Those returns locate the bucket well but its speed poorly, since one stroke of the scan pattern
// only says how far the wall is along that line. The speed comes from the chassis instead: its
// newest returns are matched to those just before them, which needs no heading and no model.
class BucketAxisFilter {
 public:
  using Point = Eigen::Vector3d;
  // Height range of the chassis returns, and the bucket's height and its radii at the rim and floor.
  BucketAxisFilter(double chassis_min_z, double chassis_max_z, double bucket_height, double rim_radius,
                   double floor_radius, double strip_m)
      : chassis_min_z_(chassis_min_z), chassis_max_z_(chassis_max_z), bucket_height_(bucket_height),
        rim_radius_(rim_radius), floor_radius_(floor_radius), strip_m_(strip_m) {}
  bool ready() const { return ready_; }
  void clear() {
    ready_ = false;
    scans_.clear();
    recent_.clear();
  }
  // Start from where the chassis pose puts the bucket.
  void reset(const Eigen::Vector2d& axis, double stamp) {
    clear();
    ready_ = true;
    state_ << axis, 0, 0;
    covariance_ = Eigen::Vector4d(start_sigma_m * start_sigma_m, start_sigma_m * start_sigma_m,
                                  start_speed_sigma_mps * start_speed_sigma_mps,
                                  start_speed_sigma_mps * start_speed_sigma_mps)
                      .asDiagonal();
    stamp_ = last_hit_ = stamp;
    along_ = -.5;
    strip_rate_ = 30;
  }
  // One scan in the map frame, with the rim height and where the chassis pose puts the bucket.
  void update(double stamp, const std::vector<Point>& points, const Eigen::Vector2d& sensor_xy, double rim,
              const std::optional<Eigen::Vector2d>& inferred) {
    if (!ready_) return;
    if (stamp < stamp_) {  // The clock went back, so old returns mean nothing.
      reset(inferred ? *inferred : Eigen::Vector2d(state_.head<2>()), stamp);
      return;
    }
    const double dt = stamp - stamp_;
    stamp_ = stamp;
    predict(dt);
    new_returns_.clear();
    keep_new_returns(points, stamp);
    collect_chassis(stamp);
    if (const auto velocity = chassis_velocity(stamp)) fuse_velocity(*velocity);
    follow_strip(sensor_xy, rim, dt, stamp);
    if (inferred && stamp - last_hit_ > stale_s) {
      fuse(0, inferred->x(), inferred_sigma_m);
      fuse(1, inferred->y(), inferred_sigma_m);
    }
    // Beyond any turn of the robot it is not following the bucket any more.
    if (inferred && !((state_.head<2>() - *inferred).norm() <= lost_m)) reset(*inferred, stamp);
  }
  Eigen::Vector2d axis() const { return state_.head<2>(); }
  Eigen::Vector2d velocity() const { return state_.tail<2>(); }
  // When a return last hit the top strip.
  double last_hit() const { return last_hit_; }
  // Move the axis to a better fix, keeping its speed.
  void place(const Eigen::Vector2d& axis) { state_.head<2>() = axis; }

 private:
  struct Scan {
    double stamp;
    std::vector<Point> points;
  };
  // Scatter of a start, of a return around the wall (at least, and per metre of range), and of a
  // return inside the circle. Only returns outside it are sure of the position, as a transparent
  // bucket also returns from its far wall and its inside.
  static constexpr double start_speed_sigma_mps = .5;
  static constexpr double start_sigma_m = .1, return_sigma_m = .03, return_sigma_per_m = .0023, inside_sigma_m = .15;
  // Unexplained acceleration of the bucket, and how far its axis may be from where the pose puts it.
  // A turning pose moves the mount by up to twice its lever.
  static constexpr double acceleration_mps2 = 6., lost_m = 1.5;
  // How far above the rim a return still counts, as the rim may lag a lift, and how far from the
  // circle: nothing else reaches this height near the robot.
  static constexpr double above_rim_m = .3, reach_m = .8;
  // With nothing on the strip for this long, the pose has a say, as a guess.
  static constexpr double stale_s = 1., inferred_sigma_m = .4;
  // The returns lie on the near side of the axis as seen from the sensor, or around it if the bucket
  // is transparent. Near-side returns also fit a circle on the sensor's side of them, and that is told
  // by their mean share of the radius along the line of sight, over this time.
  static constexpr double along_time_s = .4, along_limit = .1, along_gain = 1.;
  // Chassis matching: how far around the axis, the newest returns, those before them, how far apart a
  // match may be, and the fewest returns.
  static constexpr double chassis_reach_m = 1., newest_s = .1, before_s = .3, match_reach_m = .1;
  static constexpr int newest_min_returns = 15, before_min_returns = 40, match_min_pairs = 10, match_iterations = 4;
  // Weight of a match falls off beyond this distance.
  static constexpr double match_soft_m = .03;
  static constexpr double velocity_sigma_mps = .3;
  // A chassis at least this thick across its visible face shows its motion in both directions.
  static constexpr double face_thickness_m = .1;
  // Scan windows may be republished, and a repeated return is an old position of a moving robot.
  static constexpr double repeat_s = .15, repeat_voxel_m = .01;

  static uint64_t voxel(const Point& p, double size) {
    const auto cell = [&](double v) { return static_cast<uint64_t>(std::llround(v / size) + (int64_t{1} << 20)); };
    return (cell(p.x()) << 42) | (cell(p.y()) << 21) | cell(p.z());
  }
  void keep_new_returns(const std::vector<Point>& points, double stamp) {
    for (auto it = recent_.begin(); it != recent_.end();)
      it = stamp - it->second > repeat_s ? recent_.erase(it) : std::next(it);
    for (const auto& p : points) {
      const auto [it, inserted] = recent_.try_emplace(voxel(p, repeat_voxel_m), stamp);
      if (inserted) new_returns_.push_back(p);
    }
  }
  void predict(double dt) {
    if (dt <= 0) return;
    Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
    transition(0, 2) = transition(1, 3) = dt;
    state_ = transition * state_;
    covariance_ = transition * covariance_ * transition.transpose();
    const double q = acceleration_mps2 * acceleration_mps2;
    const double pp = q * dt * dt * dt * dt / 4, pv = q * dt * dt * dt / 2, vv = q * dt * dt;
    for (int axis = 0; axis < 2; ++axis) {
      covariance_(axis, axis) += pp;
      covariance_(axis, axis + 2) += pv;
      covariance_(axis + 2, axis) += pv;
      covariance_(axis + 2, axis + 2) += vv;
    }
  }
  void fuse(int index, double value, double sigma) {
    const Eigen::Vector4d gain = covariance_.col(index) / (covariance_(index, index) + sigma * sigma);
    state_ += gain * (value - state_(index));
    covariance_ -= gain * covariance_.row(index);
  }
  // One flat face matches itself wherever it slides along, which only repeats the speed assumed
  // so far. Along a direction the returns barely vary across, the match says nothing.
  void fuse_velocity(const Eigen::Vector2d& velocity) {
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& r : newest_) mean += r.p.head<2>();
    mean /= static_cast<double>(newest_.size());
    Eigen::Matrix2d spread = Eigen::Matrix2d::Zero();
    for (const auto& r : newest_) {
      const Eigen::Vector2d d = r.p.head<2>() - mean;
      spread += d * d.transpose();
    }
    spread /= static_cast<double>(newest_.size());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(spread);
    if (eigen.info() != Eigen::Success) return;
    // The face runs along the larger spread, and only its thickness shows motion along it.
    const double thickness = std::sqrt(std::max(eigen.eigenvalues()[0], 0.));
    const double along_sigma = velocity_sigma_mps * std::max(1., face_thickness_m / std::max(thickness, 1e-3));
    Eigen::Matrix<double, 2, 4> h = Eigen::Matrix<double, 2, 4>::Zero();
    h.rightCols<2>() = eigen.eigenvectors().transpose();
    Eigen::Matrix2d noise = Eigen::Matrix2d::Zero();
    noise(0, 0) = velocity_sigma_mps * velocity_sigma_mps;
    noise(1, 1) = along_sigma * along_sigma;
    const Eigen::Matrix<double, 4, 2> covariance_h = covariance_ * h.transpose();
    const Eigen::Matrix2d innovation = h * covariance_h + noise;
    const Eigen::Matrix<double, 4, 2> gain = covariance_h * innovation.inverse();
    const Eigen::Vector2d residual = eigen.eigenvectors().transpose() * velocity - h * state_;
    state_ += gain * residual;
    covariance_ -= (gain * covariance_h.transpose()).eval();
    covariance_ = (.5 * (covariance_ + covariance_.transpose())).eval();
  }
  void collect_chassis(double stamp) {
    Scan scan{stamp, {}};
    for (const auto& p : new_returns_)
      if (p.z() >= chassis_min_z_ && p.z() <= chassis_max_z_ &&
          (p.head<2>() - state_.head<2>()).norm() <= chassis_reach_m)
        scan.points.push_back(p);
    scans_.push_back(std::move(scan));
    while (!scans_.empty() && stamp - scans_.front().stamp > newest_s + before_s) scans_.pop_front();
  }
  // The speed that brings the newest chassis returns onto those just before them. Each match adds a
  // velocity error times the difference of their ages, so the two groups need not hit the same spot.
  std::optional<Eigen::Vector2d> chassis_velocity(double stamp) {
    newest_.clear();
    before_.clear();
    for (const auto& scan : scans_)
      for (const auto& p : scan.points) (stamp - scan.stamp < newest_s ? newest_ : before_).push_back({p, stamp - scan.stamp});
    if (static_cast<int>(newest_.size()) < newest_min_returns || static_cast<int>(before_.size()) < before_min_returns)
      return std::nullopt;
    Eigen::Vector2d velocity = state_.tail<2>();
    bool matched = false;
    for (int iteration = 0; iteration < match_iterations; ++iteration) {
      build_grid(velocity);
      double weight_sum = 0;
      Eigen::Vector2d sum = Eigen::Vector2d::Zero();
      int pairs = 0;
      for (const auto& current : newest_) {
        Point at_now = current.p;
        at_now.head<2>() += velocity * current.age;
        double best = match_reach_m * match_reach_m;
        const Aged* nearest = nullptr;
        const CellIndex cell = cell_of(at_now);
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz) {
              const auto found = grid_.find(key_of({cell.x + dx, cell.y + dy, cell.z + dz}));
              if (found == grid_.end()) continue;
              for (const int index : found->second) {
                Point other = before_[index].p;
                other.head<2>() += velocity * before_[index].age;
                const double distance = (other - at_now).squaredNorm();
                if (distance < best) {
                  best = distance;
                  nearest = &before_[index];
                }
              }
            }
        if (!nearest) continue;
        Point other = nearest->p;
        other.head<2>() += velocity * nearest->age;
        const double weight = std::min(1., match_soft_m / std::max(std::sqrt(best), 1e-3));
        const double age_difference = nearest->age - current.age;
        weight_sum += weight * age_difference * age_difference;
        sum += weight * age_difference * (other - at_now).head<2>();
        ++pairs;
      }
      if (pairs < match_min_pairs || weight_sum < 1e-6) break;
      velocity -= sum / weight_sum;
      matched = true;
    }
    if (!matched) return std::nullopt;
    return velocity;
  }
  struct Aged {
    Point p;
    double age;
  };
  struct CellIndex {
    int x, y, z;
  };
  CellIndex cell_of(const Point& p) const {
    return {static_cast<int>(std::floor(p.x() / match_reach_m)), static_cast<int>(std::floor(p.y() / match_reach_m)),
            static_cast<int>(std::floor(p.z() / match_reach_m))};
  }
  static int64_t key_of(const CellIndex& c) {
    return (static_cast<int64_t>(c.x + (1 << 20)) << 42) | (static_cast<int64_t>(c.y + (1 << 20)) << 21) |
           static_cast<int64_t>(c.z + (1 << 20));
  }
  void build_grid(const Eigen::Vector2d& velocity) {
    grid_.clear();
    for (size_t i = 0; i < before_.size(); ++i) {
      Point at_now = before_[i].p;
      at_now.head<2>() += velocity * before_[i].age;
      grid_[key_of(cell_of(at_now))].push_back(static_cast<int>(i));
    }
  }
  double radius_at(double below_rim) const {
    const double share = std::clamp(1 - below_rim / bucket_height_, 0., 1.);
    return floor_radius_ + (rim_radius_ - floor_radius_) * share;
  }
  void follow_strip(const Eigen::Vector2d& sensor_xy, double rim, double dt, double stamp) {
    strip_.clear();
    for (const auto& p : new_returns_)
      if (p.z() > rim - strip_m_ && p.z() < rim + above_rim_m &&
          (p.head<2>() - state_.head<2>()).norm() <= rim_radius_ + reach_m)
        strip_.push_back(p);
    if (dt > 0) {
      strip_rate_ += (1 - std::exp(-dt / rate_time_s)) * (static_cast<double>(strip_.size()) / dt - strip_rate_);
      strip_rate_ = std::max(strip_rate_, 1.);
    }
    if (strip_.empty()) return;
    last_hit_ = stamp;
    const double range = (state_.head<2>() - sensor_xy).norm();
    const double sigma = std::max(return_sigma_m, return_sigma_per_m * range);
    for (const auto& p : strip_) {
      const Eigen::Vector2d offset = p.head<2>() - state_.head<2>();
      const double distance = offset.norm();
      if (distance < 1e-3) continue;
      const double error = distance - radius_at(rim - p.z());
      const Eigen::Vector2d normal = offset / distance;
      const double s = error > 0 ? sigma : std::max(sigma, inside_sigma_m);
      Eigen::Vector4d h = Eigen::Vector4d::Zero();
      h.head<2>() = normal;
      const Eigen::Vector4d covariance_h = covariance_ * h;
      const double innovation = h.dot(covariance_h) + s * s;
      const Eigen::Vector4d gain = covariance_h / innovation;
      state_ += gain * error;
      covariance_ -= gain * covariance_h.transpose();
    }
    const Eigen::Vector2d to_axis = state_.head<2>() - sensor_xy;
    if (to_axis.norm() < 1e-6) return;
    const Eigen::Vector2d line = to_axis.normalized();
    const double weight = 1 - std::exp(-1. / (along_time_s * strip_rate_));
    for (const auto& p : strip_) {
      const double share = std::clamp((p.head<2>() - state_.head<2>()).dot(line) / rim_radius_, -1.5, 1.5);
      along_ += weight * (share - along_);
    }
    if (along_ > along_limit) state_.head<2>() += along_gain * (along_ - along_limit) * rim_radius_ * line;
  }
  static constexpr double rate_time_s = 1.;

  double chassis_min_z_, chassis_max_z_, bucket_height_, rim_radius_, floor_radius_, strip_m_;
  bool ready_{false};
  double stamp_{0}, last_hit_{0}, along_{-.5}, strip_rate_{30};
  Eigen::Vector4d state_{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d covariance_{Eigen::Matrix4d::Identity()};
  std::deque<Scan> scans_;
  std::unordered_map<uint64_t, double> recent_;
  std::unordered_map<int64_t, std::vector<int>> grid_;
  std::vector<Point> new_returns_, strip_;
  std::vector<Aged> newest_, before_;
};
}  // namespace lidar_perception_system::opening
