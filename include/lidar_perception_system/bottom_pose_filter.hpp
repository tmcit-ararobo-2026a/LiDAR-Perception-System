// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <utility>
#include <stdexcept>

namespace lidar_perception_system::opening {
// Constant-velocity alpha-beta filter for the published bottom pose. Geometry
// fitting and its observation timestamps remain independent of this state.
class BottomPoseFilter {
 public:
  explicit BottomPoseFilter(double time_constant = .15, double max_acceleration = 6.,
                            double stationary_radius = .025)
      : tau_(time_constant), max_acceleration_(max_acceleration),
        stationary_radius_(stationary_radius) {
    if (!std::isfinite(tau_) || tau_ < 0 || tau_ > .5)
      throw std::invalid_argument("bottom.output_smoothing_s must be in [0, 0.5]");
    if (!std::isfinite(max_acceleration_) || max_acceleration_ <= 0 || max_acceleration_ > 30)
      throw std::invalid_argument("bottom.output_max_acceleration_mps2 must be in (0, 30]");
    if (!std::isfinite(stationary_radius_) || stationary_radius_ < 0 || stationary_radius_ > .05)
      throw std::invalid_argument("bottom.output_stationary_radius_m must be in [0, 0.05]");
  }
  void reset() { ready_ = false; }
  void update(Eigen::Vector3d& center, Eigen::Vector3d& normal, double stamp,
              uint32_t id, double max_gap, bool confirmed_geometry = false,
              bool height_observed = true, bool xy_observed = true,
              bool height_tracked = false, bool xy_followed = false) {
    const double dt = stamp - stamp_;
    const double previous_height = center_.z();
    const Eigen::Vector2d previous_xy = center_.head<2>();
    Eigen::Vector3d prediction = center_ + velocity_ * dt;
    const bool height_changed = confirmed_geometry &&
                                std::abs(center.z() - previous_height) > .03;
    // Never interpolate across a different object, stale input or rewind.
    // A height change must not reset the independent horizontal state.
    // A large innovation alone may be an ICP/fit
    // outlier: it must not bypass the filter by forcing a reset.
    if (!ready_ || id != id_ || dt <= 0 || dt > max_gap || tau_ == 0) {
      center_ = center;
      normal_ = normal.normalized();
      velocity_.setZero();
      observations_.clear();
      height_observations_ = {center.z(), center.z()};
      output_xy_ = center.head<2>();
      output_velocity_.setZero();
      stationary_ = false;
      xy_observations_.clear();
      if (xy_observed) xy_observations_.emplace_back(stamp, center);
      inferred_motion_hits_ = 0;
      diverged_hits_ = 0;
    } else {
      observations_.emplace_back(stamp, center);
      while (observations_.size() > 100 ||
             (observations_.size() > 1 && stamp - observations_.front().first > 2.))
        observations_.pop_front();
      if (xy_observed) xy_observations_.emplace_back(stamp, center);
      while (!xy_observations_.empty() &&
             (xy_observations_.size() > 100 || stamp - xy_observations_.front().first > 3.))
        xy_observations_.pop_front();
      // Keep observed bucket motion separate from noisy chassis inference.
      // Mixing them makes a stationary bucket appear to move at each mode switch.
      const bool bucket_supported = xy_observations_.size() >= 3 &&
          stamp - xy_observations_.back().first <= 1.5 &&
          xy_observations_.back().first - xy_observations_.front().first >= .4;
      const auto motion = estimate_motion(bucket_supported ? xy_observations_ : observations_,
                                          stamp, bucket_supported ? 3. : 2.);
      const auto recent = estimate_motion(observations_, stamp, .5);
      if (stationary_) {
        inferred_motion_hits_ = !xy_observed &&
            (center.head<2>() - stationary_xy_).norm() > .15 ? inferred_motion_hits_ + 1 : 0;
        const bool observed_motion = bucket_supported && motion.count >= 3 &&
            ((motion.mean.head<2>() - stationary_xy_).norm() > stationary_radius_ ||
             motion.velocity.head<2>().norm() > .12);
        const bool current_observed_motion = xy_observed &&
            (center.head<2>() - stationary_xy_).norm() > stationary_radius_;
        const bool inferred_motion = !bucket_supported && motion.count >= 3 &&
            motion.velocity.head<2>().norm() > .2 &&
            (motion.mean.head<2>() - stationary_xy_).norm() > .08;
        if (observed_motion || current_observed_motion || inferred_motion || inferred_motion_hits_ >= 3)
          stationary_ = false;
      }
      if (!stationary_ && stationary_radius_ > 0 && motion.count >= 3 &&
          motion.span >= (bucket_supported ? .4 : 1.5) &&
          motion.velocity.head<2>().norm() < .03 &&
          motion.spread < (bucket_supported ? 2. : 8.) * stationary_radius_) {
        stationary_ = true;
        stationary_xy_ = motion.mean.head<2>();
      }
      double tau = tau_;
      if (recent.count >= 4 && recent.span >= .3 &&
          recent.velocity.head<2>().norm() < .12 && std::abs(recent.velocity.z()) < .03) {
        tau = std::max(tau, .5);
        velocity_.setZero();
        prediction = center_;
        prediction.head<2>() = recent.mean.head<2>();
      }
      const bool hold_xy = stationary_;
      const bool corroborated_motion = !hold_xy && recent.count >= 4 && recent.span >= .3 &&
          motion.count >= 3 && recent.velocity.head<2>().norm() > .12 &&
          motion.velocity.head<2>().norm() > .12 &&
          recent.velocity.head<2>().dot(motion.velocity.head<2>()) >
              .8 * recent.velocity.head<2>().norm() * motion.velocity.head<2>().norm();
      if (!xy_observed && corroborated_motion) {
        // Coherent short/long-window body motion is useful velocity evidence,
        // even when no bucket surface is visible. Do not infer velocity from
        // individual chassis registration jumps.
        velocity_.head<2>() = .5 * velocity_.head<2>() + .5 * recent.velocity.head<2>();
        prediction.head<2>() = center_.head<2>() + velocity_.head<2>() * dt;
      }
      if (hold_xy) {
        prediction.head<2>() = stationary_xy_;
        velocity_.head<2>().setZero();
      }
      const double decay = std::exp(-dt / tau);
      // Chassis-inferred positions have no fresh bucket XY evidence. Do not
      // give their registration noise the same weight as a measured frustum.
      const double weight = xy_observed || corroborated_motion ? 1. : .2;
      const double alpha = weight * (1 - decay * decay);
      const double beta = weight * (1 - decay) * (1 - decay);
      Eigen::Vector3d residual = center - prediction;
      // Bound a single registration/shape correction. Velocity prediction
      // carries actual translation; repeated evidence can still correct drift.
      const double xy = residual.head<2>().norm();
      // One outlier is bounded below, but a persistent gap means the filter state
      // is wrong: without this it could stay a metre behind a good measurement.
      diverged_hits_ = xy > diverged_residual_m ? diverged_hits_ + 1 : 0;
      if (diverged_hits_ >= diverged_scans) {
        diverged_hits_ = 0;
        center_.head<2>() = center.head<2>();
        output_xy_ = center.head<2>();
        velocity_.head<2>() = recent.count >= 4 ? Eigen::Vector2d(recent.velocity.head<2>())
                                                : Eigen::Vector2d(Eigen::Vector2d::Zero());
        output_velocity_ = velocity_.head<2>();
        stationary_ = false;
        center_.z() = height_changed || height_tracked ? center.z() : previous_height;
        ready_ = true;
        id_ = id;
        stamp_ = stamp;
        center.head<2>() = output_xy_;
        center.z() = center_.z();
        normal = normal_;
        return;
      }
      if (xy > .06) residual.head<2>() *= .06 / xy;
      if (hold_xy) residual.head<2>().setZero();
      residual.z() = std::clamp(residual.z(), -.015, .015);
      center_ = prediction + alpha * residual;
      velocity_ += beta / dt * residual;
      // Height is an independently measured quantity. Never extrapolate Z
      // from noise or update it from a body/side-only inferred position.
      const double height_alpha = 1 - std::exp(-dt / std::max(tau_, .3));
      double observed_height = previous_height;
      if (height_changed) {
        height_observations_ = {center.z(), center.z()};
        observed_height = center.z();
      } else if (height_observed) {
        height_observations_.push_back(center.z());
        while (height_observations_.size() > 5) height_observations_.pop_front();
        auto heights = height_observations_;
        std::sort(heights.begin(), heights.end());
        observed_height = heights[heights.size() / 2];
      }
      // A tracked height is already filtered, and limiting it here would only delay a lift.
      center_.z() = height_changed || height_tracked ? center.z() : height_observed
                        ? previous_height + height_alpha *
                              std::clamp(observed_height - previous_height, -.015, .015)
                        : previous_height;
      normal_ = ((1 - alpha) * normal_ + alpha * normal).normalized();
      // Alpha-beta position corrections need not match its velocity state.
      // Publish a critically damped trajectory instead of those instantaneous
      // corrections. Integrating bounded acceleration keeps the output's
      // position and velocity consistent, including at mode transitions.
      const Eigen::Vector2d target_velocity = .5 * (center_.head<2>() - previous_xy) / dt +
                                               .5 * velocity_.head<2>();
      const double omega = 2. / std::max(tau_, .1);
      const int steps = std::max(1, static_cast<int>(std::ceil(dt / .01)));
      const double step = dt / steps;
      for (int i = 0; i < steps; ++i) {
        const Eigen::Vector2d target = center_.head<2>() -
            target_velocity * (dt - (i + .5) * step);
        Eigen::Vector2d acceleration = omega * omega * (target - output_xy_) +
            2 * omega * (target_velocity - output_velocity_);
        const double norm = acceleration.norm();
        if (norm > max_acceleration_) acceleration *= max_acceleration_ / norm;
        output_xy_ += output_velocity_ * step + .5 * acceleration * step * step;
        output_velocity_ += acceleration * step;
      }
    }
    // A followed axis is already filtered, and smoothing it again would only delay it.
    if (xy_followed && !stationary_) {
      output_xy_ = center_.head<2>() = center.head<2>().eval();
      output_velocity_ = velocity_.head<2>();
    }
    ready_ = true;
    if (observations_.empty()) observations_.emplace_back(stamp, center);
    id_ = id;
    stamp_ = stamp;
    velocity_.z() = 0;
    center = center_;
    center.head<2>() = output_xy_;
    normal = normal_;
  }

 private:
  struct Motion {
    Eigen::Vector3d mean{0, 0, 0}, velocity{0, 0, 0};
    double span{0}, spread{0};
    size_t count{0};
  };
  static Motion estimate_motion(
      const std::deque<std::pair<double, Eigen::Vector3d>>& observations,
      double stamp, double window) {
    Motion result;
    double mean_t = 0, first = stamp, last = 0;
    for (const auto& observation : observations) {
      if (stamp - observation.first > window) continue;
      mean_t += observation.first - stamp;
      result.mean += observation.second;
      first = std::min(first, observation.first);
      last = std::max(last, observation.first);
      ++result.count;
    }
    if (!result.count) return result;
    mean_t /= result.count;
    result.mean /= result.count;
    result.span = last - first;
    double denominator = 0;
    for (const auto& observation : observations) {
      if (stamp - observation.first > window) continue;
      const double t = observation.first - stamp - mean_t;
      result.velocity += t * (observation.second - result.mean);
      denominator += t * t;
      result.spread = std::max(result.spread,
          (observation.second.head<2>() - result.mean.head<2>()).norm());
    }
    result.velocity /= std::max(denominator, 1e-9);
    return result;
  }
  static constexpr double diverged_residual_m = .15;
  static constexpr int diverged_scans = 3;
  int diverged_hits_{0};
  double tau_, max_acceleration_, stationary_radius_, stamp_{0};
  uint32_t id_{0};
  bool ready_{false};
  Eigen::Vector3d center_{0, 0, 0}, velocity_{0, 0, 0}, normal_{0, 0, 1};
  Eigen::Vector2d output_xy_{0, 0}, output_velocity_{0, 0};
  Eigen::Vector2d stationary_xy_{0, 0};
  bool stationary_{false};
  int inferred_motion_hits_{0};
  std::deque<std::pair<double, Eigen::Vector3d>> observations_;
  std::deque<std::pair<double, Eigen::Vector3d>> xy_observations_;
  std::deque<double> height_observations_;
};
}  // namespace lidar_perception_system::opening
