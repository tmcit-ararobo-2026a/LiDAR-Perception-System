// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace lidar_perception_system::opening {
// Follows the height of the bucket rim from every scan. Returns exist below the rim and not above
// it, which tells a lift from standing still without waiting for a window to fill:
// - Returns above the rim mean it rose, which is visible at once.
// - A rim that stops being hit while the rest of the robot still is has come down.
// While holding, a long window keeps the height steady. While lifting, a line fitted to a short
// window gives the present height and speed.
class RimHeightFilter {
 public:
  RimHeightFilter(double min_rim, double max_rim) : min_(min_rim), max_(max_rim) {
    if (!std::isfinite(min_rim) || !std::isfinite(max_rim) || max_rim <= min_rim)
      throw std::invalid_argument("Rim height range must be finite and ordered");
  }
  bool ready() const { return ready_; }
  void clear() {
    ready_ = lifting_ = false;
    returns_.clear();
    speed_ = 0;
    window_start_ = -std::numeric_limits<double>::infinity();
  }
  // Start from a rim seen by a fit.
  void reset(double rim, double stamp) {
    clear();
    ready_ = true;
    rim_ = std::clamp(rim, min_, max_);
    stamp_ = hold_start_ = last_hit_ = stamp;
    at_rim_rate_ = below_rim_rate_ = missed_ = 0;
  }
  // Absorb a direct measurement of the rim, such as a fitted floor plane.
  void measure(double rim) {
    if (ready_ && !lifting_ && std::abs(rim - rim_) < measure_reach_m)
      rim_ += measure_gain * (rim - rim_);
  }
  // One scan: heights of the new returns around the bucket axis, the expected scatter of a height,
  // and the highest height the sensor can see on this bearing.
  void update(double stamp, const std::vector<double>& heights, double sigma,
              std::optional<double> ceiling = std::nullopt) {
    if (!ready_) return;
    const double dt = std::max(stamp - stamp_, 0.);
    stamp_ = stamp;
    // Returns far below the rim are kept too, because the rim may turn out to be far too high.
    for (const double z : heights) returns_.push_back({stamp, z});
    while (!returns_.empty() && stamp - returns_.front().stamp > hold_window_s) returns_.pop_front();
    const double above_margin = std::max(min_margin_m, above_sigmas * sigma);
    int at_rim = 0, below_rim = 0;
    for (const double z : heights) {
      if (z >= rim_ - rim_band_m && z <= rim_ + above_margin)
        ++at_rim;
      else if (z < rim_ - rim_band_m)
        ++below_rim;
    }
    if (!lifting_ && dt > 0) {
      const double gain = 1 - std::exp(-dt / share_time_s);
      at_rim_rate_ += gain * (at_rim - at_rim_rate_);
      below_rim_rate_ += gain * (below_rim - below_rim_rate_);
    }
    missed_ += below_rim;
    if (at_rim > 0) {
      last_hit_ = stamp;
      missed_ = 0;
    }
    int above = 0;
    double first_above = stamp, last_above = 0;
    for (const auto& r : returns_)
      if (stamp - r.stamp <= rise_window_s && r.z > rim_ + above_margin) {
        ++above;
        first_above = std::min(first_above, r.stamp);
        last_above = std::max(last_above, r.stamp);
      }
    // One burst of returns can come from anything, so a rise needs them over some time.
    const bool rising = above >= rise_returns && last_above - first_above >= rise_span_s;
    const bool rim_in_view = !ceiling || rim_ + .03 < *ceiling;
    // A rim that came down stops being hit while the rest of the robot still is. How many of those
    // other returns it takes to be sure follows from the share of returns the rim normally gets,
    // which holds at any range and density: a silence alone would also fit a bucket that is simply
    // not being hit, as often happens far away while the chassis still returns.
    const double total = at_rim_rate_ + below_rim_rate_;
    const double share = total > 0 ? std::clamp(at_rim_rate_ / total, min_rim_share, max_rim_share)
                                   : max_rim_share;
    const bool silent = stamp - last_hit_ > min_quiet_s &&
                        (missed_ >= fall_confidence / share || stamp - last_hit_ > max_silence_s);
    if (rising) {
      start_lift(stamp, -std::numeric_limits<double>::infinity());
    } else if (rim_in_view && silent) {
      // What came before the silence was at the old height.
      start_lift(stamp, last_hit_);
      // The height the silence refers to moves with the estimate from now on.
      last_hit_ = stamp;
      missed_ = 0;
    }
    estimate(stamp, dt, above);
  }
  double rim() const { return rim_; }
  double speed() const { return speed_; }
  bool lifting() const { return lifting_; }

 private:
  struct Return {
    double stamp, z;
  };
  // Band below the rim where a return counts as a hit, and where it can mark the rim.
  static constexpr double rim_band_m = .06, top_band_m = .17;
  static constexpr double min_margin_m = .02, above_sigmas = 2.5;
  // This many returns above the rim within this time start a rise.
  static constexpr int rise_returns = 2;
  static constexpr double rise_window_s = 1., rise_span_s = .08;
  // The scan pattern hits an object in bursts, so a silence shorter than this means nothing.
  static constexpr double min_quiet_s = .4;
  // Expected rim returns that must have failed to appear. The share of the rim falls with range
  // faster than it can be measured on a moving robot, so this leaves a wide margin.
  static constexpr double fall_confidence = 6.;
  // Share of the returns that hit the rim: its limits, and how long it averages.
  static constexpr double min_rim_share = .03, max_rim_share = .5, share_time_s = 3.;
  // After this long a silent rim comes down even with few other returns.
  static constexpr double max_silence_s = 10.;
  static constexpr int fall_wall_returns = 2;
  // Windows of returns while holding and while lifting (lifting also covers the silence limit).
  static constexpr double hold_window_s = 3., min_lift_window_s = .8;
  static constexpr int hold_min_returns = 8, lift_min_returns = 6;
  // A held height moves with this time constant, towards a window top no further than this.
  static constexpr double hold_time_s = .5, hold_reach_m = .05;
  // A lift ends when it lasted this long and the rim is this slow.
  static constexpr double min_lift_s = .6, end_speed = .05;
  // Candidate speeds of the rim, the band of returns the line is fitted to, and its quantile.
  static constexpr double max_speed = 1.2, speed_step = .04, line_band_m = .35, rim_quantile = .9;
  // Allowance for a sparse estimate on top of what the fastest lift explains.
  static constexpr double jump_margin_m = .01;
  // How far a line may end from the returns, and how recent "just now" is.
  static constexpr double line_margin_m = .03, newest_s = .15;
  // Faster lines must explain the returns this much better, per m/s.
  static constexpr double speed_penalty = .2;
  // Share of the wall returns that lie above the estimate of the rim.
  static constexpr double rim_share = .1;
  static constexpr double measure_reach_m = .05, measure_gain = .3;
  // Returns after a lift ended that already belong to the new height.
  static constexpr double hold_lead_s = .5;
  void start_lift(double stamp, double not_before) {
    if (!lifting_) {
      lifting_ = true;
      lift_start_ = stamp;
    }
    window_start_ = std::max(window_start_, not_before);
  }
  // Height below which the given share of the wall lies, with the wall spread evenly over the band.
  double rim_of(std::vector<double>& heights) const {
    std::sort(heights.begin(), heights.end(), std::greater<double>());
    const double highest = heights[std::min<size_t>(2, heights.size() - 1)];
    const size_t in_band = std::count_if(heights.begin(), heights.end(), [&](double z) {
      return z >= highest - top_band_m;
    });
    return heights[std::min(heights.size() - 1, static_cast<size_t>(rim_share * in_band))] +
           rim_share * top_band_m;
  }
  void estimate(double stamp, double dt, int above) {
    if (lifting_) {
      follow_lift(stamp, dt, above);
      return;
    }
    std::vector<double> heights;
    const double from = std::max(stamp - hold_window_s, hold_start_);
    for (const auto& r : returns_)
      if (r.stamp >= from) heights.push_back(r.z);
    if (static_cast<int>(heights.size()) < hold_min_returns) return;
    const double seen = rim_of(heights);
    // Far away a window can hold too few rim returns, and its top is then the chassis. Only a lift
    // moves a held height far, and that takes evidence from the rim itself.
    if (std::abs(seen - rim_) <= hold_reach_m)
      rim_ += (1 - std::exp(-dt / hold_time_s)) * (seen - rim_);
  }
  // While lifting, find the line that the returns of the window lie just below, by the quantile
  // regression of their heights over time. It uses only the returns, so an error in the estimate
  // cannot feed back into the next one.
  void follow_lift(double stamp, double dt, int above) {
    std::vector<Return> window;
    double top = -std::numeric_limits<double>::infinity();
    const auto collect = [&](double from) {
      window.clear();
      top = -std::numeric_limits<double>::infinity();
      for (const auto& r : returns_)
        if (r.stamp >= std::max(from, window_start_)) {
          window.push_back(r);
          top = std::max(top, r.z);
        }
    };
    collect(stamp - min_lift_window_s);
    // A sparse window gets longer rather than being trusted.
    if (static_cast<int>(window.size()) < lift_min_returns) collect(stamp - hold_window_s);
    if (static_cast<int>(window.size()) >= lift_min_returns) {
      // Only the upper part locates the rim. The chassis below it does not move with a lift.
      window.erase(std::remove_if(window.begin(), window.end(),
                                  [&](const Return& r) { return r.z < top - line_band_m; }),
                   window.end());
    }
    if (static_cast<int>(window.size()) >= lift_min_returns) {
      double best = std::numeric_limits<double>::infinity(), best_rim = rim_, best_speed = 0;
      std::vector<double> heights(window.size());
      // A line may not end far from the returns, or a sparse window would extrapolate: the rim is
      // no higher than what was hit, and no lower than what was hit just now.
      double newest_top = -std::numeric_limits<double>::infinity();
      for (const auto& r : window)
        if (stamp - r.stamp <= newest_s) newest_top = std::max(newest_top, r.z);
      for (double speed = -max_speed; speed <= max_speed + 1e-9; speed += speed_step) {
        for (size_t i = 0; i < window.size(); ++i)
          heights[i] = window[i].z - speed * (window[i].stamp - stamp);
        std::vector<double> sorted = heights;
        const double rim = rim_of(sorted);
        if (rim > top + line_margin_m || rim < newest_top - line_margin_m) continue;
        double loss = 0;
        for (const double z : heights) {
          const double e = z - rim;
          loss += e > 0 ? rim_quantile * e : (rim_quantile - 1) * e;
        }
        loss *= 1 + speed_penalty * std::abs(speed);
        if (loss < best) {
          best = loss;
          best_rim = rim;
          best_speed = speed;
        }
      }
      // The bucket cannot jump, whatever a sparse window suggests.
      const double reach = max_speed * dt + jump_margin_m;
      rim_ = std::clamp(std::clamp(best_rim, rim_ - reach, rim_ + reach), min_, max_);
      speed_ = best_speed;
    }
    if (stamp - lift_start_ >= min_lift_s && std::abs(speed_) < end_speed && above == 0) {
      lifting_ = false;
      speed_ = 0;
      hold_start_ = stamp - hold_lead_s;
      window_start_ = -std::numeric_limits<double>::infinity();
    }
  }
  double min_{0}, max_{0}, rim_{0}, speed_{0};
  double stamp_{0}, hold_start_{0}, last_hit_{0};
  double at_rim_rate_{0}, below_rim_rate_{0}, missed_{0};
  double lift_start_{0}, window_start_{-std::numeric_limits<double>::infinity()};
  bool ready_{false}, lifting_{false};
  std::deque<Return> returns_;
};
}  // namespace lidar_perception_system::opening
