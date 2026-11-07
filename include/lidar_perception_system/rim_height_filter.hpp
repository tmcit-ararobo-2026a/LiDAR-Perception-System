// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace lidar_perception_system::opening {
// Follows the height of the bucket rim from every scan. Nothing on the robot is above the rim and
// its wall fills the band below it, so the rim is the upper edge of the returns.
//
// The height moves only as far as the returns prove:
// - The rim is at least as high as what two scans hit above it.
// - It is lower only if enough returns stayed below that height, where the wall should have been
//   hit too. How many are needed follows from how many land on the wall, at any range.
// A line fitted to the recent returns refines the height between such proofs and gives its speed.
// Sparse returns can mislead a fit, so on its own it may not move the height far. With no evidence
// either way, the height stays where it is.
class RimHeightFilter {
 public:
  RimHeightFilter(double min_rim, double max_rim) : min_(min_rim), max_(max_rim) {
    if (!std::isfinite(min_rim) || !std::isfinite(max_rim) || max_rim <= min_rim)
      throw std::invalid_argument("Rim height range must be finite and ordered");
  }
  bool ready() const { return ready_; }
  void clear() {
    ready_ = false;
    returns_.clear();
    speed_ = 0;
    goal_.reset();
  }
  // Start from a rim seen by a fit.
  void reset(double rim, double stamp) {
    clear();
    ready_ = true;
    rim_ = seen_rim_ = std::clamp(rim, min_, max_);
    stamp_ = last_top_hit_ = stamp;
    fit_from_ = stamp - never_s;
    last_rise_ = last_fall_ = stamp - never_s;
  }
  // Absorb a direct measurement of the rim, such as a fitted floor plane.
  void measure(double rim) {
    if (ready_ && std::abs(rim - rim_) < measure_reach_m) rim_ += measure_gain * (rim - rim_);
  }
  // One scan: heights of the new returns around the bucket axis, the expected scatter of a height,
  // and the highest height the sensor can see on this bearing.
  void update(double stamp, const std::vector<double>& heights, double sigma,
              std::optional<double> ceiling = std::nullopt) {
    if (!ready_) return;
    if (stamp < stamp_) reset(rim_, stamp);  // The clock went back, so old returns mean nothing.
    const double dt = stamp - stamp_;
    stamp_ = stamp;
    // Carried along its speed, but not far past where the rim was last seen.
    double rim = std::clamp(rim_ + speed_ * dt, seen_rim_ - carry_reach_m, seen_rim_ + carry_reach_m);
    rim = std::clamp(rim, min_, max_);
    // A speed needs fresh evidence of its own direction, as a lift can stop unseen.
    if ((speed_ > 0 && stamp - last_rise_ > fresh_s) || (speed_ < 0 && stamp - last_fall_ > fresh_s))
      speed_ *= std::exp(-dt / speed_fade_s);
    add_returns(stamp, heights);
    if (heights.empty()) {
      rim_ = rim;
      return;
    }
    const double margin = std::max(min_margin_m, margin_sigmas * sigma);
    const bool in_view = !ceiling || rim + view_margin_m < *ceiling;
    const Line carried{rim, speed_};
    for (const double z : heights)
      if (z > rim - top_hit_m && z <= rim + margin) {
        seen_rim_ = rim;
        last_rise_ = stamp;
      }
    // What rose long ago says nothing about now, and after a reversal it would keep the height up.
    const double rise = proven_rise(carried, stamp, rise_window_s, margin, newest_s);
    double fall = in_view ? proven_fall(carried, stamp, fall_window_s, margin, sigma, &fit_from_) : 0.;
    if (rise <= 0 && fall <= 0 && in_view) fall = fall_after_silence(carried, stamp, margin);

    int line_returns = 0;
    const auto line = best_line(stamp, margin, sigma, in_view, &line_returns);
    // Without proof, only a fit with enough returns close to the height may refine it. Limiting the
    // move instead of ignoring a distant fit would let the height creep towards the chassis.
    double target = rim;
    if (rise > 0) {
      const double least = rim + rise - rise_offset_sigmas * sigma;
      target = line ? std::clamp(line->rim, least, std::max(least, rim + rise)) : least;
    } else if (fall > 0) {
      target = rim - fall;
    } else if (line && line_returns >= refine_min_returns && std::abs(line->rim - rim) <= refine_reach_m) {
      target = line->rim;
    }
    const bool proven = rise > 0 || fall > 0;
    if (rise > 0) {
      last_rise_ = stamp;
      speed_ = std::max(speed_, 0.);
    }
    if (fall > 0) {
      last_fall_ = stamp;
      speed_ = std::min(speed_, 0.);
    }
    // A proven height is pursued until reached, as the proof may come from a single scan.
    if (proven) goal_ = target;
    if (goal_) target = *goal_;
    const double step = (1 - std::exp(-dt / (goal_ ? proven_time_s : refine_time_s))) * (target - rim);
    const double reach = max_speed * dt + jump_margin_m;
    rim = std::clamp(rim + std::clamp(step, -reach, reach), min_, max_);
    if (goal_ && std::abs(*goal_ - rim) < goal_reached_m) goal_.reset();
    if (proven) seen_rim_ = rim;
    if (line) {
      const bool fresh = stamp - (line->speed > 0 ? last_rise_ : last_fall_) <= fresh_s;
      speed_ += (1 - std::exp(-dt / speed_time_s)) * ((fresh ? line->speed : 0.) - speed_);
    }
    rim_ = rim;
  }
  double rim() const { return rim_; }
  double speed() const { return speed_; }

 private:
  struct Return {
    double stamp, z;
    bool new_height;  // Not a repeat of a height already hit in the same scan.
  };
  struct Line {
    double rim, speed;
  };
  // Windows of returns the line is fitted to, from which the longest that nothing contradicts is used.
  static constexpr std::array<double, 4> line_windows_s{.5, .9, 1.6, 3.};
  static constexpr double return_window_s = line_windows_s.back();
  // A return this far above a line counts as above it.
  static constexpr double min_margin_m = .015, margin_sigmas = 1.5;
  // A rim this close to the highest height the sensor sees is not expected to be hit.
  static constexpr double view_margin_m = .03;
  // Time within which returns of two scans above the rim prove a rise, and how much of their
  // scatter is taken off.
  static constexpr double rise_window_s = .5, rise_offset_sigmas = .5;
  // Band below the rim where the returns fall evenly on the wall. The floor and what is below it
  // return more densely, so they say nothing about how often the wall should have been hit.
  static constexpr double wall_band_m = .17;
  // Log-odds needed to accept that a strip below the line has no wall, and the fewest returns and
  // shortest time for that, as the returns of one sweep of the scan pattern are not independent.
  static constexpr double fall_window_s = 3., fall_confidence = 2., fall_min_returns = 12, fall_min_span_s = .3;
  // Multiple of the expected gap between the rim and the highest return that the height stays above it.
  static constexpr double fall_gap_share = 2.;
  // One stroke of the scan pattern hits a height once, whatever the number of returns in this cell.
  static constexpr double height_cell_m = .01;
  // If nothing reaches the band at all while the robot still returns, the rim came down further.
  static constexpr int silence_min_returns = 3;
  static constexpr double min_silence_s = .25, max_silence_s = 1.5, silence_confidence = 6.;
  // Time constants of a proven move and of refining, and how far a fit alone may refine.
  static constexpr double proven_time_s = .04, refine_time_s = .25, refine_reach_m = .03;
  static constexpr int refine_min_returns = 10;
  // How close a proven height counts as reached.
  static constexpr double goal_reached_m = .005;
  // Time a speed stays credible without evidence, and how far it can carry the height.
  static constexpr double fresh_s = .4, speed_fade_s = .2, speed_time_s = .15, carry_reach_m = .04;
  // A return this close below the rim hits it.
  static constexpr double top_hit_m = .04;
  // Candidate speeds of the rim, the band of returns a line is fitted to, and the fewest of them.
  static constexpr double max_speed = 1.2, speed_step = .04, line_band_m = .35;
  static constexpr int line_min_returns = 6;
  // Cost of a line: the gaps below it up to a cap, returns above it, and its speed, so that a line
  // only moves if the returns say so.
  static constexpr double gap_cap_m = .2, above_weight = 3., speed_cost = .1;
  // Share of the band above the rank that locates the edge.
  static constexpr double edge_share = .1;
  // How far a line may end from the returns, and how recent "just now" is.
  static constexpr double line_margin_m = .03, newest_s = .15;
  // Allowance on top of what the fastest lift explains between two scans.
  static constexpr double jump_margin_m = .01;
  static constexpr double measure_reach_m = .05, measure_gain = .3;
  // An age older than anything the filter looks at.
  static constexpr double never_s = 1e3;

  void add_returns(double stamp, const std::vector<double>& heights) {
    cells_.clear();
    for (const double z : heights) {
      const long cell = std::lround(z / height_cell_m);
      const bool new_height = std::find(cells_.begin(), cells_.end(), cell) == cells_.end();
      if (new_height) cells_.push_back(cell);
      returns_.push_back({stamp, z, new_height});
    }
    while (!returns_.empty() && stamp - returns_.front().stamp > return_window_s) returns_.pop_front();
  }
  static double above_line(const Return& r, const Line& line, double stamp) {
    return r.z - (line.rim + line.speed * (r.stamp - stamp));
  }
  // How far above the line the rim must be. Returns of two scans are needed, which one passing
  // object is unlikely to give.
  double proven_rise(const Line& line, double stamp, double window_s, double margin,
                     double newest_within = std::numeric_limits<double>::infinity()) const {
    constexpr double none = -std::numeric_limits<double>::infinity();
    double first = none, second = none, first_stamp = -1;
    bool any = false;
    for (auto it = returns_.rbegin(); it != returns_.rend() && stamp - it->stamp <= window_s; ++it) {
      const double d = above_line(*it, line, stamp);
      if (d <= margin) continue;
      if (!any && stamp - it->stamp > newest_within) return 0.;
      any = true;
      if (d > first) {
        if (it->stamp != first_stamp) second = first;
        first = d;
        first_stamp = it->stamp;
      } else if (d > second && it->stamp != first_stamp) {
        second = d;
      }
    }
    return second > none ? second : 0.;
  }
  // How far below the line the rim must be. Had the wall reached the line, a share of the returns
  // on the wall below would have hit the strip in between.
  double proven_fall(const Line& line, double stamp, double window_s, double margin, double sigma,
                     double* since = nullptr) const {
    double top = -std::numeric_limits<double>::infinity(), newest = top, count = 0, best = 0, from = 0;
    for (auto it = returns_.rbegin(); it != returns_.rend() && stamp - it->stamp <= window_s; ++it) {
      const double d = above_line(*it, line, stamp);
      if (d > margin) break;  // The rim was on or above the line by then.
      if (d <= -wall_band_m) continue;
      count += it->new_height;
      top = std::max(top, d);
      newest = std::max(newest, it->stamp);
      if (count >= fall_min_returns && newest - it->stamp >= fall_min_span_s &&
          count * (-top - sigma) / wall_band_m >= fall_confidence) {
        // Stays above the highest return by the gap that the number of returns leaves open. Testing
        // many windows on each scan raises the odds of a false alarm, and this keeps it small.
        const double likely = -top - fall_gap_share * wall_band_m / count;
        if (likely > best) {
          best = likely;
          from = it->stamp;
        }
      }
    }
    // Older returns are from before it came down, and would only mislead a fit.
    if (since && best > refine_reach_m) *since = std::max(*since, from);
    return best;
  }
  // Nothing near the line at all although the robot keeps returning, for longer than the rate of
  // returns on the wall explains.
  double fall_after_silence(const Line& line, double stamp, double margin) {
    int wall = 0, under = 0;
    for (const auto& r : returns_) {
      const double d = above_line(r, line, stamp);
      if (d <= margin && d > -wall_band_m) {
        ++wall;
        last_top_hit_ = std::max(last_top_hit_, r.stamp);
      } else if (d <= -wall_band_m && stamp - r.stamp <= 1.) {
        ++under;
      }
    }
    const double silent = stamp - last_top_hit_;
    const double rate = wall / return_window_s;
    if (under < silence_min_returns ||
        !(silent > max_silence_s || (silent > min_silence_s && rate * silent >= silence_confidence)))
      return 0.;
    scratch_.clear();
    for (auto it = returns_.rbegin(); it != returns_.rend() && it->stamp > last_top_hit_; ++it)
      scratch_.push_back(it->z);
    if (scratch_.size() < 2) return 0.;
    std::nth_element(scratch_.begin(), scratch_.begin() + 1, scratch_.end(), std::greater<double>());
    fit_from_ = std::max(fit_from_, last_top_hit_);
    last_top_hit_ = stamp;
    return std::max(line.rim - scratch_[1], 0.);
  }
  // The line over the longest window that the returns of no shorter window contradict.
  std::optional<Line> best_line(double stamp, double margin, double sigma, bool in_view, int* returns) {
    std::optional<Line> best;
    for (size_t k = 0; k < line_windows_s.size(); ++k) {
      int in_band = 0;
      const auto line = fit_line(stamp, line_windows_s[k], &in_band);
      if (!line) continue;
      bool consistent = true;
      for (size_t s = 0; s < k && consistent; ++s)
        consistent = proven_rise(*line, stamp, line_windows_s[s], margin) <= 0 &&
                     (!in_view || proven_fall(*line, stamp, line_windows_s[s], margin, sigma) <= 0);
      if (!consistent) continue;
      best = line;
      *returns = in_band;
    }
    return best;
  }
  // Height of the edge of the returns, with the wall spread evenly over the band below it.
  double edge_of(std::vector<double>& heights, int* in_band) const {
    std::sort(heights.begin(), heights.end(), std::greater<double>());
    const double highest = heights[std::min<size_t>(2, heights.size() - 1)];
    const size_t count = std::count_if(heights.begin(), heights.end(), [&](double z) {
      return z >= highest - wall_band_m;
    });
    *in_band = static_cast<int>(count);
    const size_t rank = std::min(heights.size() - 1, std::max<size_t>(1, static_cast<size_t>(edge_share * count)));
    return heights[rank] + static_cast<double>(rank + 1) / static_cast<double>(count + 1) * wall_band_m;
  }
  std::optional<Line> fit_line(double stamp, double window_s, int* in_band) {
    window_.clear();
    double top = -std::numeric_limits<double>::infinity();
    for (auto it = returns_.rbegin();
         it != returns_.rend() && stamp - it->stamp <= window_s && it->stamp >= fit_from_; ++it) {
      window_.push_back(*it);
      top = std::max(top, it->z);
    }
    // Only the upper part locates the rim. The chassis below it does not move with a lift.
    window_.erase(std::remove_if(window_.begin(), window_.end(),
                                 [&](const Return& r) { return r.z < top - line_band_m; }),
                  window_.end());
    if (static_cast<int>(window_.size()) < line_min_returns) return std::nullopt;
    double newest_top = -std::numeric_limits<double>::infinity();
    for (const auto& r : window_)
      if (stamp - r.stamp <= newest_s) newest_top = std::max(newest_top, r.z);
    // The rim cannot travel further than its range within the window.
    const double fastest = std::min(max_speed, (max_ - min_) / window_s);
    const int steps = static_cast<int>(fastest / speed_step);
    std::optional<Line> best;
    double lowest_cost = std::numeric_limits<double>::infinity();
    detrended_.resize(window_.size());
    for (int step = -steps; step <= steps; ++step) {
      const double speed = step * speed_step;
      for (size_t i = 0; i < window_.size(); ++i)
        detrended_[i] = window_[i].z - speed * (window_[i].stamp - stamp);
      scratch_ = detrended_;
      int count = 0;
      const double rim = edge_of(scratch_, &count);
      // A line may not end far from the returns, or a sparse window would extrapolate: the rim is
      // no higher than what was hit, and no lower than what was hit just now.
      if (rim > top + line_margin_m || rim < newest_top - line_margin_m) continue;
      double cost = speed_cost * std::abs(speed) * std::sqrt(static_cast<double>(window_.size()));
      for (const double z : detrended_) {
        const double gap = rim - z;
        cost += gap >= 0 ? std::min(gap, gap_cap_m) : above_weight * -gap;
      }
      if (cost < lowest_cost) {
        lowest_cost = cost;
        best = Line{rim, speed};
        *in_band = count;
      }
    }
    return best;
  }
  double min_{0}, max_{0}, rim_{0}, speed_{0}, seen_rim_{0};
  std::optional<double> goal_;
  double stamp_{0}, last_top_hit_{0}, last_rise_{0}, last_fall_{0}, fit_from_{0};
  bool ready_{false};
  std::deque<Return> returns_;
  std::vector<Return> window_;
  std::vector<double> detrended_, scratch_;
  std::vector<long> cells_;
};
}  // namespace lidar_perception_system::opening
