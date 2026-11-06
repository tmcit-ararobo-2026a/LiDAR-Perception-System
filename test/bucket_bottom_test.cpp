// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
// Fitting the rulebook bucket to the top of a robot from sparse, noisy, far returns.
#include <Eigen/Geometry>

#include <cmath>
#include <deque>
#include <iostream>
#include <optional>
#include <random>

#include "lidar_perception_system/opening_tracker.hpp"

namespace op = lidar_perception_system::opening;

namespace {
int failures = 0;

void check(bool condition, const char* what) {
  if (condition) return;
  std::cerr << "FAILED: " << what << '\n';
  ++failures;
}

constexpr double floor_z = 1.24, scan_dt = .1, sigma = .022;
const Eigen::Vector2d axis(.31, -.18);

struct Bucket {
  double inner_share{.3};
  double noise{sigma};
  double floor{floor_z};
  // Bearing range of the wall returns around the axis, seen from the sensor.
  double bearing_from{-3.1}, bearing_to{3.1};
};

// One scan of a transparent bucket: weak wall returns, and returns between the walls that
// the sensor could not separate.
op::Points scan_bucket(const Bucket& b, std::mt19937& rng, int points = 6) {
  std::uniform_real_distribution<double> unit(0, 1);
  std::normal_distribution<double> noise(0, b.noise);
  op::Points out;
  const op::Config cfg;
  for (int i = 0; i < points; ++i) {
    const double height = cfg.bottom.top_band_m > 0 ? .06 + unit(rng) * (cfg.bottom.bucket_height - .06) : 0;
    const double radius = cfg.bottom.floor_radius +
                          (cfg.nominal_radius - cfg.bottom.floor_radius) * height / cfg.bottom.bucket_height;
    const double bearing = b.bearing_from + unit(rng) * (b.bearing_to - b.bearing_from);
    const double r = unit(rng) < b.inner_share ? radius * std::sqrt(unit(rng)) : radius + noise(rng);
    out.emplace_back(axis.x() + r * std::cos(bearing), axis.y() + r * std::sin(bearing),
                     b.floor + height + .3 * noise(rng));
  }
  return out;
}

// What a chassis under the bucket returns: wider than the bucket, but below its 2 L mark.
op::Points scan_frame(std::mt19937& rng, double top) {
  std::uniform_real_distribution<double> unit(0, 1);
  op::Points out;
  for (int i = 0; i < 6; ++i)
    out.emplace_back(axis.x() + (unit(rng) - .5) * .6, axis.y() + (unit(rng) - .5) * .5,
                     top - unit(rng) * .25);
  return out;
}

std::deque<op::TimedPoints> collect(const Bucket& bucket, double seconds, unsigned seed = 7,
                                    bool frame = true) {
  std::mt19937 rng(seed);
  std::deque<op::TimedPoints> scans;
  for (double t = 0; t < seconds; t += scan_dt) {
    op::TimedPoints scan;
    scan.stamp = 100 + t;
    scan.points = scan_bucket(bucket, rng);
    if (frame) {
      const auto below = scan_frame(rng, bucket.floor + .05);
      scan.points.insert(scan.points.end(), below.begin(), below.end());
    }
    scans.push_back(std::move(scan));
  }
  return scans;
}

op::TopFitContext context_for(const std::deque<op::TimedPoints>& scans) {
  op::TopFitContext context;
  context.stamp = scans.back().stamp;
  context.sigma = sigma;
  return context;
}

double xy_error(const op::Bottom& bottom) { return (bottom.center.head<2>() - axis).norm(); }

void test_far_transparent_bucket() {
  const auto scans = collect(Bucket{}, 3.);
  const op::Config cfg;
  op::TopFitStats stats;
  const auto bottom = op::fit_bucket_top(scans, context_for(scans), cfg, &stats);
  check(bottom.has_value(), "a far transparent bucket is fitted");
  if (!bottom) return;
  check(xy_error(*bottom) < .03, "its axis is found within 3 cm");
  check(std::abs(bottom->center.z() - floor_z) < .03, "its floor height is found within 3 cm");
}

void test_only_near_side_is_seen() {
  Bucket opaque;
  opaque.inner_share = 0;
  opaque.bearing_from = -1.4;
  opaque.bearing_to = 1.4;
  const auto scans = collect(opaque, 3.);
  const auto bottom = op::fit_bucket_top(scans, context_for(scans), op::Config{});
  check(bottom.has_value(), "the near half of a bucket is enough");
  if (bottom) check(xy_error(*bottom) < .05, "the near half locates the axis within 5 cm");
}

void test_too_little_data_waits() {
  const auto scans = collect(Bucket{}, .3);
  check(!op::fit_bucket_top(scans, context_for(scans), op::Config{}),
        "a few returns give no fit");
}

void test_top_cut_off_by_the_view() {
  const auto scans = collect(Bucket{}, 3.);
  const op::Config cfg;
  auto context = context_for(scans);
  const double top = floor_z + cfg.bottom.bucket_height;
  context.view_ceiling_z = top - .1;
  check(!op::fit_bucket_top(scans, context, cfg),
        "a top at the edge of the view says nothing about the height");
  context.expected_floor_z = floor_z;
  const auto bottom = op::fit_bucket_top(scans, context, cfg);
  check(bottom.has_value(), "a known floor still gives the position");
  if (bottom) {
    check(xy_error(*bottom) < .04, "the position is right with a cut-off top");
    check(bottom->center.z() == floor_z, "the known floor is kept");
  }
}

void test_known_floor_is_used() {
  const auto scans = collect(Bucket{}, 3.);
  const op::Config cfg;
  auto context = context_for(scans);
  context.expected_floor_z = floor_z + .02;
  const auto bottom = op::fit_bucket_top(scans, context, cfg);
  check(bottom && bottom->center.z() == floor_z + .02, "a known floor is reported as it is");
  if (bottom) check(xy_error(*bottom) < .03, "a known floor still measures the position");
}

void test_things_that_are_not_a_bucket() {
  const op::Config cfg;
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> unit(0, 1);
  std::deque<op::TimedPoints> bar, post, floating;
  for (double t = 0; t < 3.; t += scan_dt) {
    op::TimedPoints scan;
    scan.stamp = 100 + t;
    for (int i = 0; i < 8; ++i)  // A crossbar much wider than a bucket.
      scan.points.emplace_back((unit(rng) - .5) * .9, (unit(rng) - .5) * .05, 1.6 - unit(rng) * .15);
    bar.push_back(scan);
    scan.points.clear();
    for (int i = 0; i < 8; ++i)  // A thin post.
      scan.points.emplace_back((unit(rng) - .5) * .05, (unit(rng) - .5) * .05, 1.6 - unit(rng) * .4);
    post.push_back(scan);
    scan.points.clear();
    for (int i = 0; i < 8; ++i)  // A bucket-sized blob above the rulebook height.
      scan.points.emplace_back((unit(rng) - .5) * .25, (unit(rng) - .5) * .25, 2.75 - unit(rng) * .25);
    floating.push_back(scan);
  }
  check(!op::fit_bucket_top(bar, context_for(bar), cfg), "a crossbar is not a bucket");
  check(!op::fit_bucket_top(post, context_for(post), cfg), "a post is not a bucket");
  check(!op::fit_bucket_top(floating, context_for(floating), cfg),
        "a top above the rulebook height is not a bucket");
}

void test_passing_returns_do_not_lift_the_rim() {
  auto scans = collect(Bucket{}, 3.);
  // A thrown cloth or a hand: a few returns above the rim, each in one scan only.
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> unit(0, 1);
  for (size_t i = 0; i < scans.size(); i += 6)
    for (int k = 0; k < 3; ++k)
      scans[i].points.emplace_back(axis.x() + (unit(rng) - .5) * .4 + .3 * i,
                                   axis.y() + (unit(rng) - .5) * .4, floor_z + .45 + .1 * unit(rng));
  const op::Config cfg;
  const auto bottom = op::fit_bucket_top(scans, context_for(scans), cfg);
  check(bottom.has_value(), "passing returns do not prevent the fit");
  if (bottom) check(std::abs(bottom->center.z() - floor_z) < .03, "passing returns do not move the rim");
}

void test_sensor_view_ceiling() {
  op::SensorView view;
  view.origin = {0, 0, 1.09};
  // Mounted upside down: the sensor z axis points down, and its field of view is -7 to +52 deg.
  view.axis = {0, 0, -1};
  view.min_elevation = -7 * M_PI / 180;
  view.max_elevation = 52 * M_PI / 180;
  const auto ceiling = view.ceiling_z({5, 0});
  check(ceiling.has_value(), "a level upside-down sensor sees ahead");
  if (ceiling)
    check(std::abs(*ceiling - (1.09 + 5 * std::tan(7 * M_PI / 180))) < .1,
          "its ceiling follows the -7 deg edge of the field of view");
  view.axis = {0, 0, 1};
  view.min_elevation = 80 * M_PI / 180;
  view.max_elevation = 85 * M_PI / 180;
  const auto out = view.ceiling_z({5, 0});
  check(!out.has_value(), "a bearing outside the field of view has no ceiling");
}
}  // namespace

int main() {
  test_far_transparent_bucket();
  test_only_near_side_is_seen();
  test_too_little_data_waits();
  test_top_cut_off_by_the_view();
  test_known_floor_is_used();
  test_things_that_are_not_a_bucket();
  test_passing_returns_do_not_lift_the_rim();
  test_sensor_view_ceiling();
  if (failures) {
    std::cerr << failures << " check(s) failed\n";
    return 1;
  }
  std::cout << "bucket_bottom_test passed\n";
  return 0;
}
