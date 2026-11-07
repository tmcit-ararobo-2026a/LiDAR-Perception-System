// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
// Following the bucket axis from returns on the bucket and the chassis, without a chassis pose.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <random>
#include <vector>

#include "lidar_perception_system/bottom_pose_filter.hpp"
#include "lidar_perception_system/bucket_axis_filter.hpp"

namespace op = lidar_perception_system::opening;
using Point = op::BucketAxisFilter::Point;

namespace {
int failures = 0;
// Every test runs over a few random scenes, as one could be lucky.
unsigned seed_offset = 0;

void check(bool condition, const char* what) {
  if (condition) return;
  std::cerr << "FAILED: " << what << '\n';
  ++failures;
}

constexpr double scan_dt = .02, rim_height = 1.6, rim_radius = .1365, floor_radius = .1076, bucket_height = .255;
const Eigen::Vector2d sensor(0, 0);

op::BucketAxisFilter make_filter() { return op::BucketAxisFilter(.15, 1., bucket_height, rim_radius, floor_radius, .17); }

struct Scene {
  explicit Scene(unsigned seed) : rng(seed + 100 * seed_offset) {}
  std::mt19937 rng;
  double strip_rate{50}, chassis_rate{1500}, noise{.01};
  bool transparent{false}, chassis{true};
  double uniform() { return std::uniform_real_distribution<double>(0, 1)(rng); }
  // Returns come as strokes of the scan pattern: all of a stroke hit the wall on one vertical line.
  std::vector<Point> scan(const Eigen::Vector2d& axis) {
    std::vector<Point> points;
    const Eigen::Vector2d line = (axis - sensor).normalized();
    const Eigen::Vector2d side(-line.y(), line.x());
    std::normal_distribution<double> scatter(0, noise);
    const double strokes = strip_rate * scan_dt / 3;
    int count = std::poisson_distribution<int>(strokes)(rng);
    for (; count > 0; --count) {
      const double offset = (2 * uniform() - 1) * .95;
      const double far_side = transparent && uniform() < .4 ? 1. : -1.;
      for (int i = 0; i < 3; ++i) {
        const double below = .17 * uniform();
        const double radius = floor_radius + (rim_radius - floor_radius) * (1 - below / bucket_height);
        const double inside = transparent && uniform() < .3 ? uniform() : 1.;
        const Eigen::Vector2d p = axis + side * offset * radius * inside +
                                  line * far_side * std::sqrt(1 - offset * offset) * radius * inside +
                                  line * scatter(rng);
        points.emplace_back(p.x(), p.y(), rim_height - below);
      }
    }
    if (chassis) {
      // The chassis faces the sensor as one wall and its side, behind the bucket.
      int returns = std::poisson_distribution<int>(chassis_rate * scan_dt)(rng);
      for (; returns > 0; --returns) {
        const double u = uniform();
        Eigen::Vector2d p = axis + line * .1;
        if (u < .7)
          p += side * (uniform() - .5) * .7;
        else
          p += side * .35 + line * uniform() * .6;
        p += line * scatter(rng);
        points.emplace_back(p.x(), p.y(), .2 + .7 * uniform());
      }
    }
    return points;
  }
};

struct Errors {
  std::vector<double> values;
  double median() {
    std::sort(values.begin(), values.end());
    return values.empty() ? 0. : values[values.size() / 2];
  }
  double largest() { return values.empty() ? 0. : *std::max_element(values.begin(), values.end()); }
};

Eigen::Vector2d straight(double t, const Eigen::Vector2d& start, const Eigen::Vector2d& velocity) { return start + velocity * t; }

void test_holds_a_still_opaque_bucket() {
  auto filter = make_filter();
  const Eigen::Vector2d axis(5, 1);
  filter.reset(axis + Eigen::Vector2d(.05, -.05), 100);
  Scene scene(1);
  Errors errors;
  for (double t = 0; t < 10; t += scan_dt) {
    filter.update(100 + t, scene.scan(axis), sensor, rim_height, axis);
    if (t > 2) errors.values.push_back((filter.axis() - axis).norm());
  }
  check(errors.median() < .025, "a still opaque bucket is found within 2.5 cm");
  check(errors.largest() < .06, "a still opaque bucket stays within 6 cm");
}

void test_holds_a_still_transparent_bucket() {
  // Returns also come from the far wall and the inside, which must not pull it towards the sensor.
  auto filter = make_filter();
  const Eigen::Vector2d axis(5, 1);
  filter.reset(axis + Eigen::Vector2d(.05, -.05), 100);
  Scene scene(2);
  scene.transparent = true;
  Errors errors;
  for (double t = 0; t < 10; t += scan_dt) {
    filter.update(100 + t, scene.scan(axis), sensor, rim_height, axis);
    if (t > 2) errors.values.push_back((filter.axis() - axis).norm());
  }
  check(errors.median() < .04, "a still transparent bucket is found within 4 cm");
  check(errors.largest() < .1, "a still transparent bucket stays within 10 cm");
}

void test_follows_a_robot_crossing_the_line_of_sight() {
  // Sideways the strip alone hardly shows the speed, and the chassis must.
  auto filter = make_filter();
  const Eigen::Vector2d start(5, -2), velocity(0, 1);
  filter.reset(start, 100);
  Scene scene(3);
  scene.strip_rate = 20;
  Errors errors, speed;
  for (double t = 0; t < 4; t += scan_dt) {
    const auto axis = straight(t, start, velocity);
    filter.update(100 + t, scene.scan(axis), sensor, rim_height, axis);
    if (t > 1) {
      errors.values.push_back((filter.axis() - axis).norm());
      speed.values.push_back((filter.velocity() - velocity).norm());
    }
  }
  check(errors.median() < .08, "a robot at 1 m/s across the line of sight is followed within 8 cm");
  check(errors.largest() < .2, "a robot at 1 m/s across the line of sight stays within 20 cm");
  check(speed.median() < .15, "its speed is known within 0.15 m/s");
}

void test_chassis_pose_does_not_matter() {
  // The pose puts the mount 40 cm off, as a heading error does, and even sparse returns decide.
  auto filter = make_filter();
  const Eigen::Vector2d start(5, -2), velocity(0, 1), wrong(.3, -.25);
  filter.reset(start + wrong, 100);
  Scene scene(4);
  scene.strip_rate = 6;
  Errors errors;
  for (double t = 0; t < 4; t += scan_dt) {
    const auto axis = straight(t, start, velocity);
    filter.update(100 + t, scene.scan(axis), sensor, rim_height, Eigen::Vector2d(axis + wrong));
    if (t > 1) errors.values.push_back((filter.axis() - axis).norm());
  }
  check(errors.median() < .1, "a chassis pose 40 cm off does not move the axis");
}

void test_pose_far_off_does_not_drag_a_seen_bucket() {
  auto filter = make_filter();
  const Eigen::Vector2d axis(5, 1), inferred(5.9, 1);
  filter.reset(axis, 100);
  Scene scene(14);
  scene.strip_rate = 15;
  Errors errors;
  for (double t = 0; t < 8; t += scan_dt) {
    filter.update(100 + t, scene.scan(axis), sensor, rim_height, inferred);
    if (t > 3) errors.values.push_back((filter.axis() - axis).norm());
  }
  check(errors.median() < .05, "a pose 90 cm off does not drag a bucket that is seen");
}

void test_sparse_returns_are_not_lost_to_a_pose_far_off() {
  // With a gap in the returns the pose has a say. It must not take the axis out of their reach.
  auto filter = make_filter();
  const Eigen::Vector2d axis(5, 1), inferred(5.9, 1);
  filter.reset(axis, 100);
  Scene scene(15);
  scene.strip_rate = 6;
  Errors errors;
  for (double t = 0; t < 8; t += scan_dt) {
    filter.update(100 + t, scene.scan(axis), sensor, rim_height, inferred);
    if (t > 3) errors.values.push_back((filter.axis() - axis).norm());
  }
  check(errors.median() < .2, "sparse returns keep the axis although the pose is 90 cm off");
}

void test_settles_after_a_stop() {
  auto filter = make_filter();
  const Eigen::Vector2d start(5, -2), velocity(0, 1);
  filter.reset(start, 100);
  Scene scene(5);
  double worst_after = 0;
  for (double t = 0; t < 5; t += scan_dt) {
    const auto axis = straight(std::min(t, 2.), start, velocity);
    filter.update(100 + t, scene.scan(axis), sensor, rim_height, axis);
    if (t > 3) worst_after = std::max(worst_after, (filter.axis() - axis).norm());
  }
  check(worst_after < .06, "the axis stays within 6 cm from 1 s after a stop");
  check(filter.velocity().norm() < .2, "the speed is gone after a stop");
}

void test_recovers_from_the_sensor_side_of_the_returns() {
  // Near-side returns also fit a circle on the sensor's side of them.
  auto filter = make_filter();
  const Eigen::Vector2d axis(5, 0);
  filter.reset(axis - Eigen::Vector2d(2 * rim_radius, 0), 100);
  Scene scene(6);
  scene.strip_rate = 30;
  for (double t = 0; t < 3; t += scan_dt) filter.update(100 + t, scene.scan(axis), sensor, rim_height, std::nullopt);
  check((filter.axis() - axis).norm() < .08, "a fit on the sensor side of the returns comes back");
}

void test_repeated_returns_are_old_positions() {
  // A window of scans republished: old returns taken as new are old positions of a moving robot.
  auto filter = make_filter();
  const Eigen::Vector2d start(5, -2), velocity(0, 1);
  filter.reset(start, 100);
  Scene scene(7);
  std::vector<std::vector<Point>> history;
  Errors errors;
  for (double t = 0; t < 4; t += scan_dt) {
    const auto axis = straight(t, start, velocity);
    history.push_back(scene.scan(axis));
    std::vector<Point> window;
    for (size_t i = history.size() > 7 ? history.size() - 7 : 0; i < history.size(); ++i)
      window.insert(window.end(), history[i].begin(), history[i].end());
    filter.update(100 + t, window, sensor, rim_height, axis);
    if (t > 1) errors.values.push_back((filter.axis() - axis).norm());
  }
  check(errors.median() < .05, "republished returns do not delay the axis");
}

void test_one_flat_face_says_nothing_along_it() {
  // Only one face of the chassis is in view, and the robot stops while moving along it. The match
  // would go on repeating the old speed.
  auto filter = make_filter();
  const Eigen::Vector2d start(5, -1), velocity(0, 1);
  filter.reset(start, 100);
  Scene scene(12), face(13);
  face.chassis = false;
  std::mt19937 rng(31 + seed_offset);
  std::uniform_real_distribution<double> unit(0, 1);
  double worst = 0;
  for (double t = 0; t < 4; t += scan_dt) {
    const auto axis = straight(std::min(t, 2.), start, velocity);
    auto points = face.scan(axis);
    for (int i = 0; i < 30; ++i) points.emplace_back(axis.x() - .1, axis.y() + 2 * (unit(rng) - .5), .2 + .7 * unit(rng));
    filter.update(100 + t, points, sensor, rim_height, axis);
    if (t > 3) worst = std::max(worst, (filter.axis() - axis).norm());
  }
  check(worst < .1, "after stopping along one flat face the axis stays within 10 cm");
}

void test_another_robot_is_left_alone() {
  // Another robot passes 1.5 m away, its top at the height of the strip, moving the other way.
  auto filter = make_filter();
  const Eigen::Vector2d start(5, -2), velocity(0, 1);
  filter.reset(start, 100);
  Scene scene(9), other(10);
  other.strip_rate = 200;
  other.chassis_rate = 6000;
  Errors errors, speed;
  for (double t = 0; t < 4; t += scan_dt) {
    const auto axis = straight(t, start, velocity);
    auto points = scene.scan(axis);
    const auto far = other.scan(Eigen::Vector2d(axis + Eigen::Vector2d(1.5, 0) - 2 * velocity * t));
    points.insert(points.end(), far.begin(), far.end());
    filter.update(100 + t, points, sensor, rim_height, axis);
    if (t > 1) {
      errors.values.push_back((filter.axis() - axis).norm());
      speed.values.push_back((filter.velocity() - velocity).norm());
    }
  }
  check(errors.median() < .06, "another robot does not pull the axis");
  check(speed.median() < .15, "another robot does not change the speed");
}

void test_only_the_top_strip_locates_the_bucket() {
  // Parts of the robot below the strip spread wider than the bucket.
  auto filter = make_filter();
  const Eigen::Vector2d axis(5, 1);
  filter.reset(axis, 100);
  Scene scene(11);
  scene.chassis = false;
  std::mt19937 rng(21);
  std::uniform_real_distribution<double> unit(0, 1);
  for (double t = 0; t < 5; t += scan_dt) {
    auto points = scene.scan(axis);
    for (int i = 0; i < 4; ++i)
      points.emplace_back(axis.x() + .3 * unit(rng), axis.y() + .5 * (unit(rng) - .5), rim_height - .25 - .2 * unit(rng));
    filter.update(100 + t, points, sensor, rim_height, axis);
  }
  check((filter.axis() - axis).norm() < .05, "returns below the strip do not move the axis");
}

void test_no_returns_falls_back_on_the_pose() {
  auto filter = make_filter();
  const Eigen::Vector2d axis(5, 1), inferred(5.3, 1.2);
  filter.reset(axis, 100);
  for (double t = 0; t < 8; t += scan_dt) filter.update(100 + t, {}, sensor, rim_height, inferred);
  check((filter.axis() - inferred).norm() < .05, "with nothing to see the axis goes to where the pose puts it");
  check(filter.velocity().norm() < .1, "with nothing to see it does not move off");
}

void test_empty_scans_do_not_run_away() {
  // The speed goes on through a gap, but only for as far as it is credible.
  auto filter = make_filter();
  const Eigen::Vector2d start(5, -2), velocity(0, 1);
  filter.reset(start, 100);
  Scene scene(8);
  for (double t = 0; t < 2; t += scan_dt) {
    const auto axis = straight(t, start, velocity);
    filter.update(100 + t, scene.scan(axis), sensor, rim_height, axis);
  }
  const auto stopped = straight(2, start, velocity);
  for (double t = 2; t < 12; t += scan_dt) filter.update(100 + t, {}, sensor, rim_height, stopped);
  check((filter.axis() - stopped).norm() < .1, "a gap does not leave the axis running away");
}

void test_far_from_the_pose_starts_over() {
  auto filter = make_filter();
  filter.reset(Eigen::Vector2d(5, 1), 100);
  filter.update(100.02, {}, sensor, rim_height, Eigen::Vector2d(5, 3));
  check((filter.axis() - Eigen::Vector2d(5, 3)).norm() < .01, "an axis beyond any turn of the robot starts over");
}

void test_clock_going_back_starts_over() {
  auto filter = make_filter();
  filter.reset(Eigen::Vector2d(5, 1), 100);
  filter.update(99, {}, sensor, rim_height, Eigen::Vector2d(5.2, 1));
  filter.update(99.02, {}, sensor, rim_height, Eigen::Vector2d(5.2, 1));
  check((filter.axis() - Eigen::Vector2d(5.2, 1)).norm() < .01, "a clock going back starts over");
  check(filter.ready(), "and the filter keeps going");
}

void test_clear_stops_following() {
  auto filter = make_filter();
  filter.reset(Eigen::Vector2d(5, 1), 100);
  filter.clear();
  check(!filter.ready(), "a cleared filter is not ready");
  filter.update(100.02, {}, sensor, rim_height, Eigen::Vector2d(5, 1));
  check(!filter.ready(), "and does not start by itself");
}

void test_pose_filter_stays_with_a_followed_axis_that_comes_and_goes() {
  // Switching between a followed axis and a body-inferred one must not wind up the output.
  op::BottomPoseFilter pose_filter;
  Eigen::Vector3d normal(0, 0, 1);
  double largest = 0;
  for (int i = 0; i < 2000; ++i) {
    const double t = 100 + i * scan_dt;
    const bool followed = (i / 40) % 2 == 0;
    Eigen::Vector3d center(followed ? 0.3 : 0., 0, 1);
    pose_filter.update(center, normal, t, 1, 1., false, false, followed, true, followed);
    largest = std::max(largest, center.head<2>().norm());
  }
  check(largest < .5, "the output stays between the two positions");
}

void test_pose_filter_passes_a_followed_axis() {
  op::BottomPoseFilter pose_filter;
  Eigen::Vector3d normal(0, 0, 1);
  double worst = 0;
  for (int i = 0; i < 200; ++i) {
    const double t = 100 + i * scan_dt;
    const Eigen::Vector3d truth(i * scan_dt, 0, 1);
    Eigen::Vector3d center = truth;
    pose_filter.update(center, normal, t, 1, 1., false, false, true, true, true);
    if (i > 10) worst = std::max(worst, (center - truth).head<2>().norm());
  }
  check(worst < .005, "a followed axis is not delayed by the output filter");
}
}  // namespace

int main() {
  for (seed_offset = 0; seed_offset < 8; ++seed_offset) {
    test_holds_a_still_opaque_bucket();
    test_holds_a_still_transparent_bucket();
    test_follows_a_robot_crossing_the_line_of_sight();
    test_chassis_pose_does_not_matter();
    test_pose_far_off_does_not_drag_a_seen_bucket();
    test_sparse_returns_are_not_lost_to_a_pose_far_off();
    test_settles_after_a_stop();
    test_recovers_from_the_sensor_side_of_the_returns();
    test_repeated_returns_are_old_positions();
    test_one_flat_face_says_nothing_along_it();
    test_another_robot_is_left_alone();
    test_only_the_top_strip_locates_the_bucket();
    test_empty_scans_do_not_run_away();
  }
  test_no_returns_falls_back_on_the_pose();
  test_far_from_the_pose_starts_over();
  test_clock_going_back_starts_over();
  test_clear_stops_following();
  test_pose_filter_stays_with_a_followed_axis_that_comes_and_goes();
  test_pose_filter_passes_a_followed_axis();
  if (failures) {
    std::cerr << failures << " check(s) failed\n";
    return 1;
  }
  std::cout << "bucket_axis_filter_test passed\n";
  return 0;
}
