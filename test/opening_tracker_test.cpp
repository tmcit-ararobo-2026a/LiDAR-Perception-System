// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
// Tracking behaviour on a synthetic box robot seen by a sparse scanner.
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "lidar_perception_system/opening_tracker.hpp"

namespace op = lidar_perception_system::opening;

namespace {
int failures = 0;

void check(bool condition, const char* what) {
  if (condition) return;
  std::cerr << "FAILED: " << what << '\n';
  ++failures;
}

struct RobotPose {
  double x, y, yaw;
};

constexpr double half_length = .35, half_width = .25, scan_dt = .02;
// Diagonal to every test path: a single flat face alone is rejected as a possible wall.
const Eigen::Vector2d sensor(-3.5, -3.5);

// Faces visible from the sensor, with a different sparse sample on every scan.
op::Points scan_robot(const RobotPose& robot, int scan) {
  op::Points points;
  const Eigen::Rotation2Dd rotation(robot.yaw);
  const Eigen::Vector2d center(robot.x, robot.y);
  const auto add_face = [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    const Eigen::Vector2d midpoint = center + rotation * (.5 * (a + b));
    const Eigen::Vector2d outward = rotation * (.5 * (a + b));
    if (outward.dot(sensor - midpoint) <= 0) return;
    for (int i = 0; i < 12; ++i)
      for (int level = 0; level < 5; ++level) {
        if ((i + level + scan) % 2) continue;
        const Eigen::Vector2d p = center + rotation * (a + (b - a) * (i + .5) / 12.);
        points.emplace_back(p.x(), p.y(), .25 + .15 * level);
      }
  };
  const Eigen::Vector2d c1(half_length, half_width), c2(-half_length, half_width),
      c3(-half_length, -half_width), c4(half_length, -half_width);
  add_face(c1, c2);
  add_face(c2, c3);
  add_face(c3, c4);
  add_face(c4, c1);
  return points;
}

op::Config config() {
  op::Config c;
  c.min_x = c.field_min_x;
  c.max_x = c.field_max_x;
  c.min_y = -5;
  c.max_y = 5;
  c.min_overlap = .35;
  return c;
}

// Sweeps the robot so that it is identified by its motion and its whole shape.
void acquire(op::Tracker& tracker, double& stamp, RobotPose& robot, int& scan, double seconds) {
  for (double t = 0; t < seconds; t += scan_dt, ++scan, stamp += scan_dt) {
    robot.x += .5 * scan_dt;
    robot.yaw += .3 * scan_dt;
    tracker.process(scan_robot(robot, scan), stamp, sensor);
  }
}

// Seconds until the body is reported again on the same track, or a negative value.
double recovery_time(op::Tracker& tracker, double& stamp, const RobotPose& robot, int& scan,
                     unsigned int id, double limit) {
  const double start = stamp;
  for (; stamp - start < limit; stamp += scan_dt, ++scan) {
    const auto result = tracker.process(scan_robot(robot, scan), stamp, sensor);
    if (result.body_valid) return result.track_id == id ? stamp - start : -1;
  }
  return -1;
}

void test_tracks_continuously() {
  op::Tracker tracker(config());
  double stamp = 1;
  int scan = 0, valid = 0, total = 0, ids = 0;
  unsigned int last_id = 0;
  RobotPose robot{-1, 0, 0};
  double worst_yaw = 0, reference_yaw = 0;
  bool have_reference = false;
  for (double t = 0; t < 6; t += scan_dt, ++scan, stamp += scan_dt) {
    robot.x += .4 * scan_dt;
    const auto result = tracker.process(scan_robot(robot, scan), stamp, sensor);
    if (t < 1) continue;
    ++total;
    if (!result.body_valid) continue;
    ++valid;
    if (result.track_id != last_id) ++ids;
    last_id = result.track_id;
    if (!have_reference) reference_yaw = result.pose.yaw;
    have_reference = true;
    worst_yaw = std::max(worst_yaw, std::abs(std::remainder(result.pose.yaw - reference_yaw, 2 * M_PI)));
  }
  check(valid == total, "a visible robot is tracked on every scan");
  check(ids == 1, "a visible robot keeps one track");
  check(worst_yaw < .15, "yaw of a robot moving straight does not drift");
}

void test_recovers_after_short_occlusion_with_turn() {
  op::Tracker tracker(config());
  double stamp = 1;
  int scan = 0;
  RobotPose robot{-1, 0, 0};
  acquire(tracker, stamp, robot, scan, 3);
  const auto before = tracker.process(scan_robot(robot, scan++), stamp, sensor);
  check(before.body_valid, "robot tracked before occlusion");
  for (int i = 0; i < 25; ++i, stamp += scan_dt, ++scan) tracker.process({}, stamp, sensor);
  // Outside the near-prediction yaw seeds, so only the heading sweep can find it.
  robot.yaw += 1.2;
  robot.x += .2;
  const double recovery = recovery_time(tracker, stamp, robot, scan, before.track_id, 2.);
  check(recovery >= 0, "turned robot recovers on the same track");
  check(recovery < .5, "recovery after a visible return takes under 0.5 s");
}

void test_reidentifies_after_track_timeout() {
  auto cfg = config();
  op::Tracker tracker(cfg);
  double stamp = 1;
  int scan = 0;
  RobotPose robot{-1, 0, 0};
  acquire(tracker, stamp, robot, scan, 3);
  const auto before = tracker.process(scan_robot(robot, scan++), stamp, sensor);
  check(before.body_valid, "robot tracked before long occlusion");
  // Beyond the reset deadline, the robot having moved across the field meanwhile.
  const double gap = cfg.reset_s + 1.;
  for (double t = 0; t < gap; t += scan_dt, stamp += scan_dt, ++scan)
    tracker.process({}, stamp, sensor);
  robot.x += 1.5;
  robot.y += .5;
  robot.yaw -= 1.;
  const double recovery = recovery_time(tracker, stamp, robot, scan, before.track_id, 1.);
  check(recovery >= 0, "the same model and id return after the reset deadline");
  check(recovery < .5, "reidentification takes under 0.5 s");
}

void test_static_fixture_does_not_take_the_track() {
  op::Tracker tracker(config());
  double stamp = 1;
  int scan = 0;
  op::Points fixture;
  for (int i = 0; i < 6; ++i)
    for (int level = 0; level < 5; ++level)
      fixture.emplace_back(2.75 + .03 * i, 1.37, .25 + .1 * level);
  // The fixture is seen alone long enough to be learned as one.
  for (double t = 0; t < 4; t += scan_dt, ++scan, stamp += scan_dt)
    tracker.process(fixture, stamp, sensor);
  RobotPose robot{-2.5, 0, 0};
  int on_robot = 0, on_fixture = 0;
  for (double t = 0; t < 4; t += scan_dt, ++scan, stamp += scan_dt) {
    robot.x += .4 * scan_dt;
    auto points = scan_robot(robot, scan);
    points.insert(points.end(), fixture.begin(), fixture.end());
    const auto result = tracker.process(points, stamp, sensor);
    if (t < 2 || !result.body_valid) continue;
    const bool near_robot = (result.pose.xy - Eigen::Vector2d(robot.x, robot.y)).norm() < 1.;
    near_robot ? ++on_robot : ++on_fixture;
  }
  check(on_robot > 50, "the robot is tracked beside a static fixture");
  check(on_fixture == 0, "the track never sits on the static fixture");
}
}  // namespace

// The bucket on a robot that stands 5 m away and only shows sparse, noisy returns.
op::Points scan_bucket(const RobotPose& robot, const Eigen::Vector2d& mount, double floor,
                       std::mt19937& rng) {
  std::uniform_real_distribution<double> unit(0, 1);
  std::normal_distribution<double> noise(0, .022);
  op::Points points;
  const op::Config c;
  const Eigen::Vector2d axis =
      Eigen::Vector2d(robot.x, robot.y) + Eigen::Rotation2Dd(robot.yaw) * mount;
  for (int i = 0; i < 2; ++i) {
    const double height = .06 + unit(rng) * (c.bottom.bucket_height - .06);
    const double radius = c.bottom.floor_radius +
                          (c.nominal_radius - c.bottom.floor_radius) * height / c.bottom.bucket_height;
    const double bearing = unit(rng) * 2 * M_PI;
    const double r = unit(rng) < .3 ? radius * std::sqrt(unit(rng)) : radius + noise(rng);
    points.emplace_back(axis.x() + r * std::cos(bearing), axis.y() + r * std::sin(bearing),
                        floor + height);
  }
  return points;
}

void test_far_bucket_is_recognized() {
  op::Config c = config();
  op::Tracker tracker(c);
  double stamp = 1;
  int scan = 0;
  RobotPose robot{0, 0, 0};
  acquire(tracker, stamp, robot, scan, 1.);
  std::mt19937 rng(5);
  const Eigen::Vector2d mount(.1, .05);
  const double floor = 1.25;
  op::Result last;
  double first_valid = -1;
  const double start = stamp;
  for (; stamp - start < 8; stamp += scan_dt, ++scan) {
    auto points = scan_robot(robot, scan);
    const auto bucket = scan_bucket(robot, mount, floor, rng);
    points.insert(points.end(), bucket.begin(), bucket.end());
    const auto result = tracker.process(points, stamp, sensor);
    if (result.bottom_valid && first_valid < 0) first_valid = stamp - start;
    if (result.bottom_valid) last = result;
  }
  check(first_valid >= 0, "a far bucket on a standing robot is recognized");
  check(first_valid < 5, "it is recognized within a few seconds");
  check(last.bottom_tracking_mode == op::BottomMode::TopFitted, "it comes from the top fit");
  if (last.bottom) {
    const op::Point world = last.pose.world(last.bottom->center);
    const Eigen::Vector2d truth =
        Eigen::Vector2d(robot.x, robot.y) + Eigen::Rotation2Dd(robot.yaw) * mount;
    check((world.head<2>() - truth).norm() < .05, "its position is right within 5 cm");
    check(std::abs(world.z() - floor) < .04, "its floor height is right within 4 cm");
  }
}

int main() {
  test_tracks_continuously();
  test_recovers_after_short_occlusion_with_turn();
  test_reidentifies_after_track_timeout();
  test_static_fixture_does_not_take_the_track();
  test_far_bucket_is_recognized();
  if (failures) return 1;
  std::cout << "opening_tracker_test passed\n";
  return 0;
}
