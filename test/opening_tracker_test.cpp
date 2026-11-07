// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
// Tracking behaviour on a synthetic box robot seen by a sparse scanner.
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
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

void test_bucket_lift_is_followed() {
  op::Config c = config();
  op::Tracker tracker(c);
  double stamp = 1;
  int scan = 0;
  RobotPose robot{0, 0, 0};
  acquire(tracker, stamp, robot, scan, 1.);
  std::mt19937 rng(6);
  const Eigen::Vector2d mount(.1, .05);
  const double start = stamp;
  double floor = 1.25, largest_error = 0;
  bool learned = false;
  for (; stamp - start < 16; stamp += scan_dt, ++scan) {
    const double t = stamp - start;
    // Up by 25 cm after the mount is known, and back down again.
    floor = 1.25 + std::clamp((t - 7) * .2, 0., .25) - std::clamp((t - 11) * .2, 0., .25);
    auto points = scan_robot(robot, scan);
    const auto bucket = scan_bucket(robot, mount, floor, rng);
    points.insert(points.end(), bucket.begin(), bucket.end());
    const auto result = tracker.process(points, stamp, sensor);
    learned = learned || result.tracked_bottom.has_value();
    if (t > 7 && result.tracked_bottom)
      largest_error = std::max(largest_error, std::abs(result.tracked_bottom->center.z() - floor));
  }
  check(learned, "the mount is learned before the lift");
  check(largest_error < .2, "the bucket height follows a lift");
  check(largest_error > 0, "the height was checked during the lift");
}

// A round chassis shows no heading, so its registered pose cannot follow it as it turns. A bucket
// mounted off its centre then circles in the robot frame, as it does on a real robot seen in part.
op::Points scan_round_robot(const RobotPose& robot, int scan) {
  op::Points points;
  constexpr double radius = .35;
  const Eigen::Vector2d center(robot.x, robot.y);
  for (int i = 0; i < 32; ++i) {
    const double bearing = 2 * M_PI * (i + .5) / 32;
    const Eigen::Vector2d outward(std::cos(bearing), std::sin(bearing));
    const Eigen::Vector2d p = center + radius * outward;
    if (outward.dot(sensor - p) <= 0) continue;
    for (int level = 0; level < 5; ++level)
      if ((i + level + scan) % 2 == 0) points.emplace_back(p.x(), p.y(), .25 + .15 * level);
  }
  return points;
}

// The robot keeps moving and turning while its bucket lifts, so the bucket's returns pile up neither in
// the robot frame nor in the map. A guess at the bucket must follow it before a fit can see it.
struct TurningRun {
  double first_guess{-1}, first_valid{-1}, guess_error{0}, xy_error{1}, z_error{1};
  // Worst error of the tracked bottom in the first half second after recognition, and after that.
  double early_xy{0}, followed_xy{0}, followed_z{0};
};
TurningRun run_turning_robot(unsigned seed, double lift_rate = 1, double lift = .15) {
  op::Tracker tracker(config());
  double stamp = 1;
  int scan = 0;
  RobotPose robot{-2, 0, 0};
  std::mt19937 rng(seed);
  const Eigen::Vector2d mount(.25, 0);
  TurningRun run;
  double &first_guess = run.first_guess, &first_valid = run.first_valid, &guess_error = run.guess_error,
         &xy_error = run.xy_error, &z_error = run.z_error;
  for (double t = 0; t < 6; t += scan_dt, stamp += scan_dt, ++scan) {
    robot.x += .5 * scan_dt;
    robot.yaw += 1. * scan_dt;
    const double floor = 1.25 + lift * std::sin(lift_rate * t);
    auto points = scan_round_robot(robot, scan);
    const auto bucket = scan_bucket(robot, mount, floor, rng);
    points.insert(points.end(), bucket.begin(), bucket.end());
    const auto result = tracker.process(points, stamp, sensor);
    const Eigen::Vector2d truth =
        Eigen::Vector2d(robot.x, robot.y) + Eigen::Rotation2Dd(robot.yaw) * mount;
    if (const auto guess = tracker.candidate()) {
      if (first_guess < 0) first_guess = t;
      if (t - first_guess > 1)
        guess_error = std::max(guess_error, (guess->head<2>() - truth).norm());
    }
    if (result.bottom_valid && first_valid < 0) {
      first_valid = t;
      const op::Point world = result.pose.world(result.bottom->center);
      xy_error = (world.head<2>() - truth).norm();
      z_error = std::abs(world.z() - floor);
    }
    if (first_valid >= 0 && result.tracked_bottom) {
      const op::Point world = result.pose.world(result.tracked_bottom->center);
      const double e = (world.head<2>() - truth).norm();
      if (t - first_valid < .5) {
        run.early_xy = std::max(run.early_xy, e);
      } else {
        run.followed_xy = std::max(run.followed_xy, e);
        run.followed_z = std::max(run.followed_z, std::abs(world.z() - floor));
      }
    }
  }
  return run;
}
void test_bucket_on_turning_robot_is_recognized() {
  TurningRun worst;
  worst.xy_error = worst.z_error = 0;
  for (unsigned seed = 7; seed < 15; ++seed) {
    const auto run = run_turning_robot(seed);
    std::cout << "turning robot, seed " << seed << ": guess at " << run.first_guess
              << " s, worst guess error " << run.guess_error << " m, recognized at " << run.first_valid
              << " s, error " << run.xy_error << " m, " << run.z_error << " m, then " << run.early_xy
              << " m, " << run.followed_xy << " m, " << run.followed_z << " m\n";
    worst.first_guess = run.first_guess < 0 ? 99 : std::max(worst.first_guess, run.first_guess);
    worst.first_valid = run.first_valid < 0 ? 99 : std::max(worst.first_valid, run.first_valid);
    worst.guess_error = std::max(worst.guess_error, run.guess_error);
    worst.xy_error = std::max(worst.xy_error, run.xy_error);
    worst.z_error = std::max(worst.z_error, run.z_error);
    worst.early_xy = std::max(worst.early_xy, run.early_xy);
    worst.followed_xy = std::max(worst.followed_xy, run.followed_xy);
    worst.followed_z = std::max(worst.followed_z, run.followed_z);
  }
  std::cout << "turning robot, worst: recognized at " << worst.first_valid << " s, then " << worst.early_xy
            << " m, " << worst.followed_xy << " m, " << worst.followed_z << " m\n";
  check(worst.first_guess < 1, "a guess at the bucket starts within 1 s");
  check(worst.guess_error < .08, "the guess follows the bucket within 8 cm");
  check(worst.first_valid < 2, "the bucket on a turning robot is recognized within 2 s");
  check(worst.xy_error < .07, "it is recognized where it is, within 7 cm");
  check(worst.z_error < .05, "and at its height, within 5 cm");
  check(worst.early_xy < .068, "the followed axis carries on from the fit, within 6.8 cm");
  check(worst.followed_z < .075, "its height then follows the lift within 7.5 cm");
  // Fits during a lift of up to 0.9 m/s differ in height by more than fits of a still bucket do, and the
  // guess that follows the lift accounts for that.
  double fast = 0;
  for (unsigned seed = 7; seed < 15; ++seed) {
    const auto run = run_turning_robot(seed, 3, .3);
    fast = std::max(fast, run.first_valid < 0 ? 99. : run.first_valid);
  }
  std::cout << "turning robot, fast lift: recognized at worst " << fast << " s\n";
  check(fast < 2.3, "a bucket lifting fast on a turning robot is recognized within 2.3 s");
}

// Something held above the bucket for a moment, such as a ball, is the top of the robot while it lasts.
// A guess that started on it finds nothing there afterwards and must start over on the bucket.
void test_guess_starts_over_after_a_passing_object() {
  int failed = 0;
  double worst = 0;
  for (unsigned seed = 7; seed < 15; ++seed) {
    op::Tracker tracker(config());
    double stamp = 1;
    int scan = 0;
    RobotPose robot{-2, 0, 0};
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0, .02);
    // On opposite sides of the robot, further apart than a guess looks for its bucket.
    const Eigen::Vector2d mount(.3, .3), elsewhere(-.6, 0);
    double first_valid = -1;
    for (double t = 0; t < 6; t += scan_dt, stamp += scan_dt, ++scan) {
      robot.x += .5 * scan_dt;
      robot.yaw += 1. * scan_dt;
      auto points = scan_round_robot(robot, scan);
      if (t < .6) {
        const Eigen::Vector2d held =
            Eigen::Vector2d(robot.x, robot.y) + Eigen::Rotation2Dd(robot.yaw) * elsewhere;
        for (int i = 0; i < 6; ++i)
          points.emplace_back(held.x() + noise(rng), held.y() + noise(rng), 1.95 + noise(rng));
      }
      const auto bucket = scan_bucket(robot, mount, 1.25, rng);
      points.insert(points.end(), bucket.begin(), bucket.end());
      const auto result = tracker.process(points, stamp, sensor);
      if (result.bottom_valid && first_valid < 0) first_valid = t;
    }
    failed += first_valid < 0;
    worst = std::max(worst, first_valid < 0 ? 99. : first_valid);
  }
  std::cout << "passing object: recognized at worst " << worst << " s\n";
  check(!failed && worst < 3, "the bucket is recognized after an object above it has gone");
}

// A standing robot is recognized from a fit in its own frame. The guess that followed the bucket until
// then has settled, so the followed axis carries on from it rather than from the fit alone.
void test_recognized_bucket_carries_on_steadily() {
  double worst = 0;
  for (unsigned seed = 7; seed < 15; ++seed) {
    op::Tracker tracker(config());
    double stamp = 1;
    int scan = 0;
    RobotPose robot{0, 0, 0};
    acquire(tracker, stamp, robot, scan, 1.);
    std::mt19937 rng(seed);
    const Eigen::Vector2d mount(.1, .05);
    double first_valid = -1, path = 0;
    std::optional<Eigen::Vector2d> last;
    const double start = stamp;
    for (; stamp - start < 8; stamp += scan_dt, ++scan) {
      auto points = scan_robot(robot, scan);
      const auto bucket = scan_bucket(robot, mount, 1.25, rng);
      points.insert(points.end(), bucket.begin(), bucket.end());
      const auto result = tracker.process(points, stamp, sensor);
      if (result.bottom_valid && first_valid < 0) first_valid = stamp;
      if (first_valid < 0 || stamp - first_valid > 1 || !result.tracked_bottom) continue;
      const Eigen::Vector2d xy = result.pose.world(result.tracked_bottom->center).head<2>();
      if (last) path += (xy - *last).norm();
      last = xy;
    }
    worst = std::max(worst, first_valid < 0 ? 99. : path);
  }
  std::cout << "standing robot: path of the bucket in its first second, at worst " << worst << " m\n";
  check(worst < .3, "a recognized standing bucket moves little in its first second");
}

// A far robot is registered only now and then, while the top of its bucket keeps returning. The bucket
// is followed from those returns meanwhile, and stays reported where it is.
void test_bucket_is_followed_while_the_chassis_is_unseen() {
  double worst = 0, reported = 1;
  for (unsigned seed = 7; seed < 15; ++seed) {
    op::Tracker tracker(config());
    double stamp = 1;
    int scan = 0;
    RobotPose robot{-2, 0, 0};
    std::mt19937 rng(seed);
    const Eigen::Vector2d mount(.25, 0);
    double first_valid = -1;
    int gap_frames = 0, gap_followed = 0;
    for (double t = 0; t < 6; t += scan_dt, stamp += scan_dt, ++scan) {
      robot.x += .5 * scan_dt;
      robot.yaw += 1. * scan_dt;
      const bool gap = first_valid >= 0 && t > first_valid + 1 && t < first_valid + 2;
      op::Points points;
      if (!gap) points = scan_round_robot(robot, scan);
      const auto bucket = scan_bucket(robot, mount, 1.25, rng);
      points.insert(points.end(), bucket.begin(), bucket.end());
      const auto result = tracker.process(points, stamp, sensor);
      if (result.bottom_valid && first_valid < 0) first_valid = t;
      if (!gap) continue;
      ++gap_frames;
      if (!result.bottom_followed_without_body || !result.tracked_bottom) continue;
      ++gap_followed;
      const Eigen::Vector2d truth =
          Eigen::Vector2d(robot.x, robot.y) + Eigen::Rotation2Dd(robot.yaw) * mount;
      const Eigen::Vector2d seen = result.pose.world(result.tracked_bottom->center).head<2>();
      worst = std::max(worst, (seen - truth).norm());
    }
    reported = std::min(reported, gap_frames ? static_cast<double>(gap_followed) / gap_frames : 0.);
  }
  std::cout << "unseen chassis: bucket reported in at least " << reported << " of the gap, worst error "
            << worst << " m\n";
  check(reported > .8, "the bucket stays reported while the chassis is unseen");
  check(worst < .08, "and where it is, within 8 cm");
  // With the bucket gone from view as well, nothing shows where it is, and it is not reported for long.
  double longest = 0;
  for (unsigned seed = 7; seed < 15; ++seed) {
    op::Tracker tracker(config());
    double stamp = 1;
    int scan = 0;
    RobotPose robot{-2, 0, 0};
    std::mt19937 rng(seed);
    const Eigen::Vector2d mount(.25, 0);
    double first_valid = -1, hidden_since = -1, last_reported = -1;
    for (double t = 0; t < 6; t += scan_dt, stamp += scan_dt, ++scan) {
      robot.x += .5 * scan_dt;
      robot.yaw += 1. * scan_dt;
      const bool gap = first_valid >= 0 && t > first_valid + 1 && t < first_valid + 2;
      if (gap && hidden_since < 0) hidden_since = t;
      op::Points points;
      if (!gap) {
        points = scan_round_robot(robot, scan);
        const auto bucket = scan_bucket(robot, mount, 1.25, rng);
        points.insert(points.end(), bucket.begin(), bucket.end());
      }
      const auto result = tracker.process(points, stamp, sensor);
      if (result.bottom_valid && first_valid < 0) first_valid = t;
      if (gap && result.bottom_followed_without_body) last_reported = t;
    }
    if (hidden_since >= 0) longest = std::max(longest, last_reported < 0 ? 0. : last_reported - hidden_since);
  }
  std::cout << "hidden robot: bucket reported for at most " << longest << " s\n";
  check(longest < .4, "a bucket out of view is not reported for long");
}

// A track that loses its chassis for longer than it lives starts over, but the guess at its bucket is still
// being hit and needs no pose. It carries on, so the new track recognizes the bucket at once.
void test_guess_outlives_a_lost_track() {
  double worst = 0;
  for (unsigned seed = 7; seed < 15; ++seed) {
    op::Tracker tracker(config());
    double stamp = 1;
    int scan = 0;
    RobotPose robot{-2, 0, 0};
    std::mt19937 rng(seed);
    const Eigen::Vector2d mount(.25, 0);
    double first_valid = -1;
    for (double t = 0; t < 6; t += scan_dt, stamp += scan_dt, ++scan) {
      robot.x += .5 * scan_dt;
      robot.yaw += 1. * scan_dt;
      // Seen for a moment, then unseen for longer than a track lasts.
      op::Points points;
      if (t < .3 || t > 2.8) points = scan_round_robot(robot, scan);
      const auto bucket = scan_bucket(robot, mount, 1.25, rng);
      points.insert(points.end(), bucket.begin(), bucket.end());
      const auto result = tracker.process(points, stamp, sensor);
      if (result.bottom_valid && first_valid < 0) first_valid = t;
    }
    worst = std::max(worst, first_valid < 0 ? 99. : first_valid - 2.8);
  }
  std::cout << "lost track: recognized at worst " << worst << " s after the chassis reappeared\n";
  check(worst < .6, "a guess that outlived its track gives the new one the bucket at once");
}

// A transparent bucket returns mostly from inside and below its rim, so a short window can miss its top.
op::Points scan_clear_bucket(const RobotPose& robot, const Eigen::Vector2d& mount, double floor,
                             std::mt19937& rng) {
  std::uniform_real_distribution<double> unit(0, 1);
  std::normal_distribution<double> noise(0, .015);
  op::Points points;
  const op::Config c;
  const Eigen::Vector2d axis =
      Eigen::Vector2d(robot.x, robot.y) + Eigen::Rotation2Dd(robot.yaw) * mount;
  for (int i = 0; i < 2; ++i) {
    const double height = .06 + std::pow(unit(rng), 2.) * (c.bottom.bucket_height - .06);
    const double radius = c.bottom.floor_radius +
                          (c.nominal_radius - c.bottom.floor_radius) * height / c.bottom.bucket_height;
    const double bearing = unit(rng) * 2 * M_PI;
    const double r = unit(rng) < .6 ? radius * std::sqrt(unit(rng)) : radius + noise(rng);
    points.emplace_back(axis.x() + r * std::cos(bearing), axis.y() + r * std::sin(bearing),
                        floor + height);
  }
  return points;
}

// A still bucket gives its guess time, so a fit that disagrees with the guess has not seen the rim yet.
// Here the track starts over while the robot stands, so its frame has no rest, and the fit along the
// guess runs on a short window.
void test_still_bucket_is_recognized_at_its_height() {
  double worst = 0;
  for (unsigned seed = 7; seed < 15; ++seed) {
    op::Tracker tracker(config());
    double stamp = 1;
    int scan = 0;
    RobotPose robot{0, 0, 0};
    acquire(tracker, stamp, robot, scan, 1.);
    std::mt19937 rng(seed);
    const Eigen::Vector2d mount(.1, .05);
    constexpr double floor = 1.25;
    for (double t = 0; t < 2.5; t += scan_dt, stamp += scan_dt, ++scan)
      tracker.process(scan_clear_bucket(robot, mount, floor, rng), stamp, sensor);
    const double start = stamp;
    for (; stamp - start < 5; stamp += scan_dt, ++scan) {
      auto points = scan_robot(robot, scan);
      const auto bucket = scan_clear_bucket(robot, mount, floor, rng);
      points.insert(points.end(), bucket.begin(), bucket.end());
      const auto result = tracker.process(points, stamp, sensor);
      if (result.bottom_valid && result.tracked_bottom) {
        worst = std::max(worst, std::abs(result.pose.world(result.tracked_bottom->center).z() - floor));
        break;
      }
    }
  }
  std::cout << "still clear bucket: worst height at recognition " << worst << " m\n";
  check(worst < .05, "a still bucket is recognized at its height, within 5 cm");
}

// A kept guess belongs to whichever robot the next track finds only if it is on that robot. Here the first
// robot's chassis is lost behind something while its bucket stays in view, and another robot appears.
void test_kept_guess_stays_with_its_robot() {
  double nearest = 99;
  for (unsigned seed = 7; seed < 15; ++seed) {
    op::Tracker tracker(config());
    double stamp = 1;
    int scan = 0;
    RobotPose first{-2, 0, 0}, other{1.5, 1.5, .3};
    std::mt19937 rng(seed);
    const Eigen::Vector2d mount(.25, 0);
    for (double t = 0; t < 6; t += scan_dt, stamp += scan_dt, ++scan) {
      first.x += .3 * scan_dt;
      first.yaw += 1. * scan_dt;
      op::Points points;
      if (t < .3) points = scan_round_robot(first, scan);
      if (t > 2.8) points = scan_robot(other, scan);
      const auto bucket = scan_bucket(first, mount, 1.25, rng);
      points.insert(points.end(), bucket.begin(), bucket.end());
      const auto result = tracker.process(points, stamp, sensor);
      if (!result.bottom_valid || !result.bottom) continue;
      const Eigen::Vector2d bucket_axis =
          Eigen::Vector2d(first.x, first.y) + Eigen::Rotation2Dd(first.yaw) * mount;
      const Eigen::Vector2d seen = result.pose.world(result.bottom->center).head<2>();
      nearest = std::min(nearest, (seen - bucket_axis).norm());
    }
  }
  std::cout << "kept guess on another robot: closest recognized bucket to the first robot's " << nearest
            << " m\n";
  check(nearest > .5, "a guess is not handed to another robot");
}

int main() {
  test_tracks_continuously();
  test_recovers_after_short_occlusion_with_turn();
  test_reidentifies_after_track_timeout();
  test_static_fixture_does_not_take_the_track();
  test_far_bucket_is_recognized();
  test_bucket_lift_is_followed();
  test_bucket_on_turning_robot_is_recognized();
  test_guess_starts_over_after_a_passing_object();
  test_recognized_bucket_carries_on_steadily();
  test_bucket_is_followed_while_the_chassis_is_unseen();
  test_guess_outlives_a_lost_track();
  test_still_bucket_is_recognized_at_its_height();
  test_kept_guess_stays_with_its_robot();
  if (failures) return 1;
  std::cout << "opening_tracker_test passed\n";
  return 0;
}
