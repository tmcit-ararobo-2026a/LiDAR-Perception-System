// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
// Following the bucket rim from sparse returns while it holds, rises and comes down.
#include <cmath>
#include <functional>
#include <iostream>
#include <random>
#include <vector>

#include "lidar_perception_system/rim_height_filter.hpp"

namespace op = lidar_perception_system::opening;

namespace {
int failures = 0;

void check(bool condition, const char* what) {
  if (condition) return;
  std::cerr << "FAILED: " << what << '\n';
  ++failures;
}

constexpr double scan_dt = .02, sigma = .011, start_rim = 1.5;

struct Scene {
  std::mt19937 rng{7};
  double wall_rate{60}, body_rate{120};  // Returns per second.
  std::vector<double> scan(double rim) {
    std::poisson_distribution<int> wall(wall_rate * scan_dt), body(body_rate * scan_dt);
    std::uniform_real_distribution<double> unit(0, 1);
    std::normal_distribution<double> noise(0, sigma);
    std::vector<double> heights;
    for (int i = wall(rng); i > 0; --i) heights.push_back(rim - .2 * unit(rng) + noise(rng));
    for (int i = body(rng); i > 0; --i) heights.push_back(.95 + (rim - .25 - .95) * unit(rng));
    return heights;
  }
};

struct Run {
  double largest_error{0}, end_error{0}, steady_largest{0};
};

// Runs a rim height over time. Errors are taken after the first second of settling, and
// steady_largest only from the given time on.
Run run(const std::function<double(double)>& rim_at, double seconds, double steady_from = 1e9,
        Scene scene = Scene{}) {
  op::RimHeightFilter filter(1.165, 2.185);
  filter.reset(rim_at(0), 100);
  Run result;
  for (double t = 0; t < seconds; t += scan_dt) {
    const double rim = rim_at(t);
    filter.update(100 + t, scene.scan(rim), sigma);
    const double error = std::abs(filter.rim() - rim);
    if (t > 1) result.largest_error = std::max(result.largest_error, error);
    if (t > steady_from) result.steady_largest = std::max(result.steady_largest, error);
    result.end_error = error;
  }
  return result;
}

double ramp(double t, double start, double amount, double speed) {
  return std::abs(amount) * std::clamp((t - start) * speed / std::abs(amount), 0., 1.) *
         (amount > 0 ? 1 : -1);
}

void test_holds_steadily() {
  const auto r = run([](double) { return start_rim; }, 15);
  check(r.largest_error < .05, "a held rim stays within 5 cm");
  check(r.end_error < .02, "a held rim settles within 2 cm");
}

void test_follows_a_rise() {
  const auto r = run([](double t) { return start_rim + ramp(t, 2, .3, .2); }, 8, 4.5);
  check(r.largest_error < .08, "a rise at 0.2 m/s is followed within 8 cm");
  check(r.steady_largest < .05, "the rim settles within 1 s after a rise");
}

void test_follows_a_fall() {
  const auto r = run([](double t) { return start_rim + .3 + ramp(t, 2, -.3, .2); }, 8, 5);
  check(r.largest_error < .15, "a fall at 0.2 m/s is followed within 15 cm");
  check(r.steady_largest < .04, "the rim settles within 1.5 s after a fall");
}

void test_follows_a_fast_rise() {
  const auto r = run([](double t) { return start_rim + ramp(t, 2, .4, 1.); }, 6, 3.5);
  check(r.steady_largest < .04, "the rim settles within 1 s after a rise at 1 m/s");
}

double triangle(double t, double low, double amplitude, double speed) {
  const double period = 2 * amplitude / speed;
  const double phase = std::fmod(t, period) / period;
  return low + amplitude * (phase < .5 ? 2 * phase : 2 - 2 * phase);
}

void test_follows_a_slow_lift_without_steps() {
  // Slow enough that a speed estimate is close to zero, which must not stop the following.
  const auto r = run([](double t) { return triangle(t, 1.6, .4, .08); }, 40);
  check(r.largest_error < .1, "a continuous lift at 0.08 m/s stays within 10 cm");
  check(r.end_error < .05, "a continuous lift at 0.08 m/s ends within 5 cm");
}

void test_settles_quickly_after_a_reversal() {
  // Up, then straight back down: the speed must not keep it going the old way.
  const auto r = run([](double t) { return start_rim + ramp(t, 2, .3, .2) + ramp(t, 3.5, -.3, .2); }, 7, 6);
  check(r.largest_error < .15, "a reversal at 0.2 m/s stays within 15 cm");
  check(r.steady_largest < .05, "the rim settles within 1 s after a reversal");
}

void test_comes_down_further_than_the_wall_at_once() {
  // A quick drop leaves no return anywhere near the old height, only those below it.
  const auto r = run([](double t) { return start_rim + .4 + ramp(t, 3, -.4, 1.); }, 7, 5);
  check(r.steady_largest < .04, "the rim settles within 1.5 s after a drop of 40 cm");
}

void test_one_burst_is_not_a_lift() {
  op::RimHeightFilter filter(1.165, 2.185);
  filter.reset(start_rim, 100);
  Scene scene;
  for (double t = 0; t < 3; t += scan_dt) filter.update(100 + t, scene.scan(start_rim), sigma);
  auto burst = scene.scan(start_rim);
  burst.insert(burst.end(), {1.8, 1.82, 1.85, 1.81});  // A passing cloth, in a single scan.
  filter.update(103, burst, sigma);
  for (double t = .02; t < 1; t += scan_dt) {
    filter.update(103 + t, scene.scan(start_rim), sigma);
    check(std::abs(filter.rim() - start_rim) < .08, "one burst of returns 30 cm above does not move the rim");
  }
}

void test_no_returns_holds() {
  op::RimHeightFilter filter(1.165, 2.185);
  filter.reset(start_rim, 100);
  for (double t = 0; t < 8; t += scan_dt) filter.update(100 + t, {}, sigma);
  check(filter.rim() == start_rim, "a robot out of sight keeps its height");
}

void test_chassis_alone_does_not_lower_a_rim_out_of_view() {
  op::RimHeightFilter filter(1.165, 2.185);
  filter.reset(start_rim, 100);
  Scene scene;
  for (double t = 0; t < 3; t += scan_dt) filter.update(100 + t, scene.scan(start_rim), sigma);
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> unit(0, 1);
  for (double t = 3; t < 12; t += scan_dt) {
    std::vector<double> chassis;
    for (int i = 0; i < 2; ++i) chassis.push_back(.95 + .25 * unit(rng));
    filter.update(100 + t, chassis, sigma, 1.3);  // The rim is above what the sensor can see.
  }
  check(std::abs(filter.rim() - start_rim) < .05, "a rim out of view keeps its height");
}

void test_chassis_returns_do_not_lower_a_rim_sitting_above_them() {
  // A wall that is rarely hit may go unseen for a while, which must not make the height creep
  // towards the chassis that is still being hit.
  op::RimHeightFilter filter(1.165, 2.185);
  filter.reset(start_rim, 100);
  Scene scene;
  scene.wall_rate = 5;
  for (double t = 0; t < 3; t += scan_dt) filter.update(100 + t, scene.scan(start_rim), sigma);
  const double before = filter.rim();
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> unit(0, 1);
  for (double t = 3; t < 3.9; t += scan_dt) {
    std::vector<double> chassis;
    for (int i = 0; i < 3; ++i) chassis.push_back(.95 + .25 * unit(rng));
    filter.update(100 + t, chassis, sigma);
    check(std::abs(filter.rim() - before) < .02, "a short silence of a sparse wall keeps the height");
  }
}

void test_old_returns_do_not_hold_a_rim_up() {
  // A drop from above: the older returns stay in the fit window but must not pull the height back.
  const auto r = run([](double t) { return start_rim + .3 + ramp(t, 2, -.3, .5); }, 6, 3.6);
  check(r.steady_largest < .05, "the rim stays down within 1 s after a fall at 0.5 m/s");
}

void test_sparse_rim_is_not_a_fall() {
  // Few rim returns, in bursts seconds apart, while the chassis keeps returning.
  op::RimHeightFilter filter(1.165, 2.185);
  filter.reset(start_rim, 100);
  Scene scene;
  scene.wall_rate = 10;
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> unit(0, 1);
  for (double t = 0; t < 30; t += scan_dt) {
    std::vector<double> heights;
    if (unit(rng) < .1) heights.push_back(.95 + .25 * unit(rng));
    if (std::fmod(t, 1.5) < scan_dt)
      for (int i = 0; i < 2; ++i) heights.push_back(start_rim - .05 * unit(rng));
    filter.update(100 + t, heights, sigma);
    check(std::abs(filter.rim() - start_rim) < .06, "a sparsely hit rim stays where it is");
  }
}

void test_a_rim_that_was_too_high_comes_down() {
  op::RimHeightFilter filter(1.165, 2.185);
  filter.reset(2.1, 100);
  Scene scene;
  for (double t = 0; t < 12; t += scan_dt) filter.update(100 + t, scene.scan(start_rim), sigma);
  check(std::abs(filter.rim() - start_rim) < .04, "a rim that was far too high comes down");
}

void test_range_is_respected() {
  op::RimHeightFilter filter(1.165, 2.185);
  filter.reset(9., 100);
  check(filter.rim() <= 2.185, "the rim stays inside the rulebook range");
  bool thrown = false;
  try {
    op::RimHeightFilter invalid(2., 1.);
  } catch (const std::invalid_argument&) {
    thrown = true;
  }
  check(thrown, "an invalid range is refused");
}
}  // namespace

int main() {
  test_holds_steadily();
  test_follows_a_rise();
  test_follows_a_fall();
  test_follows_a_fast_rise();
  test_follows_a_slow_lift_without_steps();
  test_settles_quickly_after_a_reversal();
  test_comes_down_further_than_the_wall_at_once();
  test_one_burst_is_not_a_lift();
  test_no_returns_holds();
  test_chassis_alone_does_not_lower_a_rim_out_of_view();
  test_chassis_returns_do_not_lower_a_rim_sitting_above_them();
  test_old_returns_do_not_hold_a_rim_up();
  test_sparse_rim_is_not_a_fall();
  test_a_rim_that_was_too_high_comes_down();
  test_range_is_respected();
  if (failures) {
    std::cerr << failures << " check(s) failed\n";
    return 1;
  }
  std::cout << "rim_height_filter_test passed\n";
  return 0;
}
