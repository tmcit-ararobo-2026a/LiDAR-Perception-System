#include "lidar_perception_system/support_candidates.hpp"

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

namespace lps = lidar_perception_system;

std::vector<lps::GridCell> cluster(const lps::GridConfig & config, double x)
{
  std::vector<lps::GridCell> grid(config.width * config.height);
  const int middle_x = static_cast<int>((x - config.min_x) / config.resolution);
  const int middle_y = static_cast<int>((0.0 - config.min_y) / config.resolution);
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      auto & cell = grid[(middle_y + dy) * config.width + middle_x + dx];
      cell.count = 3;
      cell.sum_x_mm = static_cast<int64_t>(std::lround(x * 3000.0));
      cell.sum_y_mm = 0;
      cell.sum_z_mm = 2100;
    }
  }
  return grid;
}

int main()
{
  lps::GridConfig config;
  config.min_x = -2.0F;
  config.min_y = -2.0F;
  config.resolution = 0.1F;
  config.width = 40;
  config.height = 40;
  const std::vector<lps::GridCell> empty(config.width * config.height);

  lps::SupportTracker stationary;
  for (int frame = 0; frame < 10; ++frame) {
    if (stationary.observe(cluster(config, -1.0), config, frame * 0.1)) {
      std::cerr << "stationary fixture was acquired\n";
      return 1;
    }
  }

  lps::SupportTracker moving;
  moving.set_exclusions({Eigen::Vector3d(0.9, 0.0, 0.45)});
  bool acquired = false;
  for (int frame = 0; frame <= 4; ++frame) {
    auto result = moving.observe(cluster(config, frame * 0.1), config, frame * 0.1);
    acquired = acquired || result.has_value();
  }
  if (!acquired) {
    std::cerr << "moving target was not acquired\n";
    return 1;
  }
  for (int frame = 5; frame <= 11; ++frame) {
    if (moving.observe(empty, config, frame * 0.1)) {
      std::cerr << "missing observation was reported as measured\n";
      return 1;
    }
  }
  auto recovered = moving.observe(cluster(config, 0.9), config, 1.2,
    Eigen::Vector2d(0.9, 0.0));
  if (!recovered || (recovered->xy - Eigen::Vector2d(0.9, 0.0)).norm() > 0.05) {
    std::cerr << "target was not reacquired near static fixture\n";
    return 1;
  }
  return 0;
}
