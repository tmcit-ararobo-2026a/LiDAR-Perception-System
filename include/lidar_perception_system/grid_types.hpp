#pragma once

#include <cstdint>

namespace lidar_perception_system
{

struct GridConfig
{
  float min_x{-5.8F};
  float max_x{5.8F};
  float min_y{-5.8F};
  float max_y{5.8F};
  float min_z{1.05F};
  float max_z{1.95F};
  float resolution{0.08F};
  float self_x{0.0F};
  float self_y{0.0F};
  float self_radius{0.65F};
  int width{145};
  int height{145};
};

struct GridCell
{
  unsigned int count{0};
  int64_t sum_x_mm{0};
  int64_t sum_y_mm{0};
  int64_t sum_z_mm{0};
};

}  // namespace lidar_perception_system
