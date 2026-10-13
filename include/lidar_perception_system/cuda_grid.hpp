#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

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

class CudaGrid
{
public:
  explicit CudaGrid(std::size_t max_bytes);
  ~CudaGrid();
  CudaGrid(const CudaGrid &) = delete;
  CudaGrid & operator=(const CudaGrid &) = delete;

  // matrix: row-major 3x4 sensor-to-map transform.
  std::vector<GridCell> process(
    const std::uint8_t * data, std::size_t bytes, std::size_t points,
    int point_step, int row_step, int width, int x_offset, int y_offset, int z_offset,
    const float matrix[12], const GridConfig & config);

private:
  void * input_{nullptr};
  void * cells_{nullptr};
  void * matrix_{nullptr};
  void * stream_{nullptr};
  std::size_t max_bytes_{0};
  std::size_t cell_capacity_{0};
};

}  // namespace lidar_perception_system
