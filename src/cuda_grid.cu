#include "lidar_perception_system/cuda_grid.hpp"

#include <cuda_runtime.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace lidar_perception_system
{
namespace
{
void check(cudaError_t result, const char * operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(result));
  }
}

__device__ float coordinate(const unsigned char * point, int offset)
{
  float value;
  memcpy(&value, point + offset, sizeof(value));
  return value;
}

__global__ void accumulate(
  const unsigned char * input, int point_step, int row_step, int width, int points,
  int x_offset, int y_offset, int z_offset,
  const float * transform, GridConfig config, GridCell * cells)
{
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= points) {return;}
  const unsigned char * point = input + static_cast<size_t>(index / width) * row_step +
    static_cast<size_t>(index % width) * point_step;
  const float x = coordinate(point, x_offset);
  const float y = coordinate(point, y_offset);
  const float z = coordinate(point, z_offset);
  if (!isfinite(x) || !isfinite(y) || !isfinite(z)) {return;}
  const float px = transform[0] * x + transform[1] * y + transform[2] * z + transform[3];
  const float py = transform[4] * x + transform[5] * y + transform[6] * z + transform[7];
  const float pz = transform[8] * x + transform[9] * y + transform[10] * z + transform[11];
  if (px < config.min_x || px >= config.max_x || py < config.min_y || py >= config.max_y ||
    pz < config.min_z || pz >= config.max_z) {return;}
  const float dx = px - config.self_x;
  const float dy = py - config.self_y;
  if (dx * dx + dy * dy < config.self_radius * config.self_radius) {return;}
  const int gx = static_cast<int>((px - config.min_x) / config.resolution);
  const int gy = static_cast<int>((py - config.min_y) / config.resolution);
  if (gx < 0 || gx >= config.width || gy < 0 || gy >= config.height) {return;}
  GridCell & cell = cells[gy * config.width + gx];
  atomicAdd(&cell.count, 1U);
  atomicAdd(reinterpret_cast<unsigned long long *>(&cell.sum_x_mm),
    static_cast<unsigned long long>(llroundf(px * 1000.0F)));
  atomicAdd(reinterpret_cast<unsigned long long *>(&cell.sum_y_mm),
    static_cast<unsigned long long>(llroundf(py * 1000.0F)));
  atomicAdd(reinterpret_cast<unsigned long long *>(&cell.sum_z_mm),
    static_cast<unsigned long long>(llroundf(pz * 1000.0F)));
}
}  // namespace

CudaGrid::CudaGrid(std::size_t max_bytes) : max_bytes_(max_bytes)
{
  check(cudaMalloc(&input_, max_bytes_), "cudaMalloc(input)");
  try {
    check(cudaMalloc(&matrix_, 12 * sizeof(float)), "cudaMalloc(transform)");
    check(cudaStreamCreate(reinterpret_cast<cudaStream_t *>(&stream_)), "cudaStreamCreate");
  } catch (...) {
    if (matrix_) {cudaFree(matrix_);}
    cudaFree(input_);
    throw;
  }
}

CudaGrid::~CudaGrid()
{
  if (stream_) {cudaStreamDestroy(reinterpret_cast<cudaStream_t>(stream_));}
  if (cells_) {cudaFree(cells_);}
  if (matrix_) {cudaFree(matrix_);}
  if (input_) {cudaFree(input_);}
}

std::vector<GridCell> CudaGrid::process(
  const std::uint8_t * data, std::size_t bytes, std::size_t points,
  int point_step, int row_step, int width, int x_offset, int y_offset, int z_offset,
  const float matrix[12], const GridConfig & config)
{
  if (bytes > max_bytes_ || points > static_cast<size_t>(INT32_MAX)) {
    throw std::runtime_error("PointCloud2 exceeds configured CUDA input capacity");
  }
  const size_t cell_count = static_cast<size_t>(config.width) * config.height;
  if (cell_count > cell_capacity_) {
    if (cells_) {check(cudaFree(cells_), "cudaFree(cells)");}
    check(cudaMalloc(&cells_, cell_count * sizeof(GridCell)), "cudaMalloc(cells)");
    cell_capacity_ = cell_count;
  }
  auto stream = reinterpret_cast<cudaStream_t>(stream_);
  check(cudaMemcpyAsync(input_, data, bytes, cudaMemcpyHostToDevice, stream), "copy input");
  check(cudaMemsetAsync(cells_, 0, cell_count * sizeof(GridCell), stream), "clear grid");
  check(cudaMemcpyAsync(matrix_, matrix, 12 * sizeof(float), cudaMemcpyHostToDevice, stream),
    "copy transform");
  accumulate<<<(points + 255) / 256, 256, 0, stream>>>(
    static_cast<const unsigned char *>(input_), point_step, row_step, width,
    static_cast<int>(points),
    x_offset, y_offset, z_offset, static_cast<float *>(matrix_), config,
    static_cast<GridCell *>(cells_));
  check(cudaGetLastError(), "accumulate kernel");
  std::vector<GridCell> result(cell_count);
  check(cudaMemcpyAsync(result.data(), cells_, cell_count * sizeof(GridCell),
    cudaMemcpyDeviceToHost, stream), "copy grid");
  check(cudaStreamSynchronize(stream), "wait for grid");
  return result;
}

}  // namespace lidar_perception_system
