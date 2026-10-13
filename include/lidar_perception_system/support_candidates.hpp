#pragma once

#include "lidar_perception_system/cuda_grid.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <vector>

namespace lidar_perception_system
{

struct SupportCandidate
{
  Eigen::Vector2d xy{0.0, 0.0};
  unsigned int points{0};
  int id{0};
};

// Tracks compact lower-body returns. A moving hypothesis is required before
// lock-on, because fixed buckets and field fixtures can have the same shape.
class SupportTracker
{
public:
  void set_exclusions(const std::vector<Eigen::Vector3d> & exclusions)
  {
    exclusions_ = exclusions;
  }

  std::optional<SupportCandidate> observe(
    const std::vector<GridCell> & grid, const GridConfig & cfg, double stamp,
    const std::optional<Eigen::Vector2d> & hint = std::nullopt)
  {
    auto candidates = components(grid, cfg);
    candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
      [this, &hint](const auto & candidate) {
        // A locked moving target may pass close to a fixed fixture. Only its
        // predicted neighborhood can bypass the static exclusion mask.
        if (locked_id_ != 0 && hint && (candidate.xy - *hint).norm() < 0.45) {
          return false;
        }
        for (const auto & exclusion : exclusions_) {
          if ((candidate.xy - exclusion.head<2>()).norm() < exclusion.z()) {return true;}
        }
        return false;
      }), candidates.end());
    last_candidate_count_ = candidates.size();
    for (auto & track : tracks_) {track.matched = false;}
    for (auto & candidate : candidates) {
      Hypothesis * best = nullptr;
      double best_ratio = 1.0;
      for (auto & track : tracks_) {
        const double elapsed = stamp - track.last_stamp;
        if (track.matched || elapsed < 0.0 || elapsed > 1.5) {continue;}
        const Eigen::Vector2d predicted = track.xy + track.velocity * elapsed;
        const double gate = std::min(1.25, 0.55 + 0.55 * elapsed);
        const double ratio = (candidate.xy - predicted).norm() / gate;
        // Keep the established target from being stolen by a new hypothesis.
        const double ranked = track.id == locked_id_ ? ratio * 0.7 : ratio;
        if (ratio < 1.0 && ranked < best_ratio) {
          best = &track;
          best_ratio = ranked;
        }
      }
      if (best) {
        if (stamp - best->first_stamp > 2.0) {
          best->first_xy = best->xy;
          best->first_stamp = stamp;
        }
        const double elapsed = stamp - best->last_stamp;
        if (elapsed > 0.02) {
          const Eigen::Vector2d measured = (candidate.xy - best->xy) / elapsed;
          if (measured.norm() < 3.0) {
            best->velocity = 0.5 * best->velocity + 0.5 * measured;
          }
        }
        best->xy = candidate.xy;
        best->points = candidate.points;
        best->last_stamp = stamp;
        best->hits++;
        best->matched = true;
        candidate.id = best->id;
      } else {
        Hypothesis track;
        track.xy = candidate.xy;
        track.first_xy = candidate.xy;
        track.points = candidate.points;
        track.first_stamp = stamp;
        track.last_stamp = stamp;
        track.hits = 1;
        track.matched = true;
        track.id = next_id_++;
        candidate.id = track.id;
        tracks_.push_back(track);
      }
    }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
      [stamp](const auto & track) {return stamp - track.last_stamp > 2.0;}), tracks_.end());
    if (locked_id_ != 0) {
      for (const auto & candidate : candidates) {
        if (candidate.id == locked_id_) {return candidate;}
      }
      if (hint) {
        const SupportCandidate * recovered = nullptr;
        double best_distance = 0.55;
        for (const auto & candidate : candidates) {
          const double distance = (candidate.xy - *hint).norm();
          if (distance < best_distance) {
            best_distance = distance;
            recovered = &candidate;
          }
        }
        if (recovered) {
          locked_id_ = recovered->id;
          return *recovered;
        }
      }
      for (const auto & track : tracks_) {
        if (track.id == locked_id_) {return std::nullopt;}
      }
      locked_id_ = 0;
    }
    double best_motion = 0.25;
    for (const auto & candidate : candidates) {
      for (const auto & track : tracks_) {
        if (candidate.id != track.id || track.hits < 4) {continue;}
        const double elapsed = stamp - track.first_stamp;
        if (elapsed < 0.2 || elapsed > 2.0) {continue;}
        const double motion = (track.xy - track.first_xy).norm();
        if (motion > best_motion) {
          best_motion = motion;
          locked_id_ = track.id;
        }
      }
    }
    if (locked_id_ != 0) {
      for (const auto & candidate : candidates) {
        if (candidate.id == locked_id_) {return candidate;}
      }
    }
    return std::nullopt;
  }

  void reset()
  {
    tracks_.clear();
    locked_id_ = 0;
    last_candidate_count_ = 0;
  }

  size_t candidate_count() const {return last_candidate_count_;}

private:
  struct Hypothesis
  {
    Eigen::Vector2d xy{0.0, 0.0};
    Eigen::Vector2d first_xy{0.0, 0.0};
    Eigen::Vector2d velocity{0.0, 0.0};
    unsigned int points{0};
    double first_stamp{0.0};
    double last_stamp{0.0};
    int hits{0};
    int id{0};
    bool matched{false};
  };

  static std::vector<SupportCandidate> components(
    const std::vector<GridCell> & grid, const GridConfig & cfg)
  {
    const int width = cfg.width;
    const int height = cfg.height;
    std::vector<unsigned char> visited(grid.size(), 0);
    std::deque<int> frontier;
    std::vector<SupportCandidate> output;
    for (int start = 0; start < width * height; ++start) {
      if (visited[start] || grid[start].count == 0) {continue;}
      visited[start] = 1;
      frontier.push_back(start);
      unsigned int points = 0;
      int min_x = width, max_x = 0, min_y = height, max_y = 0;
      double sum_x = 0.0, sum_y = 0.0;
      while (!frontier.empty()) {
        const int index = frontier.front();
        frontier.pop_front();
        const auto & cell = grid[index];
        const int x = index % width;
        const int y = index / width;
        points += cell.count;
        sum_x += static_cast<double>(cell.sum_x_mm) / 1000.0;
        sum_y += static_cast<double>(cell.sum_y_mm) / 1000.0;
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {continue;}
            const int neighbor = ny * width + nx;
            if (!visited[neighbor] && grid[neighbor].count != 0) {
              visited[neighbor] = 1;
              frontier.push_back(neighbor);
            }
          }
        }
      }
      if (points < 20 || points > 1000) {continue;}
      const double x_span = (max_x - min_x + 1) * cfg.resolution;
      const double y_span = (max_y - min_y + 1) * cfg.resolution;
      if (x_span < 0.20 || y_span < 0.20 || x_span > 1.15 || y_span > 1.15) {continue;}
      output.push_back({Eigen::Vector2d(sum_x / points, sum_y / points), points, 0});
    }
    return output;
  }

  std::vector<Eigen::Vector3d> exclusions_;
  std::vector<Hypothesis> tracks_;
  int next_id_{1};
  int locked_id_{0};
  size_t last_candidate_count_{0};
};

}  // namespace lidar_perception_system
