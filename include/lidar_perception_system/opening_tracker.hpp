// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <Eigen/Core>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "lidar_perception_system/support_candidates.hpp"

namespace lidar_perception_system::opening {
using Point = Eigen::Vector3d;
using Points = std::vector<Point>;

struct Pose {
  Eigen::Vector2d xy{0, 0};
  double yaw{0};
  Point world(const Point& p) const;
  Point local(const Point& p) const;
};

struct BottomConfig {
  // Standard competition bucket dimensions, not a robot mounting position.
  double bucket_height{0.255}, height_tolerance{0.035};
  double min_radius{0.075}, max_radius{0.135};
  double plane_distance{0.012}, max_plane_rmse{0.008}, max_tilt_rad{0.1745329252};
  double connection_m{0.035}, side_tolerance{0.030}, min_side_span{0.12};
  // Zero retains a learned semantic mount while the same body is observed.
  double max_body_inference_s{0.0};
  int min_plane_points{20}, min_current_points{4}, min_side_points{6};
};

struct Config {
  double min_x{-5}, max_x{5}, min_y{-5}, max_y{5};
  // Fence inner faces in the supplied localization map (metres).
  double field_min_x{-5.25}, field_max_x{5.25}, field_min_y{-5.70}, field_max_y{5.70};
  double min_z{0.10}, max_z{2.20}, self_radius{0.8};
  double body_min_z{0.15}, body_max_z{1.0}, cluster_resolution{0.10};
  double tracking_body_max_z{1.8};
  double min_span{0.20}, max_span{1.85}, crop_margin{0.30};
  double max_body_side{1.20}, body_size_tolerance{0.05};
  int min_body_points{20};
  double body_voxel{0.03}, opening_voxel{0.01}, window_s{1.0};
  int max_body_points{2000}, max_model_points{20000}, max_frame_points{50000};
  double correspondence_m{0.15}, max_registration_rmse{0.05}, min_overlap{0.5};
  double min_support_spread_ratio{0.1};
  int icp_iterations{20};
  double min_opening_z{1.2}, max_opening_z{2.1};
  double min_radius{0.110}, max_radius{0.150}, nominal_radius{0.1365};
  double slice_m{0.020}, max_residual{0.015}, max_plane_rmse{0.004};
  double min_arc{2.09439510239}, height_change_m{0.03}, min_lip_width{0.003};
  int min_rim_points{12}, min_current_points{4}, min_side_points{8};
  int ransac_iterations{80}, confirm_hits{3};
  double prediction_s{0.3}, reset_s{2.0}, association_m{0.65}, body_recovery_s{2.0};
  double reidentification_s{15.0}, reidentification_m{3.0};
  BottomConfig bottom;
  bool inside_field(const Point& p) const;
  void validate() const;
};

struct Opening {
  Point center{0, 0, 0};
  double radius{0}, residual{0}, plane_rmse{0}, arc{0}, score{0};
  int points{0}, current_points{0};
};

struct Bottom {
  Point center{0, 0, 0}, normal{0, 0, 1};
  double radius{0}, plane_rmse{0}, boundary_residual{0}, score{0};
  int plane_points{0}, current_points{0}, side_points{0};
};

// Geometry evidence only: no fixed mounting coordinates or opening inference.
std::optional<Bottom> fit_bucket_bottom(const Points& accumulated, const Points& current,
                                        const Config& config);
std::optional<Bottom> track_bucket_bottom(const Bottom& learned, const Points& current,
                                          const Config& config);
enum class BottomMode { None = 0, Direct = 1, PlaneTracked = 2, BodyInferred = 3 };

struct FitStats {
  int max_slice_points{0}, radius_hypotheses{0}, geometry_hypotheses{0}, edge_hypotheses{0};
  int max_inliers{0};
  int interior_rejections{0};
  double max_arc{0};
};

struct Result {
  bool valid{false}, predicted{false}, body_valid{false};
  bool body_recovered{false};
  bool body_upper_assisted{false};
  bool body_reidentified{false};
  int recovery_attempts{0};
  bool motion_seeded{false};
  bool bottom_valid{false};
  double bottom_observation_stamp{0};
  std::optional<Bottom> bottom;
  BottomMode bottom_tracking_mode{BottomMode::None};
  std::optional<Bottom> tracked_bottom;
  double bottom_tracking_observation_stamp{0};
  unsigned int track_id{0};
  double stamp{0}, observation_stamp{0}, confidence{0}, position_variance{1};
  Point position{0, 0, 0}, velocity{0, 0, 0};
  Pose pose;
  std::optional<Opening> opening;
  std::string reason{"no target"};
  int candidates{0}, model_points{0};
  double registration_rmse{0}, overlap{0};
  FitStats fit;
};

// Core has no ROS/GPU dependencies. All times are measurement times in seconds.
class Tracker {
 public:
  explicit Tracker(Config config);
  Result process(const Points& map_points, double stamp, const Eigen::Vector2d& sensor_xy);
  void reset();
  void set_exclusions(const std::vector<Eigen::Vector3d>& masks);
  const Points& selected_cloud() const { return selected_; }
  const Points& accumulated_cloud() const { return accumulated_; }
  const Points& body_model() const { return reference_body_; }
  size_t upper_body_model_points() const { return reference_upper_.size(); }
  unsigned int height_resets() const { return height_resets_; }
  std::optional<Opening> fit_opening(const Points& accumulated, const Points& current,
                                     FitStats* stats = nullptr) const;

 private:
  struct Cluster {
    Points body;
    Eigen::Vector2d center;
    double min_x, max_x, min_y, max_y;
  };
  struct Frame {
    double stamp;
    Points body, upper;
  };
  struct LostModel {
    Points body;
    Pose pose;
    Bottom bottom;
    double body_stamp, bottom_stamp, attempt_stamp{-1}, pending_stamp{-1};
    unsigned int id;
    int hits{0};
    Pose pending_pose;
  };
  struct Reidentification {
    Cluster cluster;
    Pose pose;
    double rmse, overlap;
  };
  std::vector<Cluster> clusters(const Points& points) const;
  bool body_size_valid(const Points& points) const;
  bool register_body(const Points& body, const Pose& initial, Pose& output, double& rmse,
                     double& overlap, Points* supported = nullptr, bool use_upper = false) const;
  std::optional<Reidentification> reidentify(const std::vector<Cluster>& candidates, double stamp);
  Result missing(double stamp, const std::string& reason, Result result = Result{});
  void clear_model();
  void rebuild(double stamp);
  Config cfg_;
  lidar_perception_system::SupportTracker acquisition_;
  std::vector<Eigen::Vector3d> exclusions_;
  std::deque<Frame> frames_;
  Points selected_, accumulated_, reference_body_, reference_upper_;
  Pose pose_;
  Eigen::Vector2d body_velocity_{0, 0}, cluster_center_{0, 0};
  double yaw_rate_{0}, body_stamp_{0}, input_stamp_{-1};
  double acquired_stamp_{0}, rejection_until_{0};
  std::optional<Eigen::Vector2d> rejected_center_;
  bool have_body_{false}, identified_{false}, motion_seeded_{false}, confirmed_{false},
      height_pending_{false};
  int hits_{0};
  unsigned int next_id_{0}, track_id_{0}, height_resets_{0};
  std::optional<Opening> last_opening_;
  std::optional<Bottom> last_bottom_;
  std::optional<Bottom> learned_bottom_;
  std::optional<LostModel> lost_model_;
  double learned_bottom_stamp_{0};
  int bottom_hits_{0};
  double bottom_stamp_{0};
  Point position_{0, 0, 0}, measured_{0, 0, 0}, velocity_{0, 0, 0};
  double observation_stamp_{0}, variance_{1};
};
}  // namespace lidar_perception_system::opening
