#include "lidar_perception_system/cuda_grid.hpp"
#include "lidar_perception_system/support_candidates.hpp"
#include "lidar_perception_system/msg/moving_bucket_track.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lps = lidar_perception_system;
using TrackMsg = lps::msg::MovingBucketTrack;
using CloudMsg = sensor_msgs::msg::PointCloud2;

namespace
{
struct Candidate
{
  Eigen::Vector2d xy{0.0, 0.0};
  double z{0.0};
  double residual{0.0};
  double arc{0.0};
  unsigned int points{0};
  double score{0.0};
};

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::vector<Candidate> extract_candidates(
  const std::vector<lps::GridCell> & grid, const lps::GridConfig & cfg,
  double radius, unsigned int min_points, int min_cells, double max_residual,
  double min_arc)
{
  const int width = cfg.width;
  const int height = cfg.height;
  std::vector<unsigned char> visited(grid.size(), 0);
  std::vector<Candidate> candidates;
  std::deque<int> frontier;
  for (int start = 0; start < width * height; ++start) {
    if (visited[start] || grid[start].count < 2) {continue;}
    frontier.push_back(start);
    visited[start] = 1;
    std::vector<int> cells;
    unsigned int total_points = 0;
    int min_x = width, max_x = 0, min_y = height, max_y = 0;
    while (!frontier.empty()) {
      const int index = frontier.front();
      frontier.pop_front();
      cells.push_back(index);
      total_points += grid[index].count;
      const int x = index % width;
      const int y = index / width;
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
          if (!visited[neighbor] && grid[neighbor].count >= 2) {
            visited[neighbor] = 1;
            frontier.push_back(neighbor);
          }
        }
      }
    }
    if (total_points < min_points || static_cast<int>(cells.size()) < min_cells) {continue;}
    const double size_x = (max_x - min_x + 1) * cfg.resolution;
    const double size_y = (max_y - min_y + 1) * cfg.resolution;
    if (size_x > 0.65 || size_y > 0.65 || size_x < 0.12 || size_y < 0.12) {continue;}

    // Fixed-radius fit to cell means. The robust weight prevents dense outliers from
    // determining the center; cell weights are capped to avoid angular imbalance.
    Eigen::Vector2d center(0.0, 0.0);
    double sum_z = 0.0;
    for (int index : cells) {
      const auto & cell = grid[index];
      center.x() += static_cast<double>(cell.sum_x_mm) / (1000.0 * cell.count);
      center.y() += static_cast<double>(cell.sum_y_mm) / (1000.0 * cell.count);
      sum_z += static_cast<double>(cell.sum_z_mm) / 1000.0;
    }
    center /= static_cast<double>(cells.size());
    for (int iteration = 0; iteration < 10; ++iteration) {
      Eigen::Matrix2d h = Eigen::Matrix2d::Zero();
      Eigen::Vector2d g = Eigen::Vector2d::Zero();
      for (int index : cells) {
        const auto & cell = grid[index];
        const Eigen::Vector2d point(
          static_cast<double>(cell.sum_x_mm) / (1000.0 * cell.count),
          static_cast<double>(cell.sum_y_mm) / (1000.0 * cell.count));
        const Eigen::Vector2d delta = center - point;
        const double distance = delta.norm();
        if (distance < 1e-4) {continue;}
        const double error = distance - radius;
        const double weight = std::min(1.0, 0.04 / std::max(1e-6, std::abs(error)));
        const Eigen::Vector2d jacobian = delta / distance;
        h.noalias() += weight * jacobian * jacobian.transpose();
        g.noalias() += weight * error * jacobian;
      }
      if (h.determinant() < 1e-5) {break;}
      const Eigen::Vector2d step = h.ldlt().solve(g);
      center -= step;
      if (step.norm() < 1e-4) {break;}
    }
    if ((center.array() < -20.0).any() || (center.array() > 20.0).any()) {continue;}
    double squared_error = 0.0;
    std::vector<double> angles;
    for (int index : cells) {
      const auto & cell = grid[index];
      const Eigen::Vector2d point(
        static_cast<double>(cell.sum_x_mm) / (1000.0 * cell.count),
        static_cast<double>(cell.sum_y_mm) / (1000.0 * cell.count));
      const auto delta = point - center;
      const double error = delta.norm() - radius;
      squared_error += error * error;
      angles.push_back(std::atan2(delta.y(), delta.x()));
    }
    std::sort(angles.begin(), angles.end());
    double max_gap = angles.front() + 2.0 * M_PI - angles.back();
    for (size_t i = 1; i < angles.size(); ++i) {
      max_gap = std::max(max_gap, angles[i] - angles[i - 1]);
    }
    const double arc = 2.0 * M_PI - max_gap;
    const double residual = std::sqrt(squared_error / cells.size());
    if (arc < min_arc || residual > max_residual) {continue;}
    Candidate candidate;
    candidate.xy = center;
    candidate.z = sum_z / total_points;
    candidate.residual = residual;
    candidate.arc = arc;
    candidate.points = total_points;
    candidate.score = 2.0 * arc - 30.0 * residual + std::log1p(total_points);
    candidates.push_back(candidate);
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
    return a.score > b.score;
  });
  if (candidates.size() > 8) {candidates.resize(8);}
  return candidates;
}

int field_offset(const CloudMsg & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      field.count == 1) {return static_cast<int>(field.offset);}
  }
  return -1;
}
}  // namespace

class MovingBucketDetector : public rclcpp::Node
{
public:
  MovingBucketDetector() : Node("moving_bucket_detector")
  {
    const auto path = declare_parameter<std::string>("config_file", "");
    if (path.empty()) {throw std::runtime_error("config_file is required");}
    std::ifstream input(path);
    if (!input) {throw std::runtime_error("Cannot open config: " + path);}
    const auto config = nlohmann::json::parse(input);
    if (config.at("schema_version").get<int>() != 1) {
      throw std::runtime_error("Unsupported config schema_version");
    }
    const auto & bounds = config.at("grid");
    grid_config_.min_x = bounds.at("min_x").get<float>();
    grid_config_.max_x = bounds.at("max_x").get<float>();
    grid_config_.min_y = bounds.at("min_y").get<float>();
    grid_config_.max_y = bounds.at("max_y").get<float>();
    grid_config_.min_z = bounds.at("min_z").get<float>();
    grid_config_.max_z = bounds.at("max_z").get<float>();
    grid_config_.resolution = bounds.at("resolution").get<float>();
    grid_config_.self_radius = bounds.at("self_radius").get<float>();
    support_min_z_ = bounds.at("support_min_z").get<float>();
    support_max_z_ = bounds.at("support_max_z").get<float>();
    support_self_radius_ = bounds.at("support_self_radius").get<float>();
    if (grid_config_.resolution <= 0.0F || grid_config_.min_x >= grid_config_.max_x ||
      grid_config_.min_y >= grid_config_.max_y || grid_config_.min_z >= grid_config_.max_z ||
      support_min_z_ >= support_max_z_ || support_self_radius_ < 0.0F) {
      throw std::runtime_error("Invalid grid bounds");
    }
    grid_config_.width = static_cast<int>(std::ceil(
      (grid_config_.max_x - grid_config_.min_x) / grid_config_.resolution));
    grid_config_.height = static_cast<int>(std::ceil(
      (grid_config_.max_y - grid_config_.min_y) / grid_config_.resolution));
    if (static_cast<int64_t>(grid_config_.width) * grid_config_.height > 300000) {
      throw std::runtime_error("Grid exceeds maximum cell count");
    }
    target_frame_ = config.at("target_frame").get<std::string>();
    radius_ = config.at("bucket").at("radius").get<double>();
    support_target_z_ = config.at("bucket").at("support_target_z").get<double>();
    min_points_ = config.at("bucket").at("min_points").get<unsigned int>();
    min_cells_ = config.at("bucket").at("min_cells").get<int>();
    max_residual_ = config.at("bucket").at("max_fit_residual").get<double>();
    min_arc_ = config.at("bucket").at("min_arc_span_rad").get<double>();
    gate_ = config.at("tracking").at("gate_m").get<double>();
    confirm_hits_ = config.at("tracking").at("confirm_hits").get<int>();
    prediction_s_ = config.at("tracking").at("max_prediction_s").get<double>();
    reset_s_ = config.at("tracking").at("reset_s").get<double>();
    tf_wait_s_ = config.at("tf_wait_s").get<double>();
    queue_size_ = config.at("queue_size").get<size_t>();
    publish_tf_ = config.at("publish_tf").get<bool>();
    const auto max_bytes = config.at("max_input_bytes").get<size_t>();
    if (radius_ <= 0.0 || support_target_z_ <= 0.0 || gate_ <= 0.0 ||
      queue_size_ == 0 || queue_size_ > 32 ||
      max_bytes < 1024 || max_bytes > 128 * 1024 * 1024) {
      throw std::runtime_error("Invalid detector configuration");
    }
    gpu_ = std::make_unique<lps::CudaGrid>(max_bytes);
    max_bytes_ = max_bytes;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    target_pub_ = create_publisher<TrackMsg>("~/target", 10);
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 10);
    static_tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",
      rclcpp::QoS(1).transient_local().reliable(),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto & transform : msg->transforms) {
          if (transform.header.frame_id != target_frame_) {continue;}
          const auto & child = transform.child_frame_id;
          double radius = 0.0;
          if (child.rfind("desk_", 0) == 0) {radius = 0.65;}
          else if (child.rfind("bucket_", 0) == 0) {radius = 0.45;}
          else if (child.rfind("flag_", 0) == 0) {radius = 0.50;}
          if (radius == 0.0) {continue;}
          bool known = false;
          for (auto & item : static_exclusions_) {
            if (item.first == child) {
              item.second = Eigen::Vector3d(transform.transform.translation.x,
                transform.transform.translation.y, radius);
              known = true;
              break;
            }
          }
          if (!known) {
            static_exclusions_.push_back({child, Eigen::Vector3d(
              transform.transform.translation.x, transform.transform.translation.y, radius)});
          }
        }
        std::vector<Eigen::Vector3d> masks;
        for (const auto & item : static_exclusions_) {masks.push_back(item.second);}
        support_tracker_.set_exclusions(masks);
      });
    sub_ = create_subscription<CloudMsg>(config.at("input_topic").get<std::string>(),
      rclcpp::SensorDataQoS(),
      [this](CloudMsg::ConstSharedPtr msg) {on_cloud(std::move(msg));});
    process_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {process_queue();});
    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_diagnostics();});
    RCLCPP_INFO(get_logger(), "CUDA moving bucket detector ready: %dx%d grid", grid_config_.width,
      grid_config_.height);
  }

private:
  struct QueuedCloud
  {
    CloudMsg::ConstSharedPtr cloud;
    std::chrono::steady_clock::time_point received;
  };

  void on_cloud(CloudMsg::ConstSharedPtr cloud)
  {
    ++received_;
    const double stamp = stamp_seconds(cloud->header.stamp);
    if (stamp + 0.01 < last_input_stamp_) {
      queue_.clear();
      has_track_ = false;
      hits_ = 0;
      support_tracker_.reset();
    }
    last_input_stamp_ = stamp;
    if (queue_.size() >= queue_size_) {
      queue_.pop_front();
      ++queue_drops_;
    }
    queue_.push_back({std::move(cloud), std::chrono::steady_clock::now()});
  }

  void process_queue()
  {
    if (queue_.empty()) {return;}
    auto & oldest = queue_.front();
    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - oldest.received).count();
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(target_frame_, oldest.cloud->header.frame_id,
        rclcpp::Time(oldest.cloud->header.stamp), rclcpp::Duration::from_seconds(0.0));
    } catch (const tf2::TransformException &) {
      if (age >= tf_wait_s_) {
        auto cloud = oldest.cloud;
        queue_.pop_front();
        ++tf_drops_;
        // A short TF outage must not erase an otherwise confirmed map track.
        // The output remains explicitly PREDICTED, never a new observation.
        if (has_track_ && hits_ >= confirm_hits_ &&
          stamp_seconds(cloud->header.stamp) - last_observation_ <= prediction_s_) {
          update_track(cloud->header, {}, TrackMsg::NONE);
        } else {
          publish_invalid(cloud->header, "TF unavailable");
        }
      }
      return;
    }
    auto cloud = oldest.cloud;
    queue_.pop_front();
    last_tf_wait_ms_ = age * 1000.0;
    tf_wait_samples_.push_back(last_tf_wait_ms_);
    if (tf_wait_samples_.size() > 1000) {
      tf_wait_samples_.pop_front();
    }
    const auto started = std::chrono::steady_clock::now();
    try {
      process_cloud(*cloud, transform);
      ++processed_;
    } catch (const std::exception & e) {
      ++processing_errors_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Frame rejected: %s", e.what());
      publish_invalid(cloud->header, e.what());
    }
    last_processing_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    processing_samples_.push_back(last_processing_ms_);
    if (processing_samples_.size() > 1000) {
      processing_samples_.pop_front();
    }
  }

  void process_cloud(const CloudMsg & cloud, const geometry_msgs::msg::TransformStamped & tf)
  {
    if (cloud.width == 0 || cloud.height == 0 || cloud.width > INT32_MAX ||
      cloud.is_bigendian || cloud.point_step < 12 || cloud.data.size() > max_bytes_ ||
      static_cast<uint64_t>(cloud.row_step) <
        static_cast<uint64_t>(cloud.width) * cloud.point_step ||
      static_cast<uint64_t>(cloud.row_step) * cloud.height > cloud.data.size()) {
      throw std::runtime_error("Unsupported or oversized PointCloud2 layout");
    }
    const int x_offset = field_offset(cloud, "x");
    const int y_offset = field_offset(cloud, "y");
    const int z_offset = field_offset(cloud, "z");
    if (x_offset < 0 || y_offset < 0 || z_offset < 0 ||
      x_offset + 4 > static_cast<int>(cloud.point_step) ||
      y_offset + 4 > static_cast<int>(cloud.point_step) ||
      z_offset + 4 > static_cast<int>(cloud.point_step)) {
      throw std::runtime_error("PointCloud2 requires FLOAT32 x/y/z fields");
    }
    const auto & q = tf.transform.rotation;
    const Eigen::Quaternionf rotation(q.w, q.x, q.y, q.z);
    if (rotation.norm() < 0.5F) {throw std::runtime_error("Invalid TF quaternion");}
    const Eigen::Matrix3f matrix = rotation.normalized().toRotationMatrix();
    const auto & t = tf.transform.translation;
    const float m[12] = {
      matrix(0, 0), matrix(0, 1), matrix(0, 2), static_cast<float>(t.x),
      matrix(1, 0), matrix(1, 1), matrix(1, 2), static_cast<float>(t.y),
      matrix(2, 0), matrix(2, 1), matrix(2, 2), static_cast<float>(t.z)};
    grid_config_.self_x = static_cast<float>(t.x);
    grid_config_.self_y = static_cast<float>(t.y);
    const auto cells = gpu_->process(cloud.data.data(), cloud.data.size(),
      static_cast<size_t>(cloud.width) * cloud.height, cloud.point_step,
      cloud.row_step, cloud.width, x_offset, y_offset, z_offset, m, grid_config_);
    auto support_grid_config = grid_config_;
    support_grid_config.min_z = support_min_z_;
    support_grid_config.max_z = support_max_z_;
    support_grid_config.self_radius = support_self_radius_;
    const auto support_grid = gpu_->process(cloud.data.data(), cloud.data.size(),
      static_cast<size_t>(cloud.width) * cloud.height, cloud.point_step,
      cloud.row_step, cloud.width, x_offset, y_offset, z_offset, m, support_grid_config);
    const double stamp = stamp_seconds(cloud.header.stamp);
    std::optional<Eigen::Vector2d> hint;
    if (has_track_ && stamp - last_observation_ <= reset_s_) {
      const double dt = std::clamp(stamp - state_stamp_, 0.0, 1.5);
      hint = state_.head<2>() + state_.tail<2>() * dt;
    }
    const auto support = support_tracker_.observe(support_grid, support_grid_config,
      stamp, hint);
    if (support_tracker_.candidate_count() > 0) {++support_candidate_frames_;}
    if (support) {++support_association_frames_;}
    std::vector<Candidate> candidates;
    uint8_t source_mode = TrackMsg::NONE;
    if (support) {
      Candidate candidate;
      candidate.xy = support->xy;
      candidate.z = support_target_z_;
      candidate.points = support->points;
      candidate.score = 1.0;
      candidates.push_back(candidate);
      source_mode = TrackMsg::SUPPORT_INFERRED;
    } else if (hint) {
      candidates = extract_candidates(cells, grid_config_, radius_, min_points_,
        min_cells_, max_residual_, min_arc_);
      candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
        [&hint](const Candidate & candidate) {
          return candidate.residual > 0.035 || (candidate.xy - *hint).norm() > 0.4;
        }), candidates.end());
      if (!candidates.empty()) {source_mode = TrackMsg::DIRECT;}
    }
    update_track(cloud.header, candidates, source_mode);
  }

  void update_track(const std_msgs::msg::Header & header,
    const std::vector<Candidate> & candidates, uint8_t source_mode)
  {
    const double stamp = stamp_seconds(header.stamp);
    double dt = has_track_ ? std::clamp(stamp - state_stamp_, 0.0, 1.5) : 0.0;
    if (has_track_) {
      Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
      transition(0, 2) = dt;
      transition(1, 3) = dt;
      state_ = transition * state_;
      covariance_ = transition * covariance_ * transition.transpose() +
        Eigen::Matrix4d::Identity() * (0.01 + 0.1 * dt);
      state_stamp_ = stamp;
      if (stamp - last_observation_ > reset_s_) {
        has_track_ = false;
        hits_ = 0;
      }
    }
    const Candidate * selected = nullptr;
    double best = -std::numeric_limits<double>::infinity();
    for (const auto & candidate : candidates) {
      const double distance = has_track_ ? (candidate.xy - state_.head<2>()).norm() : 0.0;
      const double association_gate = std::min(1.3, gate_ +
        0.5 * std::max(0.0, stamp - last_observation_));
      if (has_track_ && distance > association_gate) {continue;}
      const double score = candidate.score - 8.0 * distance;
      if (score > best) {best = score; selected = &candidate;}
    }
    if (selected) {
      if (!has_track_) {
        state_ << selected->xy.x(), selected->xy.y(), 0.0, 0.0;
        covariance_ = Eigen::Matrix4d::Identity() * 0.1;
        state_stamp_ = stamp;
        has_track_ = true;
        hits_ = 1;
        ++track_id_;
      } else {
        const Eigen::Matrix<double, 2, 4> h = (Eigen::Matrix<double, 2, 4>() <<
          1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished();
        const double observation_variance = source_mode == TrackMsg::SUPPORT_INFERRED ?
          0.04 : std::max(0.0025, selected->residual * selected->residual * 4.0);
        const Eigen::Matrix2d r = Eigen::Matrix2d::Identity() * observation_variance;
        const Eigen::Matrix2d s = h * covariance_ * h.transpose() + r;
        const Eigen::Matrix<double, 4, 2> k = covariance_ * h.transpose() * s.inverse();
        state_ += k * (selected->xy - h * state_);
        const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
        covariance_ = (identity - k * h) * covariance_ * (identity - k * h).transpose() +
          k * r * k.transpose();
        hits_ = std::min(hits_ + 1, confirm_hits_);
      }
      z_ = selected->z;
      last_observation_ = stamp;
      last_observation_stamp_ = header.stamp;
      last_points_ = selected->points;
      last_residual_ = selected->residual;
      if (hits_ >= confirm_hits_) {++detections_;}
    }
    const bool confirmed = has_track_ && hits_ >= confirm_hits_;
    const bool predicted = confirmed && !selected && stamp - last_observation_ <= prediction_s_;
    const bool valid = confirmed && (selected || predicted);
    last_status_reason_ = valid ? "" : "unconfirmed / no candidate";
    publish_track(header, valid, predicted ? TrackMsg::PREDICTED :
      (selected && confirmed ? source_mode : TrackMsg::NONE));
  }

  void publish_invalid(const std_msgs::msg::Header & header, const std::string & reason)
  {
    last_status_reason_ = reason;
    publish_track(header, false, TrackMsg::NONE);
  }

  void publish_track(const std_msgs::msg::Header & header, bool valid, uint8_t mode)
  {
    TrackMsg output;
    output.header = header;
    output.header.frame_id = target_frame_;
    output.last_observation_stamp = last_observation_stamp_;
    output.track_id = track_id_;
    output.valid = valid;
    output.observation_mode = mode;
    if (valid) {
      output.position.x = state_.x();
      output.position.y = state_.y();
      output.position.z = z_;
      output.velocity.x = state_(2);
      output.velocity.y = state_(3);
      const int order[4] = {0, 1, 3, 4};
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          output.state_covariance[order[r] * 6 + order[c]] = covariance_(r, c);
        }
      }
      output.state_covariance[2 * 6 + 2] = mode == TrackMsg::DIRECT ? 0.04 : 0.25;
      output.state_covariance[5 * 6 + 5] = 1.0;
      output.confidence = mode == TrackMsg::DIRECT ?
        static_cast<float>(std::clamp(1.0 - last_residual_ / max_residual_, 0.0, 1.0)) :
        (mode == TrackMsg::SUPPORT_INFERRED ? 0.4F : 0.0F);
      output.support_points = mode == TrackMsg::PREDICTED ? 0 : last_points_;
      output.fit_residual = static_cast<float>(last_residual_);
    }
    target_pub_->publish(output);
    if (marker_pub_->get_subscription_count() > 0) {
      publish_markers(output);
    }
    if (valid && mode == TrackMsg::DIRECT && publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = "moving_bucket";
      transform.transform.translation.x = output.position.x;
      transform.transform.translation.y = output.position.y;
      transform.transform.translation.z = output.position.z;
      transform.transform.rotation.w = 1.0;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void publish_markers(const TrackMsg & track)
  {
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;
    const auto make_marker = [&track](int id, int type) {
      Marker marker;
      marker.header = track.header;
      marker.ns = "moving_bucket_detector";
      marker.id = id;
      marker.type = type;
      marker.action = Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.lifetime.nanosec = 500000000;
      return marker;
    };
    const auto delete_marker = [&array, &make_marker](int id, int type) {
      auto marker = make_marker(id, type);
      marker.action = Marker::DELETE;
      array.markers.push_back(marker);
    };
    if (!track.valid) {
      trail_.clear();
      delete_marker(0, Marker::CYLINDER);
      delete_marker(1, Marker::ARROW);
      delete_marker(2, Marker::LINE_STRIP);
      delete_marker(3, Marker::TEXT_VIEW_FACING);
    } else {
      const bool direct = track.observation_mode == TrackMsg::DIRECT;
      const bool predicted = track.observation_mode == TrackMsg::PREDICTED;
      const float red = direct ? 0.15F : (predicted ? 0.25F : 1.0F);
      const float green = direct ? 0.95F : (predicted ? 0.55F : 0.7F);
      const float blue = direct ? 0.25F : (predicted ? 1.0F : 0.1F);
      auto target = make_marker(0, Marker::CYLINDER);
      target.pose.position = track.position;
      target.scale.x = radius_ * 2.0;
      target.scale.y = radius_ * 2.0;
      target.scale.z = 0.12;
      target.color.r = red;
      target.color.g = green;
      target.color.b = blue;
      target.color.a = predicted ? 0.4F : 0.8F;
      array.markers.push_back(target);

      if (track.track_id != visual_track_id_) {
        trail_.clear();
        visual_track_id_ = track.track_id;
      }
      if (!predicted) {
        trail_.push_back(track.position);
        if (trail_.size() > 100) {
          trail_.pop_front();
        }
      }
      auto history = make_marker(2, Marker::LINE_STRIP);
      history.scale.x = 0.025;
      history.color.r = red;
      history.color.g = green;
      history.color.b = blue;
      history.color.a = 0.85F;
      history.points.assign(trail_.begin(), trail_.end());
      array.markers.push_back(history);

      const double speed = std::hypot(track.velocity.x, track.velocity.y);
      if (speed > 0.03) {
        auto velocity = make_marker(1, Marker::ARROW);
        velocity.scale.x = 0.035;
        velocity.scale.y = 0.075;
        velocity.scale.z = 0.10;
        velocity.color = target.color;
        velocity.points.push_back(track.position);
        auto end = track.position;
        end.x += track.velocity.x * 0.5;
        end.y += track.velocity.y * 0.5;
        velocity.points.push_back(end);
        array.markers.push_back(velocity);
      } else {
        delete_marker(1, Marker::ARROW);
      }
      auto label = make_marker(3, Marker::TEXT_VIEW_FACING);
      label.pose.position = track.position;
      label.pose.position.z += 0.35;
      label.scale.z = 0.22;
      label.color.r = red;
      label.color.g = green;
      label.color.b = blue;
      label.color.a = 1.0F;
      label.text = "#" + std::to_string(track.track_id) + " " +
        (direct ? "DIRECT" : (predicted ? "PREDICTED" : "SUPPORT / Z inferred")) +
        " points=" + std::to_string(track.support_points);
      array.markers.push_back(label);
    }
    auto status = make_marker(4, Marker::TEXT_VIEW_FACING);
    status.pose.position.x = grid_config_.min_x + 1.4;
    status.pose.position.y = grid_config_.max_y - 0.5;
    status.pose.position.z = 2.0;
    status.scale.z = 0.25;
    status.color.r = track.valid ? 0.1F : 1.0F;
    status.color.g = track.valid ? 1.0F : 0.2F;
    status.color.b = 0.2F;
    status.color.a = 1.0F;
    status.text = (track.valid ? "TRACKING" : "NO TARGET: " + last_status_reason_) +
      " | processed=" + std::to_string(processed_) +
      " tf_drops=" + std::to_string(tf_drops_);
    array.markers.push_back(status);
    marker_pub_->publish(array);
  }

  static double percentile95(const std::deque<double> & samples)
  {
    if (samples.empty()) {
      return 0.0;
    }
    std::vector<double> sorted(samples.begin(), samples.end());
    const size_t index = static_cast<size_t>(0.95 * (sorted.size() - 1));
    std::nth_element(sorted.begin(), sorted.begin() + index, sorted.end());
    return sorted[index];
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "moving_bucket_detector";
    status.hardware_id = "CUDA";
    status.level = processing_errors_ > 0 ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = has_track_ && hits_ >= confirm_hits_ ? "tracking" : "no confirmed target";
    auto value = [&status](const std::string & key, auto number) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = std::to_string(number);
      status.values.push_back(item);
    };
    value("received", received_);
    value("processed", processed_);
    value("detections", detections_);
    value("support_candidate_frames", support_candidate_frames_);
    value("support_association_frames", support_association_frames_);
    value("tf_drops", tf_drops_);
    value("queue_drops", queue_drops_);
    value("processing_errors", processing_errors_);
    value("last_processing_ms", last_processing_ms_);
    value("processing_p95_ms", percentile95(processing_samples_));
    value("processing_max_ms", processing_samples_.empty() ? 0.0 :
      *std::max_element(processing_samples_.begin(), processing_samples_.end()));
    value("tf_wait_p95_ms", percentile95(tf_wait_samples_));
    array.status.push_back(status);
    diagnostic_pub_->publish(array);
  }

  lps::GridConfig grid_config_;
  float support_min_z_{0.4F}, support_max_z_{1.1F}, support_self_radius_{1.0F};
  lps::SupportTracker support_tracker_;
  std::unique_ptr<lps::CudaGrid> gpu_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<CloudMsg>::SharedPtr sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_tf_sub_;
  std::vector<std::pair<std::string, Eigen::Vector3d>> static_exclusions_;
  rclcpp::Publisher<TrackMsg>::SharedPtr target_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::deque<QueuedCloud> queue_;
  std::deque<geometry_msgs::msg::Point> trail_;
  uint32_t visual_track_id_{0};
  std::string last_status_reason_;
  std::string target_frame_;
  double support_target_z_{1.2};
  double radius_{0.1365}, max_residual_{0.055}, min_arc_{1.2}, gate_{0.65};
  double prediction_s_{0.3}, reset_s_{1.0}, tf_wait_s_{0.2};
  double last_input_stamp_{0.0}, last_observation_{0.0}, state_stamp_{0.0}, z_{0.0};
  double last_residual_{0.0}, last_processing_ms_{0.0}, last_tf_wait_ms_{0.0};
  std::deque<double> processing_samples_, tf_wait_samples_;
  unsigned int min_points_{18}, last_points_{0}, track_id_{0};
  int min_cells_{5}, confirm_hits_{3}, hits_{0};
  size_t queue_size_{5}, max_bytes_{8 * 1024 * 1024};
  bool publish_tf_{false}, has_track_{false};
  builtin_interfaces::msg::Time last_observation_stamp_;
  Eigen::Vector4d state_{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d covariance_{Eigen::Matrix4d::Identity()};
  uint64_t received_{0}, processed_{0}, detections_{0}, tf_drops_{0};
  uint64_t queue_drops_{0}, processing_errors_{0};
  uint64_t support_candidate_frames_{0}, support_association_frames_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MovingBucketDetector>());
  } catch (const std::exception & e) {
    fprintf(stderr, "moving_bucket_detector: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
