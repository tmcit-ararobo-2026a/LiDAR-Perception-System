// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lidar_perception_system/msg/bucket_bottom.hpp"
#include "lidar_perception_system/msg/moving_bucket_track.hpp"
#include "lidar_perception_system/opening_tracker.hpp"

namespace op = lidar_perception_system::opening;
using Track = lidar_perception_system::msg::MovingBucketTrack;
using Cloud = sensor_msgs::msg::PointCloud2;
using BottomTrack = lidar_perception_system::msg::BucketBottom;
using Marker = visualization_msgs::msg::Marker;

namespace {
op::Config read_config(const nlohmann::json& j) {
  if (j.at("schema_version").get<int>() != 1 || !j.contains("opponent_roi") ||
      !j.at("opponent_roi").is_object()) {
    throw std::invalid_argument("Set opponent_roi min_x/max_x/min_y/max_y in the map frame");
  }
  op::Config c;
  const auto& roi = j.at("opponent_roi");
  c.min_x = roi.at("min_x").get<double>();
  c.max_x = roi.at("max_x").get<double>();
  c.min_y = roi.at("min_y").get<double>();
  c.max_y = roi.at("max_y").get<double>();
  const auto field = j.value("field_bounds", nlohmann::json::object());
  c.field_min_x = field.value("min_x", c.field_min_x);
  c.field_max_x = field.value("max_x", c.field_max_x);
  c.field_min_y = field.value("min_y", c.field_min_y);
  c.field_max_y = field.value("max_y", c.field_max_y);
  const auto filter = j.value("filter", nlohmann::json::object());
  const auto robot = j.value("robot", nlohmann::json::object());
  const auto accumulation = j.value("accumulation", nlohmann::json::object());
  const auto registration = j.value("registration", nlohmann::json::object());
  const auto opening = j.value("opening", nlohmann::json::object());
  const auto tracking = j.value("tracking", nlohmann::json::object());
#define READ(group, field) c.field = group.value(#field, c.field)
  READ(filter, min_z);
  READ(filter, max_z);
  READ(filter, self_radius);
  READ(filter, max_frame_points);
  READ(robot, body_min_z);
  READ(robot, body_max_z);
  READ(robot, cluster_resolution);
  READ(robot, min_span);
  READ(robot, max_span);
  READ(robot, max_body_side);
  READ(robot, body_size_tolerance);
  READ(robot, crop_margin);
  READ(robot, min_body_points);
  READ(accumulation, body_voxel);
  READ(accumulation, opening_voxel);
  READ(accumulation, window_s);
  READ(accumulation, max_body_points);
  READ(accumulation, max_model_points);
  READ(registration, correspondence_m);
  READ(registration, max_registration_rmse);
  READ(registration, min_overlap);
  READ(registration, min_support_spread_ratio);
  READ(registration, icp_iterations);
  READ(opening, min_opening_z);
  READ(opening, max_opening_z);
  READ(opening, min_radius);
  READ(opening, max_radius);
  READ(opening, nominal_radius);
  READ(opening, slice_m);
  READ(opening, max_residual);
  READ(opening, max_plane_rmse);
  READ(opening, min_arc);
  READ(opening, height_change_m);
  READ(opening, min_lip_width);
  READ(opening, min_rim_points);
  READ(opening, min_current_points);
  READ(opening, min_side_points);
  READ(opening, ransac_iterations);
  READ(tracking, confirm_hits);
  READ(tracking, reset_s);
  READ(tracking, association_m);
  READ(tracking, body_recovery_s);
#undef READ
  const auto bottom = j.value("bottom", nlohmann::json::object());
#define READ_BOTTOM(field) c.bottom.field = bottom.value(#field, c.bottom.field)
  READ_BOTTOM(bucket_height);
  READ_BOTTOM(height_tolerance);
  READ_BOTTOM(min_radius);
  READ_BOTTOM(max_radius);
  READ_BOTTOM(plane_distance);
  READ_BOTTOM(max_plane_rmse);
  READ_BOTTOM(max_tilt_rad);
  READ_BOTTOM(connection_m);
  READ_BOTTOM(side_tolerance);
  READ_BOTTOM(min_side_span);
  READ_BOTTOM(max_body_inference_s);
  READ_BOTTOM(min_plane_points);
  READ_BOTTOM(min_current_points);
  READ_BOTTOM(min_side_points);
#undef READ_BOTTOM
  c.prediction_s = tracking.value("max_prediction_s", c.prediction_s);
  c.validate();
  return c;
}
geometry_msgs::msg::Point point(const op::Point& v) {
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}
Cloud make_cloud(const op::Points& points, const std_msgs::msg::Header& header) {
  Cloud out;
  out.header = header;
  out.height = 1;
  out.width = static_cast<uint32_t>(points.size());
  out.is_dense = true;
  out.point_step = 12;
  out.row_step = 12 * out.width;
  for (size_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = std::string(1, "xyz"[i]);
    f.offset = 4 * i;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    out.fields.push_back(f);
  }
  out.data.resize(out.row_step);
  for (size_t i = 0; i < points.size(); ++i) {
    const float p[3] = {static_cast<float>(points[i].x()), static_cast<float>(points[i].y()),
                        static_cast<float>(points[i].z())};
    std::memcpy(out.data.data() + 12 * i, p, 12);
  }
  return out;
}
}  // namespace

class BucketOpeningTracker : public rclcpp::Node {
 public:
  BucketOpeningTracker() : Node("bucket_opening_tracker") {
    const auto path = declare_parameter<std::string>("config_file", "");
    std::ifstream stream(path);
    if (!stream) {
      throw std::invalid_argument("Cannot open opening tracker config_file");
    }
    const auto j = nlohmann::json::parse(stream);
    cfg_ = read_config(j);
    radius_ = cfg_.nominal_radius;
    frame_ = j.at("target_frame").get<std::string>();
    input_topic_ = j.at("input_topic").get<std::string>();
    if (frame_.empty() || input_topic_.empty()) {
      throw std::invalid_argument("Input topic and target frame must be nonempty");
    }
    tf_wait_s_ = j.value("tf_wait_s", 0.2);
    queue_limit_ = j.value("queue_size", 4);
    max_bytes_ = j.value("max_input_bytes", 8388608);
    if (!std::isfinite(tf_wait_s_) || tf_wait_s_ < 0 || tf_wait_s_ > 2 || queue_limit_ < 1 ||
        queue_limit_ > 100 || max_bytes_ < 12 || max_bytes_ > 67108864) {
      throw std::invalid_argument("Invalid input queue/capacity");
    }
    tracker_ = std::make_unique<op::Tracker>(cfg_);
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    raw_ = create_publisher<Track>("~/target_raw", 1);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 1);
    robot_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/robot_markers", 1);
    selected_ = create_publisher<Cloud>("~/selected_cloud", rclcpp::SensorDataQoS());
    accumulated_ = create_publisher<Cloud>("~/accumulated_cloud", rclcpp::SensorDataQoS());
    bottom_ = create_publisher<BottomTrack>("~/bottom", 10);
    bottom_track_ = create_publisher<BottomTrack>("~/bottom_track", 10);
    bottom_markers_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("~/bottom_markers", 10);
    bottom_cloud_ = create_publisher<Cloud>("~/bottom_cloud", rclcpp::SensorDataQoS());
    clouds_ =
        create_subscription<Cloud>(input_topic_, rclcpp::SensorDataQoS(),
                                   [this](Cloud::ConstSharedPtr c) { receive(std::move(c)); });
    final_ = create_subscription<Track>("~/target", 1,
                                        [this](Track::ConstSharedPtr t) { show_final(*t); });
    fixed_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(1).transient_local().reliable(),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
          for (const auto& t : m->transforms) {
            if (t.header.frame_id != frame_) {
              continue;
            }
            const auto& name = t.child_frame_id;
            double radius = 0;
            if (name.rfind("desk_", 0) == 0) {
              radius = 0.65;
            } else if (name.rfind("bucket_", 0) == 0) {
              radius = 0.45;
            } else if (name.rfind("flag_", 0) == 0) {
              radius = 0.50;
            }
            if (radius == 0) {
              continue;
            }
            const auto& p = t.transform.translation;
            if (std::isfinite(p.x) && std::isfinite(p.y)) {
              exclusions_[name] = {p.x, p.y, radius};
            }
          }
          std::vector<Eigen::Vector3d> masks;
          for (const auto& item : exclusions_) {
            masks.push_back(item.second);
          }
          tracker_->set_exclusions(masks);
        });
    processing_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() { process(); });
    diagnostic_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { diagnose(); });
    bottom_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      if (bottom_output_.valid &&
          (std::chrono::steady_clock::now() - bottom_received_ >
               std::chrono::duration<double>(cfg_.prediction_s) ||
           (now() - rclcpp::Time(bottom_output_.header.stamp)).seconds() > cfg_.prediction_s ||
           now() < rclcpp::Time(bottom_output_.header.stamp))) {
        auto header = bottom_output_.header;
        header.stamp = now();
        publish_bottom(op::Result{}, header);
      }
      if (body_debug_valid_ &&
          (std::chrono::steady_clock::now() - body_debug_received_ >
               std::chrono::duration<double>(cfg_.prediction_s) ||
           (now() - rclcpp::Time(body_debug_header_.stamp)).seconds() > cfg_.prediction_s ||
           now() < rclcpp::Time(body_debug_header_.stamp))) {
        auto header = body_debug_header_;
        header.stamp = now();
        debug(op::Result{}, header);
      }
    });
    RCLCPP_INFO(get_logger(),
                "CPU opening tracker ready; explicit map ROI, %.2fs robot-frame window",
                cfg_.window_s);
  }

 private:
  struct Queued {
    Cloud::ConstSharedPtr cloud;
    std::chrono::steady_clock::time_point received;
  };
  void receive(Cloud::ConstSharedPtr cloud) {
    ++received_;
    if (cloud->data.size() > static_cast<size_t>(max_bytes_)) {
      ++errors_;
      invalid(cloud->header, "PointCloud2 exceeds input capacity");
      return;
    }
    if (cloud->header.stamp.sec < 0 || cloud->header.stamp.nanosec >= 1000000000U) {
      ++errors_;
      auto header = cloud->header;
      header.stamp = now();
      publish_bottom(op::Result{}, header);
      debug(op::Result{}, header);
      return;
    }
    const int64_t stamp = rclcpp::Time(cloud->header.stamp).nanoseconds();
    if (stamp <= 0) {
      ++errors_;
      auto header = cloud->header;
      header.stamp = now();
      publish_bottom(op::Result{}, header);
      debug(op::Result{}, header);
      return;
    }
    if (last_input_ns_ > 0 && stamp < last_input_ns_) {
      queue_.clear();
      tracker_->reset();
      publish_bottom(op::Result{}, cloud->header);
      debug(op::Result{}, cloud->header);
      last_observation_ = builtin_interfaces::msg::Time();
      ++rewinds_;
    }
    if (stamp == last_input_ns_) {
      ++duplicates_;
      return;
    }
    last_input_ns_ = stamp;
    if (queue_.size() >= static_cast<size_t>(queue_limit_)) {
      queue_.pop_front();
      ++queue_drops_;
    }
    queue_.push_back({std::move(cloud), std::chrono::steady_clock::now()});
  }
  void invalid(const std_msgs::msg::Header& header, const std::string& why) {
    Track t;
    t.header = header;
    t.header.frame_id = frame_;
    t.valid = false;
    t.last_observation_stamp = last_observation_;
    raw_->publish(t);
    publish_bottom(op::Result{}, header);
    debug(op::Result{}, header);
    reason_ = why;
    last_result_.valid = false;
  }
  op::Points decode(const Cloud& c, const geometry_msgs::msg::TransformStamped& transform) const {
    // A structured zero-width cloud is a valid observation with no returns.
    // Process it as a miss so the learned opening expires without a decode
    // error.
    if (c.height == 0 || c.is_bigendian || c.point_step < 12 ||
        c.data.size() > static_cast<size_t>(max_bytes_) ||
        static_cast<uint64_t>(c.width) * c.point_step > c.row_step ||
        static_cast<uint64_t>(c.row_step) * c.height > c.data.size()) {
      throw std::invalid_argument("Unsupported PointCloud2 layout/capacity");
    }
    std::array<int, 3> offsets{-1, -1, -1};
    for (const auto& f : c.fields) {
      for (size_t i = 0; i < 3; ++i) {
        if (f.name == std::string(1, "xyz"[i]) &&
            f.datatype == sensor_msgs::msg::PointField::FLOAT32 && f.count == 1 &&
            f.offset <= c.point_step - 4) {
          offsets[i] = static_cast<int>(f.offset);
        }
      }
    }
    if (std::any_of(offsets.begin(), offsets.end(), [](int n) { return n < 0; })) {
      throw std::invalid_argument("FLOAT32 x/y/z fields required");
    }
    const auto& q = transform.transform.rotation;
    Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    const auto& t = transform.transform.translation;
    op::Point translation(t.x, t.y, t.z);
    if (!rotation.coeffs().allFinite() || rotation.norm() < 0.5 || !translation.allFinite()) {
      throw std::invalid_argument("Invalid sensor transform");
    }
    const auto matrix = rotation.normalized().toRotationMatrix();
    const size_t count = static_cast<size_t>(c.width) * c.height;
    const size_t stride =
        std::max(size_t{1}, (count + cfg_.max_frame_points - 1) / cfg_.max_frame_points);
    op::Points points;
    points.reserve(std::min(count, static_cast<size_t>(cfg_.max_frame_points)));
    for (size_t i = 0; i < count; i += stride) {
      const auto* src = c.data.data() + (i / c.width) * c.row_step + (i % c.width) * c.point_step;
      float xyz[3];
      for (size_t axis = 0; axis < 3; ++axis) {
        std::memcpy(&xyz[axis], src + offsets[axis], 4);
      }
      op::Point p(xyz[0], xyz[1], xyz[2]);
      if (p.allFinite()) {
        points.push_back(matrix * p + translation);
      }
    }
    return points;
  }
  void process() {
    if (queue_.empty()) {
      return;
    }
    const auto item = queue_.front();
    const double age =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - item.received).count();
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = buffer_->lookupTransform(frame_, item.cloud->header.frame_id,
                                           rclcpp::Time(item.cloud->header.stamp),
                                           rclcpp::Duration::from_seconds(0));
    } catch (const tf2::TransformException&) {
      if (age >= tf_wait_s_) {
        queue_.pop_front();
        ++tf_drops_;
        invalid(item.cloud->header, "TF unavailable");
      }
      return;
    }
    queue_.pop_front();
    const auto started = std::chrono::steady_clock::now();
    try {
      const auto points = decode(*item.cloud, transform);
      const auto& t = transform.transform.translation;
      const double stamp = rclcpp::Time(item.cloud->header.stamp).seconds();
      last_result_ = tracker_->process(points, stamp, {t.x, t.y});
      body_frames_ += last_result_.body_valid;
      radius_frames_ += last_result_.fit.radius_hypotheses > 0;
      geometry_frames_ += last_result_.fit.geometry_hypotheses > 0;
      edge_frames_ += last_result_.fit.edge_hypotheses > 0;
      fresh_rim_frames_ += last_result_.opening.has_value() && !last_result_.predicted;
      if (last_result_.observation_stamp == stamp && last_result_.opening &&
          !last_result_.predicted) {
        last_observation_ = item.cloud->header.stamp;
      }
      if (last_result_.observation_stamp == 0) {
        last_observation_ = builtin_interfaces::msg::Time();
      }
      publish(last_result_, item.cloud->header);
      publish_bottom(last_result_, item.cloud->header);
      ++processed_;
      if (last_result_.valid && !last_result_.predicted) {
        ++observed_;
      }
      if (last_result_.predicted) {
        ++predicted_;
      }
      debug(last_result_, item.cloud->header);
    } catch (const std::exception& e) {
      ++errors_;
      invalid(item.cloud->header, e.what());
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Opening frame rejected: %s",
                           e.what());
    }
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count();
    timings_.push_back(ms);
    if (timings_.size() > 1000) {
      timings_.pop_front();
    }
  }
  void publish(const op::Result& r, std_msgs::msg::Header header) {
    Track t;
    header.frame_id = frame_;
    t.header = header;
    t.last_observation_stamp = last_observation_;
    t.track_id = r.track_id;
    t.valid = r.valid;
    t.observation_mode = !r.valid ? Track::NONE : (r.predicted ? Track::PREDICTED : Track::DIRECT);
    if (r.valid) {
      t.position = point(r.position);
      t.velocity.x = r.velocity.x();
      t.velocity.y = r.velocity.y();
      t.velocity.z = 0;
      t.state_covariance[0] = r.position_variance;
      t.state_covariance[7] = r.position_variance;
      t.state_covariance[14] =
          std::max(0.0001, r.opening ? std::pow(r.opening->plane_rmse, 2) : 0.01);
      t.state_covariance[21] = 1;
      t.state_covariance[28] = 1;
      t.state_covariance[35] = 1;
      t.confidence = r.predicted ? 0 : static_cast<float>(r.confidence);
      if (r.opening) {
        t.fit_residual = static_cast<float>(r.opening->residual);
        t.support_points = r.predicted ? 0 : r.opening->current_points;
        radius_ = r.opening->radius;
      }
    }
    reason_ = r.reason;
    raw_->publish(t);
  }
  Marker base_marker(const std_msgs::msg::Header& header, int id, int type,
                     const std::string& ns) const {
    Marker m;
    m.header = header;
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1;
    m.color.a = 1;
    m.lifetime = rclcpp::Duration::from_seconds(0.35);
    return m;
  }
  BottomTrack bottom_message(const op::Result& r, std_msgs::msg::Header header,
                             const std::optional<op::Bottom>& geometry, op::BottomMode mode,
                             const builtin_interfaces::msg::Time& previous_stamp) const {
    header.frame_id = frame_;
    BottomTrack t;
    t.header = header;
    t.track_id = r.track_id;
    t.last_observation_stamp = previous_stamp;
    const double age = (now() - rclcpp::Time(header.stamp)).seconds();
    t.valid = r.body_valid && geometry && mode != op::BottomMode::None && age >= -.05 &&
              age <= cfg_.prediction_s;
    if (!t.valid) return t;
    const auto& b = *geometry;
    t.observation_mode = static_cast<uint8_t>(mode);
    if (mode != op::BottomMode::BodyInferred) t.last_observation_stamp = header.stamp;
    t.center = point(r.pose.world(b.center));
    const op::Point normal =
        (r.pose.world(b.normal) - r.pose.world(op::Point::Zero())).normalized();
    t.normal.x = normal.x();
    t.normal.y = normal.y();
    t.normal.z = normal.z();
    t.radius = b.radius;
    t.plane_rmse = b.plane_rmse;
    t.boundary_residual = b.boundary_residual;
    t.plane_points = b.plane_points;
    t.current_points = b.current_points;
    t.side_points = b.side_points;
    t.confidence = mode == op::BottomMode::BodyInferred
                       ? 0
                       : std::clamp(1. - b.plane_rmse / cfg_.bottom.max_plane_rmse, 0., 1.);
    return t;
  }
  void publish_bottom(const op::Result& r, std_msgs::msg::Header header) {
    header.frame_id = frame_;
    auto measured = bottom_message(r, header, r.bottom,
                                   r.bottom_valid ? op::BottomMode::Direct : op::BottomMode::None,
                                   bottom_direct_stamp_);
    if (measured.valid) {
      bottom_direct_stamp_ = measured.last_observation_stamp;
      ++bottom_observed_;
    }
    bottom_->publish(measured);
    // Retain the exact plane timestamp even when processing latency prevents
    // publishing that frame as fresh. Later body inference must not inherit a
    // timestamp from another track or from an earlier published observation.
    if (r.bottom_tracking_observation_stamp > 0 && r.bottom_tracking_observation_stamp == r.stamp) {
      bottom_tracking_stamp_ = header.stamp;
    }
    auto t =
        bottom_message(r, header, r.tracked_bottom, r.bottom_tracking_mode, bottom_tracking_stamp_);
    visualization_msgs::msg::MarkerArray markers;
    auto ring = base_marker(header, 0, Marker::LINE_STRIP, "bucket_bottom");
    ring.action = Marker::DELETE;
    auto center_marker = base_marker(header, 1, Marker::SPHERE, "bucket_bottom");
    center_marker.action = Marker::DELETE;
    std::array<Marker, 3> axes;
    for (int axis = 0; axis < 3; ++axis) {
      axes[axis] = base_marker(header, axis + 2, Marker::ARROW, "bucket_bottom_axes");
      axes[axis].action = Marker::DELETE;
    }
    op::Points plane;
    if (t.valid) {
      const auto& b = *r.tracked_bottom;
      const auto center = r.pose.world(b.center);
      const op::Point normal =
          (r.pose.world(b.normal) - r.pose.world(op::Point::Zero())).normalized();
      // A circular bottom has no observable yaw. Use the robot's local X direction
      // projected onto the measured plane to define a consistent tangent axis.
      op::Point x_axis(std::cos(r.pose.yaw), std::sin(r.pose.yaw), 0);
      x_axis = (x_axis - normal * normal.dot(x_axis)).normalized();
      Eigen::Matrix3d basis;
      basis.col(0) = x_axis;
      basis.col(1) = normal.cross(x_axis);
      basis.col(2) = normal;
      const Eigen::Quaterniond orientation(basis);
      ring.action = Marker::ADD;
      ring.scale.x = .012;
      const bool inferred = t.observation_mode == BottomTrack::BODY_INFERRED;
      ring.color.r = inferred ? 0 : 1;
      ring.color.g = t.observation_mode == BottomTrack::DIRECT ? .55 : 1;
      ring.color.b = inferred ? 1 : 0;
      center_marker.action = Marker::ADD;
      center_marker.pose.position = t.center;
      center_marker.scale.x = center_marker.scale.y = center_marker.scale.z = .045;
      center_marker.color = ring.color;
      for (int axis = 0; axis < 3; ++axis) {
        auto& marker = axes[axis];
        marker.action = Marker::ADD;
        marker.scale.x = .012;
        marker.scale.y = .025;
        marker.scale.z = .035;
        marker.color.r = axis == 0 ? 1 : 0;
        marker.color.g = axis == 1 ? 1 : 0;
        marker.color.b = axis == 2 ? 1 : 0;
        marker.points = {t.center, point(center + .18 * basis.col(axis))};
      }
      geometry_msgs::msg::TransformStamped tf;
      tf.header = header;
      tf.child_frame_id = "bucket_bottom";
      tf.transform.translation.x = center.x();
      tf.transform.translation.y = center.y();
      tf.transform.translation.z = center.z();
      tf.transform.rotation.x = orientation.x();
      tf.transform.rotation.y = orientation.y();
      tf.transform.rotation.z = orientation.z();
      tf.transform.rotation.w = orientation.w();
      broadcaster_->sendTransform(tf);
      const op::Point u = normal.unitOrthogonal(), v = normal.cross(u);
      for (int i = 0; i <= 64; ++i) {
        const double a = i * 2 * 3.141592653589793 / 64;
        ring.points.push_back(point(center + b.radius * (std::cos(a) * u + std::sin(a) * v)));
      }
      for (const auto& p : tracker_->selected_cloud()) {
        const auto local = r.pose.local(p);
        if (t.observation_mode != BottomTrack::BODY_INFERRED &&
            std::abs(b.normal.dot(local - b.center)) <= cfg_.bottom.plane_distance &&
            (local.head<2>() - b.center.head<2>()).norm() <= b.radius + cfg_.bottom.plane_distance)
          plane.push_back(p);
      }
      bottom_received_ = std::chrono::steady_clock::now();
      ++bottom_tracked_;
    }
    markers.markers.push_back(ring);
    markers.markers.push_back(center_marker);
    markers.markers.insert(markers.markers.end(), axes.begin(), axes.end());
    bottom_markers_->publish(markers);
    bottom_cloud_->publish(make_cloud(plane, header));
    bottom_output_ = t;
    bottom_track_->publish(t);
  }
  void show_final(const Track& track) {
    visualization_msgs::msg::MarkerArray a;
    for (int id = 0; id < 3; ++id) {
      auto m = base_marker(track.header, id, Marker::SPHERE, "opening");
      m.action = Marker::DELETE;
      a.markers.push_back(m);
    }
    if (track.valid) {
      const bool predicted = track.observation_mode == Track::PREDICTED;
      auto center = base_marker(track.header, 0, Marker::SPHERE, "opening");
      center.pose.position = track.position;
      center.scale.x = center.scale.y = center.scale.z = 0.05;
      center.color.g = 0.9;
      center.color.b = predicted ? 1 : 0;
      a.markers.push_back(center);
      auto rim = base_marker(track.header, 1, Marker::LINE_STRIP, "opening");
      rim.scale.x = 0.012;
      rim.color.g = 0.9;
      rim.color.b = predicted ? 1 : 0;
      for (int i = 0; i <= 64; ++i) {
        const double theta = i * 2 * 3.141592653589793 / 64;
        auto p = track.position;
        p.x += radius_ * std::cos(theta);
        p.y += radius_ * std::sin(theta);
        rim.points.push_back(p);
      }
      a.markers.push_back(rim);
      auto velocity = base_marker(track.header, 2, Marker::ARROW, "opening");
      velocity.scale.x = 0.02;
      velocity.scale.y = 0.04;
      velocity.scale.z = 0.06;
      velocity.color.g = 0.8;
      velocity.points.push_back(track.position);
      auto end = track.position;
      end.x += track.velocity.x * 0.5;
      end.y += track.velocity.y * 0.5;
      velocity.points.push_back(end);
      a.markers.push_back(velocity);
    }
    auto status = base_marker(track.header, 3, Marker::TEXT_VIEW_FACING, "opening");
    status.pose.position.z = 2.3;
    status.scale.z = 0.13;
    status.color.g = track.valid ? 1 : 0.2;
    status.color.r = track.valid ? 0.2 : 1;
    status.text = track.valid ? (track.observation_mode == Track::PREDICTED ? "PREDICTED OPENING"
                                                                            : "OBSERVED OPENING")
                              : "NO OPENING: " + reason_;
    a.markers.push_back(status);
    markers_->publish(a);
  }
  void debug(const op::Result& r, std_msgs::msg::Header header) {
    header.frame_id = frame_;
    const double age = (now() - rclcpp::Time(header.stamp)).seconds();
    const bool fresh = r.body_valid && age >= -.05 && age <= cfg_.prediction_s;
    body_debug_valid_ = fresh;
    if (fresh) {
      body_debug_header_ = header;
      body_debug_received_ = std::chrono::steady_clock::now();
    }
    if (selected_->get_subscription_count()) {
      selected_->publish(make_cloud(fresh ? tracker_->selected_cloud() : op::Points{}, header));
    }
    if (accumulated_->get_subscription_count()) {
      op::Points map;
      map.reserve(tracker_->accumulated_cloud().size());
      if (fresh) {
        for (const auto& p : tracker_->body_model()) {
          map.push_back(r.pose.world(p));
        }
        for (const auto& p : tracker_->accumulated_cloud()) {
          map.push_back(r.pose.world(p));
        }
      }
      accumulated_->publish(make_cloud(map, header));
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header = header;
    tf.child_frame_id = "lps_base_link";
    tf.transform.translation.x = r.pose.xy.x();
    tf.transform.translation.y = r.pose.xy.y();
    tf.transform.rotation.z = std::sin(r.pose.yaw / 2);
    tf.transform.rotation.w = std::cos(r.pose.yaw / 2);
    if (fresh) {
      broadcaster_->sendTransform(tf);
    }
    if (!robot_markers_->get_subscription_count()) {
      return;
    }
    visualization_msgs::msg::MarkerArray a;
    for (int axis = 0; axis < 3; ++axis) {
      auto m = base_marker(header, axis, Marker::ARROW, "robot_frame");
      if (!fresh) {
        m.action = Marker::DELETE;
        a.markers.push_back(m);
        continue;
      }
      m.scale.x = 0.02;
      m.scale.y = 0.04;
      m.scale.z = 0.06;
      if (axis == 0) {
        m.color.r = 1;
      } else if (axis == 1) {
        m.color.g = 1;
      } else {
        m.color.b = 1;
      }
      m.points.push_back(point(r.pose.world({0, 0, 0.1})));
      op::Point end(0, 0, 0.1);
      end[axis] += 0.35;
      m.points.push_back(point(r.pose.world(end)));
      a.markers.push_back(m);
    }
    robot_markers_->publish(a);
  }
  void diagnose() {
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "bucket_opening_tracker";
    s.hardware_id = "CPU robot-frame geometry";
    s.message = reason_;
    s.level = last_result_.valid ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                 : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    const auto add = [&s](const std::string& k, auto v) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = std::to_string(v);
      s.values.push_back(kv);
    };
    add("received", received_);
    add("processed", processed_);
    add("observed", observed_);
    add("predicted", predicted_);
    add("tf_drops", tf_drops_);
    add("queue_drops", queue_drops_);
    add("processing_errors", errors_);
    add("duplicates", duplicates_);
    add("rewinds", rewinds_);
    add("candidates", last_result_.candidates);
    add("model_points", last_result_.model_points);
    add("registration_rmse", last_result_.registration_rmse);
    add("registration_overlap", last_result_.overlap);
    add("height_resets", tracker_->height_resets());
    add("body_registered_frames", body_frames_);
    add("robot_valid", body_debug_valid_);
    add("robot_recovered", last_result_.body_recovered);
    add("recovery_attempts", last_result_.recovery_attempts);
    add("robot_has_motion_evidence", last_result_.motion_seeded);
    add("bottom_observed_frames", bottom_observed_);
    add("bottom_tracking_valid", bottom_output_.valid);
    add("bottom_tracking_mode", bottom_output_.observation_mode);
    add("bottom_tracked_frames", bottom_tracked_);
    add("bottom_plane_points", last_result_.bottom ? last_result_.bottom->plane_points : 0);
    add("bottom_current_points", last_result_.bottom ? last_result_.bottom->current_points : 0);
    add("bottom_side_points", last_result_.bottom ? last_result_.bottom->side_points : 0);
    add("rim_radius_frames", radius_frames_);
    add("rim_geometry_frames", geometry_frames_);
    add("rim_edge_frames", edge_frames_);
    add("rim_fresh_frames", fresh_rim_frames_);
    add("rim_points", last_result_.opening ? last_result_.opening->points : 0);
    add("rim_arc", last_result_.opening ? last_result_.opening->arc : 0);
    add("rim_residual", last_result_.opening ? last_result_.opening->residual : 0);
    add("rim_max_slice_points", last_result_.fit.max_slice_points);
    add("rim_max_inliers", last_result_.fit.max_inliers);
    add("rim_max_arc", last_result_.fit.max_arc);
    add("rim_radius_hypotheses", last_result_.fit.radius_hypotheses);
    add("rim_geometry_hypotheses", last_result_.fit.geometry_hypotheses);
    add("rim_edge_hypotheses", last_result_.fit.edge_hypotheses);
    add("rim_interior_rejections", last_result_.fit.interior_rejections);
    if (!timings_.empty()) {
      std::vector<double> sorted(timings_.begin(), timings_.end());
      std::sort(sorted.begin(), sorted.end());
      add("processing_p95_ms", sorted[static_cast<size_t>((sorted.size() - 1) * 0.95)]);
      add("processing_max_ms", sorted.back());
    }
    a.status.push_back(s);
    diagnostics_->publish(a);
  }
  op::Config cfg_;
  std::unique_ptr<op::Tracker> tracker_;
  op::Result last_result_;
  std::string frame_, input_topic_, reason_{"waiting for input"};
  double tf_wait_s_{}, radius_{0.1365};
  int queue_limit_{}, max_bytes_{};
  int64_t last_input_ns_{0};
  builtin_interfaces::msg::Time last_observation_;
  std::deque<Queued> queue_;
  std::deque<double> timings_;
  std::map<std::string, Eigen::Vector3d> exclusions_;
  uint64_t received_{0}, processed_{0}, observed_{0}, predicted_{0}, tf_drops_{0}, queue_drops_{0},
      errors_{0}, duplicates_{0}, rewinds_{0};
  uint64_t body_frames_{0}, radius_frames_{0}, geometry_frames_{0}, edge_frames_{0},
      fresh_rim_frames_{0};
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<Track>::SharedPtr raw_;
  rclcpp::Publisher<BottomTrack>::SharedPtr bottom_, bottom_track_;
  builtin_interfaces::msg::Time bottom_direct_stamp_, bottom_tracking_stamp_;
  BottomTrack bottom_output_;
  bool body_debug_valid_{false};
  std_msgs::msg::Header body_debug_header_;
  std::chrono::steady_clock::time_point body_debug_received_;
  std::chrono::steady_clock::time_point bottom_received_;
  uint64_t bottom_observed_{0}, bottom_tracked_{0};
  rclcpp::Publisher<Cloud>::SharedPtr selected_, accumulated_, bottom_cloud_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_, robot_markers_,
      bottom_markers_;
  rclcpp::Subscription<Cloud>::SharedPtr clouds_;
  rclcpp::Subscription<Track>::SharedPtr final_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr fixed_;
  rclcpp::TimerBase::SharedPtr processing_timer_, diagnostic_timer_, bottom_timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BucketOpeningTracker>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("bucket_opening_tracker"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
