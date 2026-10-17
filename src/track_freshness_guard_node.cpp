// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <stdexcept>

#include "lidar_perception_system/msg/moving_bucket_track.hpp"

namespace lidar_perception_system {
/** @brief Independent CPU guard; only this node publishes the final Track and target TF. */
class TrackFreshnessGuard : public rclcpp::Node {
public:
  using Track = msg::MovingBucketTrack;
  using Clock = std::chrono::steady_clock;
  /** @brief Configure deadlines from the detector prediction horizon or explicit parameters. */
  TrackFreshnessGuard() : Node("track_freshness_guard") {
    nlohmann::json config = nlohmann::json::object();
    const auto path = declare_parameter<std::string>("config_file", "");
    if (!path.empty()) {
      std::ifstream stream(path);
      if (!stream) {
        throw std::invalid_argument("Cannot open detector configuration");
      }
      stream >> config;
    }
    const auto tracking = config.value("tracking", nlohmann::json::object());
    const double prediction_s = tracking.value("max_prediction_s", 0.3);
    input_timeout_s_ = declare_parameter<double>("input_timeout_s",
      tracking.value("input_timeout_s", prediction_s));
    track_timeout_s_ = declare_parameter<double>("track_timeout_s",
      tracking.value("track_timeout_s", prediction_s));
    observation_timeout_s_ = declare_parameter<double>("observation_timeout_s", prediction_s);
    frame_ = declare_parameter<std::string>("target_frame", config.value("target_frame", "map"));
    publish_tf_ = declare_parameter<bool>("publish_tf", config.value("publish_tf", false));
    for (double limit : {input_timeout_s_, track_timeout_s_, observation_timeout_s_}) {
      if (!std::isfinite(limit) || limit <= 0) {
        throw std::invalid_argument("Freshness deadlines must be finite and positive");
      }
    }
    const auto input_topic = declare_parameter<std::string>("input_topic",
      config.value("input_topic", "/livox/lidar"));
    const auto raw_topic = declare_parameter<std::string>("raw_track_topic", "/moving_bucket_detector/target_raw");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/moving_bucket_detector/target");
    if (raw_topic == output_topic || frame_.empty()) {
      throw std::invalid_argument("Guard input/output must differ and frame must be nonempty");
    }
    publisher_ = create_publisher<Track>(output_topic, rclcpp::QoS(1).durability_volatile());
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_topic,
      rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        const auto stamp_ns = checked_stamp(cloud->header.stamp);
        const double age_s = static_cast<double>(now().nanoseconds() - stamp_ns) * 1e-9;
        if (stamp_ns <= 0 || age_s < 0 || age_s >= input_timeout_s_) {
          return;
        }
        if (stamp_ns <= cloud_stamp_ns_) {
          return;  // Duplicate sensor messages cannot renew an input lease.
        }
        cloud_stamp_ns_ = stamp_ns;
        cloud_deadline_ = deadline(input_timeout_s_ - age_s);
        have_cloud_ = true;
      });
    raw_subscription_ = create_subscription<Track>(raw_topic, rclcpp::QoS(1).durability_volatile(),
      [this](Track::ConstSharedPtr track) { accept_track(*track); });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { evaluate(); });
  }
private:
  /** @brief Return a valid ROS timestamp in nanoseconds or zero for malformed input. */
  int64_t checked_stamp(const builtin_interfaces::msg::Time & stamp) const {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
      return 0;
    }
    return rclcpp::Time(stamp).nanoseconds();
  }
  /** @brief Convert remaining validity to a monotonic deadline. */
  Clock::time_point deadline(double seconds) const {
    return Clock::now() + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(seconds));
  }
  /** @brief Admit one current, finite raw track without extending duplicate observations. */
  void accept_track(const Track & track) {
    if (!track.valid) {
      have_track_ = false;
      return;
    }
    const auto stamp_ns = checked_stamp(track.header.stamp);
    const auto observation_ns = checked_stamp(track.last_observation_stamp);
    const auto current_ns = now().nanoseconds();
    const double age_s = static_cast<double>(current_ns - stamp_ns) * 1e-9;
    const double observation_age_s = static_cast<double>(current_ns - observation_ns) * 1e-9;
    if (track.header.frame_id != frame_ || stamp_ns <= 0 || observation_ns <= 0 ||
      observation_ns > stamp_ns || age_s < 0 || age_s >= track_timeout_s_ ||
      observation_age_s < 0 || observation_age_s >= observation_timeout_s_ ||
      track.observation_mode == Track::NONE || track.observation_mode > Track::PREDICTED ||
      !std::isfinite(track.confidence) || track.confidence < 0 || track.confidence > 1 || track.fit_residual < 0)
    {
      have_track_ = false;
      return;
    }
    for (const double value : {track.position.x, track.position.y, track.position.z,
        track.velocity.x, track.velocity.y, track.velocity.z, static_cast<double>(track.fit_residual)})
    {
      if (!std::isfinite(value)) {
        have_track_ = false;
        return;
      }
    }
    for (const auto value : track.state_covariance) {
      if (!std::isfinite(value)) {
        have_track_ = false;
        return;
      }
    }
    for (std::size_t axis = 0; axis < 6; ++axis) {
      if (track.state_covariance[axis * 6 + axis] < 0) {
        have_track_ = false;
        return;
      }
    }
    if (stamp_ns <= track_stamp_ns_) {
      return;  // No queue replay can revive or refresh a previously evaluated track.
    }
    track_ = track;
    track_stamp_ns_ = stamp_ns;
    track_deadline_ = deadline(std::min(track_timeout_s_ - age_s,
      observation_timeout_s_ - observation_age_s));
    have_track_ = true;
  }
  /** @brief Independently expire input/result/observation leases, even if CUDA stalls. */
  void evaluate() {
    const auto current_ns = now().nanoseconds();
    if (last_ros_now_ns_ > 0 && current_ns < last_ros_now_ns_) {
      have_cloud_ = false;
      have_track_ = false;
      cloud_stamp_ns_ = 0;
      track_stamp_ns_ = 0;
      last_tf_stamp_ns_ = 0;
    }
    last_ros_now_ns_ = current_ns;
    const auto steady_now = Clock::now();
    const auto observation_ns = checked_stamp(track_.last_observation_stamp);
    const double cloud_age_s = static_cast<double>(current_ns - cloud_stamp_ns_) * 1e-9;
    const double track_age_s = static_cast<double>(current_ns - track_stamp_ns_) * 1e-9;
    const double observation_age_s = static_cast<double>(current_ns - observation_ns) * 1e-9;
    const bool fresh = have_cloud_ && have_track_ && steady_now < cloud_deadline_ &&
      steady_now < track_deadline_ && cloud_age_s >= 0 && cloud_age_s < input_timeout_s_ &&
      track_age_s >= 0 && track_age_s < track_timeout_s_ &&
      observation_age_s >= 0 && observation_age_s < observation_timeout_s_;
    if (!fresh) {
      have_track_ = false;
      Track invalid;
      invalid.header.stamp = now();
      invalid.header.frame_id = frame_;
      invalid.track_id = track_.track_id;
      invalid.last_observation_stamp = track_.last_observation_stamp;
      invalid.valid = false;
      invalid.observation_mode = Track::NONE;
      publisher_->publish(invalid);
      return;
    }
    // Preserve the original observation/measurement timestamps on repeated reports.
    publisher_->publish(track_);
    if (publish_tf_ && track_.observation_mode == Track::DIRECT && track_stamp_ns_ > last_tf_stamp_ns_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = track_.header;
      transform.child_frame_id = "moving_bucket";
      transform.transform.translation.x = track_.position.x;
      transform.transform.translation.y = track_.position.y;
      transform.transform.translation.z = track_.position.z;
      transform.transform.rotation.w = 1;
      broadcaster_->sendTransform(transform);
      last_tf_stamp_ns_ = track_stamp_ns_;
    }
  }
  std::string frame_;
  double input_timeout_s_{}, track_timeout_s_{}, observation_timeout_s_{};
  bool publish_tf_{}, have_cloud_{}, have_track_{};
  int64_t cloud_stamp_ns_{}, track_stamp_ns_{}, last_ros_now_ns_{}, last_tf_stamp_ns_{};
  Clock::time_point cloud_deadline_{}, track_deadline_{};
  Track track_{};
  rclcpp::Publisher<Track>::SharedPtr publisher_;
  rclcpp::Subscription<Track>::SharedPtr raw_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace lidar_perception_system

/** @brief Start only the CPU guard; it does not allocate a CUDA context. */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<lidar_perception_system::TrackFreshnessGuard>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("track_freshness_guard"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
