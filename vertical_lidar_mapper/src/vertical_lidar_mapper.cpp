// Timestamp: 2026-02-23 23:10:00 +07+0700
// Most Recent Update: Added risk-hardening controls for motion gating, bounded global cloud,
// TF message filtering, and diagnostics publishing.
#include "vertical_lidar_mapper/vertical_lidar_mapper.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#if __has_include(<tf2/exceptions.hpp>)
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#else
#include <tf2/exceptions.h>
#include <tf2/time.h>
#endif
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace vertical_lidar_mapper
{

namespace
{
inline std::string to_string_with_precision(double value, int precision = 3)
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(precision);
  oss << value;
  return oss.str();
}

double normalize_angle(double angle_rad)
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

tf2::Transform to_tf2_transform(const geometry_msgs::msg::TransformStamped & t_msg)
{
  tf2::Quaternion q(
    t_msg.transform.rotation.x,
    t_msg.transform.rotation.y,
    t_msg.transform.rotation.z,
    t_msg.transform.rotation.w);
  if (q.length2() <= 1e-12) {
    q.setRPY(0.0, 0.0, 0.0);
  } else {
    q.normalize();
  }

  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(
      t_msg.transform.translation.x,
      t_msg.transform.translation.y,
      t_msg.transform.translation.z));
  tf.setRotation(q);
  return tf;
}

Eigen::Matrix4f to_eigen_matrix(const tf2::Transform & tf)
{
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  const tf2::Matrix3x3 basis = tf.getBasis();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      m(r, c) = static_cast<float>(basis[r][c]);
    }
  }
  m(0, 3) = static_cast<float>(tf.getOrigin().x());
  m(1, 3) = static_cast<float>(tf.getOrigin().y());
  m(2, 3) = static_cast<float>(tf.getOrigin().z());
  return m;
}
}  // namespace

VerticalLidarMapper::VerticalLidarMapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("vertical_lidar_mapper", options), global_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>())
{
  loadParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_timer_interface_ = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(tf_timer_interface_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS());
  map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    map_topic_, rclcpp::QoS(5).reliable());
  global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    global_map_topic_, rclcpp::QoS(5).reliable());
  status_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    status_topic_, rclcpp::QoS(10).reliable());
  save_pcd_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/save_pcd",
    std::bind(
      &VerticalLidarMapper::handleSavePcdRequest,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  motion_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    motion_odom_topic_,
    rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::SharedPtr msg) { motion_odom_ = *msg; });

  scan_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>();
  scan_filter_sub_->subscribe(this, scan_topic_, rclcpp::SensorDataQoS());

  scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *scan_filter_sub_,
    *tf_buffer_,
    target_frame_,
    static_cast<uint32_t>(std::max(1, tf_filter_queue_size_)),
    get_node_logging_interface(),
    get_node_clock_interface(),
    tf2::durationFromSec(tf_timeout_));

  scan_filter_->registerCallback(
    std::bind(&VerticalLidarMapper::scanCallback, this, std::placeholders::_1));

  const double global_period_sec = 1.0 / std::max(0.1, global_publish_hz_);
  global_publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(global_period_sec)),
    std::bind(&VerticalLidarMapper::onGlobalPublishTimer, this));

  status_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&VerticalLidarMapper::onStatusTimer, this));

  RCLCPP_INFO(
    this->get_logger(),
    "vertical_lidar_mapper started. scan_topic='%s' target_frame='%s' local_voxel=%.3f global_voxel=%.3f global_cap=%d",
    scan_topic_.c_str(),
    target_frame_.c_str(),
    voxel_leaf_,
    global_voxel_leaf_size_,
    max_global_points_);
  RCLCPP_INFO(
    this->get_logger(),
    "Map-rebase: %s (map_frame='%s' odom_frame='%s' trans_th=%.3fm yaw_th=%.3frad)",
    enable_map_rebase_ ? "enabled" : "disabled",
    map_rebase_map_frame_.c_str(),
    map_rebase_odom_frame_.c_str(),
    map_rebase_translation_threshold_,
    map_rebase_yaw_threshold_);
  RCLCPP_INFO(
    this->get_logger(),
    "Relative-pose gate: %s (map='%s' odom='%s' trans_err_th=%.3fm yaw_err_th=%.3frad min_motion=%.3fm)",
    enable_relative_pose_gate_ ? "enabled" : "disabled",
    relative_pose_map_frame_.c_str(),
    relative_pose_odom_frame_.c_str(),
    relative_pose_translation_error_threshold_,
    relative_pose_yaw_error_threshold_,
    relative_pose_min_motion_xy_);
  const std::string save_service_name = this->get_fully_qualified_name() + std::string("/save_pcd");
  RCLCPP_INFO(
    this->get_logger(),
    "PCD export service: '%s' dir='%s' binary=%s",
    save_service_name.c_str(),
    pcd_export_dir_.c_str(),
    pcd_export_binary_ ? "true" : "false");
}

void VerticalLidarMapper::loadParameters()
{
  scan_topic_ = this->declare_parameter<std::string>(
    "scan_topic",
    "/world/walls/model/x500_lidar_2d_tilted_0/model/lidar_vert/link/link/sensor/lidar_2d_v2/scan");
  target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");
  map_topic_ = this->declare_parameter<std::string>("map_topic", "/vertical_map");
  cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/vertical_cloud");
  global_map_topic_ = this->declare_parameter<std::string>("global_map_topic", "/mapping/global_cloud");
  status_topic_ = this->declare_parameter<std::string>("status_topic", "/mapping/status");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  lidar_frame_override_ = this->declare_parameter<std::string>("lidar_frame_override", "");

  voxel_leaf_ = this->declare_parameter<double>("voxel_leaf", 0.15);
  max_points_ = this->declare_parameter<int>("max_points", 500000);
  keep_seconds_ = this->declare_parameter<double>("keep_seconds", 30.0);
  min_range_ = this->declare_parameter<double>("min_range", 0.2);
  max_range_ = this->declare_parameter<double>("max_range", 12.0);
  tf_timeout_ = this->declare_parameter<double>("tf_timeout", 0.05);
  debug_ = this->declare_parameter<bool>("debug", false);

  global_voxel_leaf_size_ = this->declare_parameter<double>("global_voxel_leaf_size", 0.20);
  max_global_points_ = this->declare_parameter<int>("max_global_points", 1500000);
  global_publish_hz_ = this->declare_parameter<double>("global_publish_hz", 2.0);
  drop_scan_on_excess_motion_ = this->declare_parameter<bool>("drop_scan_on_excess_motion", true);
  max_integration_yaw_rate_ = this->declare_parameter<double>("max_integration_yaw_rate", 0.8);
  motion_odom_topic_ = this->declare_parameter<std::string>("motion_odom_topic", "/mavros/local_position/odom");
  tilt_compensation_ = this->declare_parameter<bool>("tilt_compensation", true);
  enable_relative_pose_gate_ = this->declare_parameter<bool>("enable_relative_pose_gate", true);
  enable_map_rebase_ = this->declare_parameter<bool>("enable_map_rebase", true);
  map_rebase_map_frame_ = this->declare_parameter<std::string>("map_rebase_map_frame", "map");
  map_rebase_odom_frame_ = this->declare_parameter<std::string>("map_rebase_odom_frame", "odom");
  relative_pose_map_frame_ = this->declare_parameter<std::string>("relative_pose_map_frame", "map");
  relative_pose_odom_frame_ = this->declare_parameter<std::string>("relative_pose_odom_frame", "odom");
  map_rebase_translation_threshold_ =
    this->declare_parameter<double>("map_rebase_translation_threshold", 0.10);
  map_rebase_yaw_threshold_ =
    this->declare_parameter<double>("map_rebase_yaw_threshold", 0.03);
  relative_pose_translation_error_threshold_ =
    this->declare_parameter<double>("relative_pose_translation_error_threshold", 0.20);
  relative_pose_yaw_error_threshold_ =
    this->declare_parameter<double>("relative_pose_yaw_error_threshold", 0.12);
  relative_pose_min_motion_xy_ =
    this->declare_parameter<double>("relative_pose_min_motion_xy", 0.05);
  pcd_export_dir_ = this->declare_parameter<std::string>("pcd_export_dir", "/home/lehaitrung/vertical_mapper_exports");
  pcd_export_prefix_ = this->declare_parameter<std::string>("pcd_export_prefix", "vertical_global_map");
  pcd_export_binary_ = this->declare_parameter<bool>("pcd_export_binary", true);
  export_map2d_on_save_ = this->declare_parameter<bool>("export_map2d_on_save", true);
  export_trajectory_on_save_ = this->declare_parameter<bool>("export_trajectory_on_save", true);
  map2d_export_prefix_ = this->declare_parameter<std::string>("map2d_export_prefix", "vertical_map2d");
  trajectory_export_prefix_ = this->declare_parameter<std::string>("trajectory_export_prefix", "vertical_trajectory");
  map2d_resolution_ = this->declare_parameter<double>("map2d_resolution", 0.10);
  map2d_padding_m_ = this->declare_parameter<double>("map2d_padding_m", 1.0);
  trajectory_min_step_ = this->declare_parameter<double>("trajectory_min_step", 0.05);

  tf_filter_queue_size_ = this->declare_parameter<int>("tf_filter_queue_size", 50);
  const int tf_lookup_window_size_param = this->declare_parameter<int>("tf_lookup_window_size", 200);
  tf_lookup_window_size_ = static_cast<std::size_t>(std::max(20, tf_lookup_window_size_param));

  if (target_frame_.empty()) {
    target_frame_ = "odom";
    RCLCPP_WARN(this->get_logger(), "Parameter 'target_frame' was empty, defaulting to 'odom'.");
  }

  if (max_points_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "Parameter 'max_points' <= 0, disabling local point-cap pruning.");
  }

  if (voxel_leaf_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Parameter 'voxel_leaf' <= 0.0, local voxel filtering is disabled.");
  }

  if (global_voxel_leaf_size_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter 'global_voxel_leaf_size' <= 0.0, global downsampling is disabled (not recommended).");
  }

  if (max_global_points_ <= 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter 'max_global_points' <= 0, global point cap disabled (not recommended).");
  }

  if (global_publish_hz_ <= 0.0) {
    global_publish_hz_ = 2.0;
    RCLCPP_WARN(this->get_logger(), "Parameter 'global_publish_hz' <= 0.0, defaulting to 2.0 Hz.");
  }

  if (tf_timeout_ < 0.0) {
    tf_timeout_ = 0.0;
    RCLCPP_WARN(this->get_logger(), "Parameter 'tf_timeout' was negative, clamped to 0.0.");
  }

  if (map_rebase_translation_threshold_ < 0.0) {
    map_rebase_translation_threshold_ = 0.0;
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter 'map_rebase_translation_threshold' was negative, clamped to 0.0.");
  }

  if (map_rebase_yaw_threshold_ < 0.0) {
    map_rebase_yaw_threshold_ = 0.0;
    RCLCPP_WARN(this->get_logger(), "Parameter 'map_rebase_yaw_threshold' was negative, clamped to 0.0.");
  }

  if (relative_pose_translation_error_threshold_ < 0.0) {
    relative_pose_translation_error_threshold_ = 0.0;
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter 'relative_pose_translation_error_threshold' was negative, clamped to 0.0.");
  }

  if (relative_pose_yaw_error_threshold_ < 0.0) {
    relative_pose_yaw_error_threshold_ = 0.0;
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter 'relative_pose_yaw_error_threshold' was negative, clamped to 0.0.");
  }

  if (relative_pose_min_motion_xy_ < 0.0) {
    relative_pose_min_motion_xy_ = 0.0;
    RCLCPP_WARN(this->get_logger(), "Parameter 'relative_pose_min_motion_xy' was negative, clamped to 0.0.");
  }

  if (pcd_export_dir_.empty()) {
    pcd_export_dir_ = "/home/lehaitrung/vertical_mapper_exports";
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter 'pcd_export_dir' was empty, defaulting to /home/lehaitrung/vertical_mapper_exports.");
  }

  if (pcd_export_prefix_.empty()) {
    pcd_export_prefix_ = "vertical_global_map";
    RCLCPP_WARN(this->get_logger(), "Parameter 'pcd_export_prefix' was empty, defaulting to vertical_global_map.");
  }

  if (map2d_export_prefix_.empty()) {
    map2d_export_prefix_ = "vertical_map2d";
    RCLCPP_WARN(this->get_logger(), "Parameter 'map2d_export_prefix' was empty, defaulting to vertical_map2d.");
  }

  if (trajectory_export_prefix_.empty()) {
    trajectory_export_prefix_ = "vertical_trajectory";
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter 'trajectory_export_prefix' was empty, defaulting to vertical_trajectory.");
  }

  if (map2d_resolution_ <= 0.0) {
    map2d_resolution_ = 0.10;
    RCLCPP_WARN(this->get_logger(), "Parameter 'map2d_resolution' <= 0.0, defaulting to 0.10 m/cell.");
  }

  if (map2d_padding_m_ < 0.0) {
    map2d_padding_m_ = 0.0;
    RCLCPP_WARN(this->get_logger(), "Parameter 'map2d_padding_m' was negative, clamped to 0.0.");
  }

  if (trajectory_min_step_ < 0.0) {
    trajectory_min_step_ = 0.0;
    RCLCPP_WARN(this->get_logger(), "Parameter 'trajectory_min_step' was negative, clamped to 0.0.");
  }
}

void VerticalLidarMapper::maybeApplyMapFrameCorrection(const rclcpp::Time & stamp)
{
  if (!enable_map_rebase_ || target_frame_ != map_rebase_map_frame_) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_map_odom;
  try {
    tf_map_odom = tf_buffer_->lookupTransform(
      map_rebase_map_frame_,
      map_rebase_odom_frame_,
      stamp,
      tf2::durationFromSec(tf_timeout_));
  } catch (const tf2::TransformException &) {
    try {
      const auto latest = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
      tf_map_odom = tf_buffer_->lookupTransform(
        map_rebase_map_frame_,
        map_rebase_odom_frame_,
        latest,
        tf2::durationFromSec(tf_timeout_));
    } catch (const tf2::TransformException & ex_latest) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Map-rebase lookup failed (%s -> %s): %s",
        map_rebase_odom_frame_.c_str(),
        map_rebase_map_frame_.c_str(),
        ex_latest.what());
      return;
    }
  }

  if (!last_applied_map_to_odom_tf_.has_value()) {
    last_applied_map_to_odom_tf_ = tf_map_odom;
    return;
  }

  const tf2::Transform curr_tf = to_tf2_transform(tf_map_odom);
  const tf2::Transform prev_tf = to_tf2_transform(last_applied_map_to_odom_tf_.value());
  const tf2::Transform delta_tf = curr_tf * prev_tf.inverse();

  const double translation_norm = delta_tf.getOrigin().length();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(delta_tf.getRotation()).getRPY(roll, pitch, yaw);
  const double yaw_abs = std::fabs(yaw);

  if (translation_norm < map_rebase_translation_threshold_ && yaw_abs < map_rebase_yaw_threshold_) {
    return;
  }

  const Eigen::Matrix4f delta = to_eigen_matrix(delta_tf);

  if (global_cloud_ && !global_cloud_->empty()) {
    pcl::transformPointCloud(*global_cloud_, *global_cloud_, delta);
    global_points_total_ = global_cloud_->size();
  }

  for (auto & scan : scan_queue_) {
    if (scan.cloud && !scan.cloud->empty()) {
      pcl::transformPointCloud(*scan.cloud, *scan.cloud, delta);
    }
  }

  for (auto & point : trajectory_points_) {
    Eigen::Vector4f p(point.x, point.y, point.z, 1.0F);
    p = delta * p;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
  }

  if (last_trajectory_point_.has_value()) {
    Eigen::Vector4f p(
      last_trajectory_point_.value().x(),
      last_trajectory_point_.value().y(),
      last_trajectory_point_.value().z(),
      1.0F);
    p = delta * p;
    last_trajectory_point_ = Eigen::Vector3f(p.x(), p.y(), p.z());
  }

  last_applied_map_to_odom_tf_ = tf_map_odom;
  ++map_rebase_count_;
  last_map_rebase_translation_m_ = translation_norm;
  last_map_rebase_yaw_rad_ = yaw_abs;

  RCLCPP_INFO(
    this->get_logger(),
    "Applied map-rebase correction #%zu: dtrans=%.3fm dyaw=%.3frad (target_frame=%s).",
    map_rebase_count_,
    translation_norm,
    yaw_abs,
    target_frame_.c_str());
}

rclcpp::Time VerticalLidarMapper::resolveScanStamp(const sensor_msgs::msg::LaserScan & scan_msg) const
{
  const bool zero_stamp = (scan_msg.header.stamp.sec == 0) && (scan_msg.header.stamp.nanosec == 0);

  if (zero_stamp) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "LaserScan stamp is zero. Using node clock now(). Fix publisher timestamps if possible.");
    return this->now();
  }

  return rclcpp::Time(scan_msg.header.stamp, this->get_clock()->get_clock_type());
}

std::string VerticalLidarMapper::resolveSourceFrame(const sensor_msgs::msg::LaserScan & scan_msg) const
{
  if (!lidar_frame_override_.empty()) {
    return lidar_frame_override_;
  }

  if (!scan_msg.header.frame_id.empty()) {
    return scan_msg.header.frame_id;
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    5000,
    "LaserScan frame_id is empty and no lidar_frame_override provided. Falling back to 'lidar_vert_link'.");
  return "lidar_vert_link";
}

void VerticalLidarMapper::warnIfTimeMismatch(const rclcpp::Time & scan_stamp) const
{
  const double age_sec = std::fabs((this->now() - scan_stamp).seconds());
  if (age_sec > 2.0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Large time offset between now() and scan stamp: %.3fs. If using simulation, set use_sim_time=true and confirm /clock is active.",
      age_sec);
  }
}

void VerticalLidarMapper::pruneOldScans(const rclcpp::Time & newest_stamp)
{
  if (keep_seconds_ <= 0.0) {
    return;
  }

  while (!scan_queue_.empty()) {
    const double age = (newest_stamp - scan_queue_.front().stamp).seconds();
    if (age <= keep_seconds_) {
      break;
    }

    raw_points_total_ -= scan_queue_.front().points;
    scan_queue_.pop_front();
  }
}

void VerticalLidarMapper::enforceRawPointCap()
{
  if (max_points_ <= 0) {
    return;
  }

  const std::size_t point_cap = static_cast<std::size_t>(max_points_);
  while (scan_queue_.size() > 1 && raw_points_total_ > point_cap) {
    raw_points_total_ -= scan_queue_.front().points;
    scan_queue_.pop_front();
  }

  if (raw_points_total_ > point_cap && scan_queue_.size() == 1U) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Single scan (%zu points) exceeds max_points (%zu). Keeping latest scan only.",
      raw_points_total_,
      point_cap);
  }
}

void VerticalLidarMapper::recordTfLookupSampleMs(double duration_ms)
{
  tf_lookup_samples_ms_.push_back(duration_ms);
  if (tf_lookup_samples_ms_.size() > tf_lookup_window_size_) {
    tf_lookup_samples_ms_.pop_front();
  }
}

bool VerticalLidarMapper::shouldDropGlobalIntegration(double & yaw_rate, double & odom_age_sec) const
{
  yaw_rate = 0.0;
  odom_age_sec = 0.0;

  if (!drop_scan_on_excess_motion_ || !motion_odom_.has_value() || !last_scan_stamp_.has_value()) {
    return false;
  }

  const auto & odom = motion_odom_.value();
  yaw_rate = odom.twist.twist.angular.z;

  const rclcpp::Time odom_stamp(odom.header.stamp, this->get_clock()->get_clock_type());
  odom_age_sec = std::fabs((last_scan_stamp_.value() - odom_stamp).seconds());

  // Do not gate with stale motion estimates.
  if (odom_age_sec > 0.25) {
    return false;
  }

  return std::fabs(yaw_rate) > max_integration_yaw_rate_;
}

bool VerticalLidarMapper::shouldDropByRelativePoseConsistency(
  const rclcpp::Time & scan_stamp,
  double & translation_error_m,
  double & yaw_error_rad,
  double & motion_xy_m)
{
  translation_error_m = 0.0;
  yaw_error_rad = 0.0;
  motion_xy_m = 0.0;

  if (!enable_relative_pose_gate_) {
    return false;
  }

  geometry_msgs::msg::TransformStamped tf_map_base;
  geometry_msgs::msg::TransformStamped tf_odom_base;
  try {
    tf_map_base = tf_buffer_->lookupTransform(
      relative_pose_map_frame_,
      base_frame_,
      scan_stamp,
      tf2::durationFromSec(tf_timeout_));
    tf_odom_base = tf_buffer_->lookupTransform(
      relative_pose_odom_frame_,
      base_frame_,
      scan_stamp,
      tf2::durationFromSec(tf_timeout_));
  } catch (const tf2::TransformException & ex) {
    ++relative_pose_gate_lookup_failures_;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Relative-pose gate lookup failed (map='%s' odom='%s' base='%s'): %s",
      relative_pose_map_frame_.c_str(),
      relative_pose_odom_frame_.c_str(),
      base_frame_.c_str(),
      ex.what());
    return false;
  }

  if (!last_relative_pose_map_base_tf_.has_value() || !last_relative_pose_odom_base_tf_.has_value()) {
    last_relative_pose_map_base_tf_ = tf_map_base;
    last_relative_pose_odom_base_tf_ = tf_odom_base;
    return false;
  }

  const tf2::Transform curr_map_tf = to_tf2_transform(tf_map_base);
  const tf2::Transform prev_map_tf = to_tf2_transform(last_relative_pose_map_base_tf_.value());
  const tf2::Transform curr_odom_tf = to_tf2_transform(tf_odom_base);
  const tf2::Transform prev_odom_tf = to_tf2_transform(last_relative_pose_odom_base_tf_.value());

  const tf2::Transform map_delta = curr_map_tf * prev_map_tf.inverse();
  const tf2::Transform odom_delta = curr_odom_tf * prev_odom_tf.inverse();

  const double map_dx = map_delta.getOrigin().x();
  const double map_dy = map_delta.getOrigin().y();
  const double odom_dx = odom_delta.getOrigin().x();
  const double odom_dy = odom_delta.getOrigin().y();
  const double map_step_xy = std::hypot(map_dx, map_dy);
  const double odom_step_xy = std::hypot(odom_dx, odom_dy);
  motion_xy_m = std::max(map_step_xy, odom_step_xy);

  translation_error_m = std::hypot(map_dx - odom_dx, map_dy - odom_dy);

  double map_roll = 0.0;
  double map_pitch = 0.0;
  double map_yaw = 0.0;
  double odom_roll = 0.0;
  double odom_pitch = 0.0;
  double odom_yaw = 0.0;
  tf2::Matrix3x3(map_delta.getRotation()).getRPY(map_roll, map_pitch, map_yaw);
  tf2::Matrix3x3(odom_delta.getRotation()).getRPY(odom_roll, odom_pitch, odom_yaw);
  yaw_error_rad = std::fabs(normalize_angle(map_yaw - odom_yaw));

  last_relative_pose_map_base_tf_ = tf_map_base;
  last_relative_pose_odom_base_tf_ = tf_odom_base;
  last_relative_pose_translation_error_m_ = translation_error_m;
  last_relative_pose_yaw_error_rad_ = yaw_error_rad;
  last_relative_pose_motion_xy_m_ = motion_xy_m;

  if (motion_xy_m < relative_pose_min_motion_xy_) {
    return false;
  }

  return (translation_error_m > relative_pose_translation_error_threshold_) ||
         (yaw_error_rad > relative_pose_yaw_error_threshold_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VerticalLidarMapper::voxelDownsample(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
  double leaf_size) const
{
  if (!input_cloud || input_cloud->empty() || leaf_size <= 0.0) {
    return input_cloud;
  }

  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(input_cloud);
  voxel.setLeafSize(
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size));
  voxel.filter(*filtered_cloud);

  return filtered_cloud;
}

void VerticalLidarMapper::integrateGlobalCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & scan_cloud)
{
  if (!scan_cloud || scan_cloud->empty()) {
    return;
  }

  *global_cloud_ += *scan_cloud;

  if (global_voxel_leaf_size_ > 0.0) {
    global_cloud_ = voxelDownsample(global_cloud_, global_voxel_leaf_size_);
  }

  if (max_global_points_ > 0) {
    const std::size_t cap = static_cast<std::size_t>(max_global_points_);
    if (global_cloud_->size() > cap) {
      double adaptive_leaf = std::max(0.05, global_voxel_leaf_size_ > 0.0 ? global_voxel_leaf_size_ : 0.10);
      int iter = 0;
      while (global_cloud_->size() > cap && iter < 6) {
        adaptive_leaf *= 1.25;
        global_cloud_ = voxelDownsample(global_cloud_, adaptive_leaf);
        ++iter;
      }
      ++global_revoxelization_count_;

      if (global_cloud_->size() > cap) {
        global_cloud_->points.resize(cap);
        global_cloud_->width = static_cast<uint32_t>(global_cloud_->points.size());
        global_cloud_->height = 1;
        global_cloud_->is_dense = false;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Global map still exceeded max_global_points=%zu after re-voxelization. Hard-capped to limit.",
          cap);
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Global map exceeded max_global_points=%zu. Re-voxelized with adaptive leaf %.3f.",
          cap,
          adaptive_leaf);
      }
    }
  }

  global_points_total_ = global_cloud_->size();
  ++total_scans_global_integrated_;
}

void VerticalLidarMapper::recordTrajectoryPoint(
  const rclcpp::Time & stamp,
  const geometry_msgs::msg::TransformStamped & tf_target)
{
  const float x = static_cast<float>(tf_target.transform.translation.x);
  const float y = static_cast<float>(tf_target.transform.translation.y);
  const float z = static_cast<float>(tf_target.transform.translation.z);

  if (trajectory_min_step_ > 0.0 && last_trajectory_point_.has_value()) {
    const Eigen::Vector3f current(x, y, z);
    const double step = static_cast<double>((current - last_trajectory_point_.value()).norm());
    if (step < trajectory_min_step_) {
      return;
    }
  }

  trajectory_points_.push_back(TrajectoryPoint{stamp, x, y, z});
  last_trajectory_point_ = Eigen::Vector3f(x, y, z);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VerticalLidarMapper::buildVoxelizedMapCloud() const
{
  auto merged_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  merged_cloud->reserve(raw_points_total_);

  for (const auto & timed_scan : scan_queue_) {
    *merged_cloud += *(timed_scan.cloud);
  }

  if (voxel_leaf_ <= 0.0) {
    return merged_cloud;
  }

  return voxelDownsample(merged_cloud, voxel_leaf_);
}

void VerticalLidarMapper::publishMap(const rclcpp::Time & stamp)
{
  if (scan_queue_.empty()) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud = buildVoxelizedMapCloud();

  if (max_points_ > 0) {
    const std::size_t point_cap = static_cast<std::size_t>(max_points_);
    while (scan_queue_.size() > 1 && map_cloud->size() > point_cap) {
      raw_points_total_ -= scan_queue_.front().points;
      scan_queue_.pop_front();
      map_cloud = buildVoxelizedMapCloud();
    }

    if (scan_queue_.size() == 1U && map_cloud->size() > point_cap) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Voxelized local map has %zu points and still exceeds max_points (%zu) with one scan retained.",
        map_cloud->size(),
        point_cap);
    }
  }

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*map_cloud, map_msg);
  map_msg.header.frame_id = target_frame_;
  map_msg.header.stamp = stamp;
  map_pub_->publish(map_msg);

  if (debug_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "vertical_map(local): scans=%zu raw_points=%zu voxel_points=%zu",
      scan_queue_.size(),
      raw_points_total_,
      map_cloud->size());
  }
}

void VerticalLidarMapper::publishGlobalMap(const rclcpp::Time & stamp)
{
  if (!global_cloud_ || global_cloud_->empty()) {
    return;
  }

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*global_cloud_, map_msg);
  map_msg.header.frame_id = target_frame_;
  map_msg.header.stamp = stamp;
  global_map_pub_->publish(map_msg);
}

bool VerticalLidarMapper::saveGlobalCloudToPcd(std::string & output_path, std::string & error_message)
{
  if (!global_cloud_ || global_cloud_->empty()) {
    error_message = "Global cloud is empty.";
    return false;
  }

  auto cloud_copy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*global_cloud_);
  if (cloud_copy->empty()) {
    error_message = "Global cloud is empty after copy.";
    return false;
  }

  std::error_code ec;
  const std::filesystem::path export_dir(pcd_export_dir_);
  std::filesystem::create_directories(export_dir, ec);
  if (ec) {
    error_message = "Failed to create export directory: " + export_dir.string() + " (" + ec.message() + ")";
    return false;
  }

  const int64_t stamp_ns = this->now().nanoseconds();
  const int64_t stamp_sec = stamp_ns / 1000000000LL;
  const int64_t stamp_nsec = std::llabs(stamp_ns % 1000000000LL);
  std::ostringstream file_name;
  file_name << pcd_export_prefix_ << "_" << stamp_sec << "_" << std::setw(9) << std::setfill('0')
            << stamp_nsec << ".pcd";
  const std::filesystem::path output_file = export_dir / file_name.str();

  int rc = 0;
  if (pcd_export_binary_) {
    rc = pcl::io::savePCDFileBinary(output_file.string(), *cloud_copy);
  } else {
    rc = pcl::io::savePCDFileASCII(output_file.string(), *cloud_copy);
  }

  if (rc != 0) {
    error_message = "pcl::io::savePCDFile failed with code " + std::to_string(rc);
    return false;
  }

  output_path = output_file.string();
  last_pcd_export_path_ = output_path;
  ++pcd_export_count_;

  std::string map_pgm_path;
  std::string map_yaml_path;
  if (export_map2d_on_save_) {
    std::string map_error;
    const bool map_ok = save2DMapToPgm(
      cloud_copy,
      export_dir,
      stamp_sec,
      stamp_nsec,
      map_pgm_path,
      map_yaml_path,
      map_error);
    if (!map_ok) {
      error_message = "PCD saved, but 2D map export failed: " + map_error;
      return false;
    }

    last_map2d_export_path_ = map_pgm_path;
    last_map2d_yaml_path_ = map_yaml_path;
    ++map2d_export_count_;
  }

  std::string trajectory_path;
  if (export_trajectory_on_save_) {
    std::string trajectory_error;
    const bool trajectory_ok = saveTrajectoryToCsv(
      export_dir,
      stamp_sec,
      stamp_nsec,
      trajectory_path,
      trajectory_error);
    if (!trajectory_ok) {
      error_message = "PCD and 2D map saved, but trajectory export failed: " + trajectory_error;
      return false;
    }

    last_trajectory_export_path_ = trajectory_path;
    ++trajectory_export_count_;
  }

  return true;
}

bool VerticalLidarMapper::save2DMapToPgm(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_copy,
  const std::filesystem::path & export_dir,
  int64_t stamp_sec,
  int64_t stamp_nsec,
  std::string & pgm_path,
  std::string & yaml_path,
  std::string & error_message) const
{
  if (!cloud_copy || cloud_copy->empty()) {
    error_message = "Global cloud is empty.";
    return false;
  }

  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();

  for (const auto & point : cloud_copy->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }

  if (!std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) || !std::isfinite(max_y)) {
    error_message = "Cloud has no finite XY points.";
    return false;
  }

  const double pad = std::max(0.0, map2d_padding_m_);
  const double min_x_p = static_cast<double>(min_x) - pad;
  const double min_y_p = static_cast<double>(min_y) - pad;
  const double max_x_p = static_cast<double>(max_x) + pad;
  const double max_y_p = static_cast<double>(max_y) + pad;
  const double resolution = map2d_resolution_;

  const std::size_t width =
    static_cast<std::size_t>(std::max(1.0, std::ceil((max_x_p - min_x_p) / resolution) + 1.0));
  const std::size_t height =
    static_cast<std::size_t>(std::max(1.0, std::ceil((max_y_p - min_y_p) / resolution) + 1.0));

  const std::size_t cell_count = width * height;
  if (cell_count > 80000000ULL) {
    error_message = "2D map image would be too large (" + std::to_string(width) + "x" +
      std::to_string(height) + ").";
    return false;
  }

  std::vector<std::uint8_t> image(cell_count, 255U);

  for (const auto & point : cloud_copy->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }

    const int col = static_cast<int>(std::floor((static_cast<double>(point.x) - min_x_p) / resolution));
    const int row = static_cast<int>(std::floor((static_cast<double>(point.y) - min_y_p) / resolution));

    if (col < 0 || row < 0 || col >= static_cast<int>(width) || row >= static_cast<int>(height)) {
      continue;
    }

    const int img_row = static_cast<int>(height) - 1 - row;
    const std::size_t idx = static_cast<std::size_t>(img_row) * width + static_cast<std::size_t>(col);
    image[idx] = 0U;
  }

  std::ostringstream base_name;
  base_name << map2d_export_prefix_ << "_" << stamp_sec << "_" << std::setw(9) << std::setfill('0')
            << stamp_nsec;

  const std::filesystem::path pgm_file = export_dir / (base_name.str() + ".pgm");
  const std::filesystem::path yaml_file = export_dir / (base_name.str() + ".yaml");

  {
    std::ofstream pgm_out(pgm_file, std::ios::binary);
    if (!pgm_out.good()) {
      error_message = "Failed to open PGM output file: " + pgm_file.string();
      return false;
    }

    pgm_out << "P5\n" << width << " " << height << "\n255\n";
    pgm_out.write(reinterpret_cast<const char *>(image.data()), static_cast<std::streamsize>(image.size()));
    if (!pgm_out.good()) {
      error_message = "Failed while writing PGM file: " + pgm_file.string();
      return false;
    }
  }

  {
    std::ofstream yaml_out(yaml_file);
    if (!yaml_out.good()) {
      error_message = "Failed to open YAML output file: " + yaml_file.string();
      return false;
    }

    yaml_out << "image: " << pgm_file.filename().string() << "\n";
    yaml_out << "resolution: " << to_string_with_precision(resolution, 6) << "\n";
    yaml_out << "origin: [" << to_string_with_precision(min_x_p, 6) << ", "
             << to_string_with_precision(min_y_p, 6) << ", 0.0]\n";
    yaml_out << "negate: 0\n";
    yaml_out << "occupied_thresh: 0.65\n";
    yaml_out << "free_thresh: 0.196\n";
    if (!yaml_out.good()) {
      error_message = "Failed while writing YAML file: " + yaml_file.string();
      return false;
    }
  }

  pgm_path = pgm_file.string();
  yaml_path = yaml_file.string();
  return true;
}

bool VerticalLidarMapper::saveTrajectoryToCsv(
  const std::filesystem::path & export_dir,
  int64_t stamp_sec,
  int64_t stamp_nsec,
  std::string & output_path,
  std::string & error_message) const
{
  if (trajectory_points_.empty()) {
    error_message = "Trajectory is empty.";
    return false;
  }

  std::ostringstream file_name;
  file_name << trajectory_export_prefix_ << "_" << stamp_sec << "_" << std::setw(9) << std::setfill('0')
            << stamp_nsec << ".csv";
  const std::filesystem::path csv_file = export_dir / file_name.str();

  std::ofstream out(csv_file);
  if (!out.good()) {
    error_message = "Failed to open trajectory CSV output file: " + csv_file.string();
    return false;
  }

  out << "stamp_sec,stamp_nsec,x,y,z\n";
  for (const auto & point : trajectory_points_) {
    const int64_t ns = point.stamp.nanoseconds();
    const int64_t sec = ns / 1000000000LL;
    const int64_t nsec = std::llabs(ns % 1000000000LL);
    out << sec << "," << nsec << ","
        << to_string_with_precision(point.x, 6) << ","
        << to_string_with_precision(point.y, 6) << ","
        << to_string_with_precision(point.z, 6) << "\n";
  }

  if (!out.good()) {
    error_message = "Failed while writing trajectory CSV file: " + csv_file.string();
    return false;
  }

  output_path = csv_file.string();
  return true;
}

void VerticalLidarMapper::handleSavePcdRequest(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  std::string output_path;
  std::string error_message;

  const bool ok = saveGlobalCloudToPcd(output_path, error_message);
  response->success = ok;
  if (ok) {
    std::ostringstream oss;
    oss << "Saved PCD: " << output_path;
    if (export_map2d_on_save_ && !last_map2d_export_path_.empty()) {
      oss << " | 2D map: " << last_map2d_export_path_;
      if (!last_map2d_yaml_path_.empty()) {
        oss << " (yaml: " << last_map2d_yaml_path_ << ")";
      }
    }
    if (export_trajectory_on_save_ && !last_trajectory_export_path_.empty()) {
      oss << " | trajectory: " << last_trajectory_export_path_;
    }
    response->message = oss.str();
  } else {
    response->message = "Failed to save map assets: " + error_message;
  }

  if (ok) {
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
  }
}

void VerticalLidarMapper::publishStatus()
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "vertical_lidar_mapper";
  status.hardware_id = "dual_2d_lidar";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "running";

  if (tf_failures_ > 0 || tf_filter_drop_count_ > 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "running_with_tf_warnings";
  }

  double tf_p95_ms = 0.0;
  if (!tf_lookup_samples_ms_.empty()) {
    std::vector<double> sorted(tf_lookup_samples_ms_.begin(), tf_lookup_samples_ms_.end());
    std::sort(sorted.begin(), sorted.end());
    const std::size_t idx = static_cast<std::size_t>(
      std::floor(0.95 * static_cast<double>(sorted.size() - 1)));
    tf_p95_ms = sorted[idx];
  }

  auto add_kv = [&status](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(std::move(kv));
  };

  add_kv("target_frame", target_frame_);
  add_kv("total_scans_seen", std::to_string(total_scans_seen_));
  add_kv("total_scans_processed", std::to_string(total_scans_processed_));
  add_kv("total_scans_global_integrated", std::to_string(total_scans_global_integrated_));
  add_kv("dropped_excess_motion", std::to_string(dropped_excess_motion_count_));
  add_kv("tilt_compensation", tilt_compensation_ ? "true" : "false");
  add_kv("tf_filter_drops", std::to_string(tf_filter_drop_count_));
  add_kv("tf_lookup_failures", std::to_string(tf_failures_));
  add_kv("tf_lookup_p95_ms", to_string_with_precision(tf_p95_ms, 3));
  add_kv("local_raw_points", std::to_string(raw_points_total_));
  add_kv("global_points", std::to_string(global_points_total_));
  add_kv("global_revoxelizations", std::to_string(global_revoxelization_count_));
  add_kv("map_rebase_enabled", enable_map_rebase_ ? "true" : "false");
  add_kv("map_rebase_count", std::to_string(map_rebase_count_));
  add_kv("last_map_rebase_translation_m", to_string_with_precision(last_map_rebase_translation_m_, 3));
  add_kv("last_map_rebase_yaw_deg", to_string_with_precision(last_map_rebase_yaw_rad_ * 57.2957795, 3));
  add_kv("relative_pose_gate_enabled", enable_relative_pose_gate_ ? "true" : "false");
  add_kv("relative_pose_gate_drops", std::to_string(relative_pose_gate_drop_count_));
  add_kv("relative_pose_gate_lookup_failures", std::to_string(relative_pose_gate_lookup_failures_));
  add_kv(
    "relative_pose_translation_error_m",
    to_string_with_precision(last_relative_pose_translation_error_m_, 3));
  add_kv(
    "relative_pose_yaw_error_deg",
    to_string_with_precision(last_relative_pose_yaw_error_rad_ * 57.2957795, 3));
  add_kv("relative_pose_motion_xy_m", to_string_with_precision(last_relative_pose_motion_xy_m_, 3));
  add_kv("pcd_export_count", std::to_string(pcd_export_count_));
  add_kv("last_pcd_export_path", last_pcd_export_path_.empty() ? "-" : last_pcd_export_path_);
  add_kv("map2d_export_count", std::to_string(map2d_export_count_));
  add_kv("last_map2d_export_path", last_map2d_export_path_.empty() ? "-" : last_map2d_export_path_);
  add_kv("last_map2d_yaml_path", last_map2d_yaml_path_.empty() ? "-" : last_map2d_yaml_path_);
  add_kv("trajectory_export_count", std::to_string(trajectory_export_count_));
  add_kv(
    "last_trajectory_export_path",
    last_trajectory_export_path_.empty() ? "-" : last_trajectory_export_path_);
  add_kv("trajectory_points", std::to_string(trajectory_points_.size()));

  diagnostic_msgs::msg::DiagnosticArray array_msg;
  array_msg.header.stamp = this->now();
  array_msg.status.push_back(std::move(status));
  status_pub_->publish(array_msg);
}

void VerticalLidarMapper::onGlobalPublishTimer()
{
  publishGlobalMap(this->now());
}

void VerticalLidarMapper::onStatusTimer()
{
  publishStatus();
}

void VerticalLidarMapper::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
  ++total_scans_seen_;

  const rclcpp::Time scan_stamp = resolveScanStamp(*scan_msg);
  last_scan_stamp_ = scan_stamp;
  warnIfTimeMismatch(scan_stamp);

  const std::string source_frame = resolveSourceFrame(*scan_msg);
  if (source_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Cannot process scan because source frame is empty.");
    return;
  }

  sensor_msgs::msg::LaserScan filtered_scan = *scan_msg;
  filtered_scan.header.frame_id = source_frame;
  filtered_scan.header.stamp = scan_stamp;
  filtered_scan.range_min = std::max(filtered_scan.range_min, static_cast<float>(min_range_));
  filtered_scan.range_max = std::min(filtered_scan.range_max, static_cast<float>(max_range_));

  if (filtered_scan.range_min >= filtered_scan.range_max) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Invalid range window after filtering: [%.3f, %.3f].",
      filtered_scan.range_min,
      filtered_scan.range_max);
    return;
  }

  for (float & range : filtered_scan.ranges) {
    if (!std::isfinite(range) || range < min_range_ || range > max_range_) {
      range = std::numeric_limits<float>::quiet_NaN();
    }
  }

  sensor_msgs::msg::PointCloud2 cloud_lidar;
  try {
    const double range_cutoff = (max_range_ > 0.0) ? max_range_ : -1.0;
    projector_.projectLaser(
      filtered_scan,
      cloud_lidar,
      range_cutoff,
      laser_geometry::channel_option::None);
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Failed to project LaserScan: %s",
      ex.what());
    return;
  }

  if (cloud_lidar.width == 0U) {
    if (debug_) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Projected cloud is empty; skipping.");
    }
    return;
  }

  cloud_lidar.header.frame_id = source_frame;
  cloud_lidar.header.stamp = scan_stamp;

  geometry_msgs::msg::TransformStamped tf_target;
  sensor_msgs::msg::PointCloud2 cloud_target;

  const auto tf_lookup_start = std::chrono::steady_clock::now();
  try {
    // One lookup per accepted scan, then apply transform directly.
    tf_target = tf_buffer_->lookupTransform(
      target_frame_,
      source_frame,
      scan_stamp,
      tf2::durationFromSec(tf_timeout_));

    if (tilt_compensation_) {
      try {
        // Preserve sensor mounting transform while removing vehicle roll/pitch.
        // Build T_target_source = T_target_base(yaw-only) * T_base_source(static/extrinsic).
        const auto tf_target_base = tf_buffer_->lookupTransform(
          target_frame_,
          base_frame_,
          scan_stamp,
          tf2::durationFromSec(tf_timeout_));
        const auto tf_base_source = tf_buffer_->lookupTransform(
          base_frame_,
          source_frame,
          scan_stamp,
          tf2::durationFromSec(tf_timeout_));

        tf2::Quaternion q_tb(
          tf_target_base.transform.rotation.x,
          tf_target_base.transform.rotation.y,
          tf_target_base.transform.rotation.z,
          tf_target_base.transform.rotation.w);
        tf2::Quaternion q_bs(
          tf_base_source.transform.rotation.x,
          tf_base_source.transform.rotation.y,
          tf_base_source.transform.rotation.z,
          tf_base_source.transform.rotation.w);

        if (q_tb.length2() > 1e-12 && q_bs.length2() > 1e-12) {
          q_tb.normalize();
          q_bs.normalize();

          double roll = 0.0;
          double pitch = 0.0;
          double yaw = 0.0;
          tf2::Matrix3x3(q_tb).getRPY(roll, pitch, yaw);

          tf2::Quaternion q_tb_yaw_only;
          q_tb_yaw_only.setRPY(0.0, 0.0, yaw);
          q_tb_yaw_only.normalize();

          tf2::Quaternion q_ts = q_tb_yaw_only * q_bs;
          q_ts.normalize();

          const tf2::Vector3 t_tb(
            tf_target_base.transform.translation.x,
            tf_target_base.transform.translation.y,
            tf_target_base.transform.translation.z);
          const tf2::Vector3 t_bs(
            tf_base_source.transform.translation.x,
            tf_base_source.transform.translation.y,
            tf_base_source.transform.translation.z);
          const tf2::Vector3 t_ts = t_tb + tf2::quatRotate(q_tb_yaw_only, t_bs);

          tf_target.transform.translation.x = t_ts.x();
          tf_target.transform.translation.y = t_ts.y();
          tf_target.transform.translation.z = t_ts.z();
          tf_target.transform.rotation.x = q_ts.x();
          tf_target.transform.rotation.y = q_ts.y();
          tf_target.transform.rotation.z = q_ts.z();
          tf_target.transform.rotation.w = q_ts.w();
        } else {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Skipping tilt compensation because TF quaternion is near zero.");
        }
      } catch (const tf2::TransformException & ex_tilt) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Tilt compensation fallback to raw TF (%s).",
          ex_tilt.what());
      }
    }

    tf2::doTransform(cloud_lidar, cloud_target, tf_target);
  } catch (const tf2::TransformException & ex) {
    ++tf_failures_;
    ++tf_filter_drop_count_;
    if (target_frame_ == "map") {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "TF lookup/transform failed (%s -> %s): %s. If map->odom is missing, try target_frame='odom'.",
        source_frame.c_str(),
        target_frame_.c_str(),
        ex.what());
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "TF lookup/transform failed (%s -> %s): %s",
        source_frame.c_str(),
        target_frame_.c_str(),
        ex.what());
    }
    return;
  }
  const auto tf_lookup_end = std::chrono::steady_clock::now();
  const double tf_lookup_ms =
    std::chrono::duration<double, std::milli>(tf_lookup_end - tf_lookup_start).count();
  recordTfLookupSampleMs(tf_lookup_ms);

  cloud_target.header.frame_id = target_frame_;
  cloud_target.header.stamp = scan_stamp;
  cloud_pub_->publish(cloud_target);

  maybeApplyMapFrameCorrection(scan_stamp);
  recordTrajectoryPoint(scan_stamp, tf_target);

  auto scan_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(cloud_target, *scan_cloud);

  std::vector<int> valid_indices;
  pcl::removeNaNFromPointCloud(*scan_cloud, *scan_cloud, valid_indices);
  if (scan_cloud->empty()) {
    return;
  }

  // Keep local bounded queue for near-term obstacle checks / compatibility map topic.
  scan_queue_.push_back(TimedScan{scan_stamp, scan_cloud, scan_cloud->size()});
  raw_points_total_ += scan_cloud->size();

  pruneOldScans(scan_stamp);
  enforceRawPointCap();
  publishMap(scan_stamp);

  double yaw_rate = 0.0;
  double odom_age_sec = 0.0;
  const bool drop_excess_motion = shouldDropGlobalIntegration(yaw_rate, odom_age_sec);

  double relative_translation_error_m = 0.0;
  double relative_yaw_error_rad = 0.0;
  double relative_motion_xy_m = 0.0;
  const bool drop_relative_pose = shouldDropByRelativePoseConsistency(
    scan_stamp,
    relative_translation_error_m,
    relative_yaw_error_rad,
    relative_motion_xy_m);

  const bool drop_global = drop_excess_motion || drop_relative_pose;
  if (drop_global) {
    if (drop_excess_motion) {
      ++dropped_excess_motion_count_;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Dropped scan from global integration: |yaw_rate|=%.3f > %.3f rad/s (odom_age=%.3fs).",
        std::fabs(yaw_rate),
        max_integration_yaw_rate_,
        odom_age_sec);
    }

    if (drop_relative_pose) {
      ++relative_pose_gate_drop_count_;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Dropped scan by relative-pose gate: trans_err=%.3fm (th=%.3f), yaw_err=%.3frad (th=%.3f), motion_xy=%.3fm.",
        relative_translation_error_m,
        relative_pose_translation_error_threshold_,
        relative_yaw_error_rad,
        relative_pose_yaw_error_threshold_,
        relative_motion_xy_m);
    }
  } else {
    integrateGlobalCloud(scan_cloud);
  }

  ++total_scans_processed_;

  if (debug_) {
    const double tf_latency = (this->now() - scan_stamp).seconds();
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "scan processed: source='%s' target='%s' latest_points=%zu tf_latency=%.3fs tf_lookup=%.3fms",
      source_frame.c_str(),
      target_frame_.c_str(),
      scan_cloud->size(),
      tf_latency,
      tf_lookup_ms);
  }
}

}  // namespace vertical_lidar_mapper

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vertical_lidar_mapper::VerticalLidarMapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
