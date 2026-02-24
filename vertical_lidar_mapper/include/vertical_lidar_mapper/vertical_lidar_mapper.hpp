// Timestamp: 2026-02-21 11:18:02 +07+0700
// Most Recent Update: Metadata timestamp now uses local time
#ifndef VERTICAL_LIDAR_MAPPER__VERTICAL_LIDAR_MAPPER_HPP_
#define VERTICAL_LIDAR_MAPPER__VERTICAL_LIDAR_MAPPER_HPP_

#include <deque>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <message_filters/subscriber.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#endif

namespace vertical_lidar_mapper
{

class VerticalLidarMapper : public rclcpp::Node
{
public:
  explicit VerticalLidarMapper(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct TimedScan
  {
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::size_t points;
  };

  struct TrajectoryPoint
  {
    rclcpp::Time stamp;
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
  };

  void loadParameters();
  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

  std::string resolveSourceFrame(const sensor_msgs::msg::LaserScan & scan_msg) const;
  rclcpp::Time resolveScanStamp(const sensor_msgs::msg::LaserScan & scan_msg) const;

  void pruneOldScans(const rclcpp::Time & newest_stamp);
  void enforceRawPointCap();
  void recordTfLookupSampleMs(double duration_ms);
  bool shouldDropGlobalIntegration(double & yaw_rate, double & odom_age_sec) const;
  bool shouldDropByRelativePoseConsistency(
    const rclcpp::Time & scan_stamp,
    double & translation_error_m,
    double & yaw_error_rad,
    double & motion_xy_m);
  void maybeApplyMapFrameCorrection(const rclcpp::Time & stamp);
  void integrateGlobalCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & scan_cloud);
  void recordTrajectoryPoint(
    const rclcpp::Time & stamp,
    const geometry_msgs::msg::TransformStamped & tf_target);
  void publishGlobalMap(const rclcpp::Time & stamp);
  void publishStatus();
  void onGlobalPublishTimer();
  void onStatusTimer();
  void handleSavePcdRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool saveGlobalCloudToPcd(std::string & output_path, std::string & error_message);
  bool save2DMapToPgm(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_copy,
    const std::filesystem::path & export_dir,
    int64_t stamp_sec,
    int64_t stamp_nsec,
    std::string & pgm_path,
    std::string & yaml_path,
    std::string & error_message) const;
  bool saveTrajectoryToCsv(
    const std::filesystem::path & export_dir,
    int64_t stamp_sec,
    int64_t stamp_nsec,
    std::string & output_path,
    std::string & error_message) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownsample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
    double leaf_size) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr buildVoxelizedMapCloud() const;
  void publishMap(const rclcpp::Time & stamp);
  void warnIfTimeMismatch(const rclcpp::Time & scan_stamp) const;

  laser_geometry::LaserProjection projector_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_filter_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> scan_filter_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr motion_odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_pcd_service_;
  rclcpp::TimerBase::SharedPtr global_publish_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::CreateTimerROS> tf_timer_interface_;

  std::deque<TimedScan> scan_queue_;
  std::size_t raw_points_total_{0};
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_;
  std::size_t global_points_total_{0};
  std::deque<double> tf_lookup_samples_ms_;
  std::optional<nav_msgs::msg::Odometry> motion_odom_;
  std::optional<rclcpp::Time> last_scan_stamp_;
  std::optional<geometry_msgs::msg::TransformStamped> last_applied_map_to_odom_tf_;
  std::optional<geometry_msgs::msg::TransformStamped> last_relative_pose_map_base_tf_;
  std::optional<geometry_msgs::msg::TransformStamped> last_relative_pose_odom_base_tf_;
  std::vector<TrajectoryPoint> trajectory_points_;
  std::optional<Eigen::Vector3f> last_trajectory_point_;

  std::string scan_topic_;
  std::string target_frame_;
  std::string base_frame_;
  std::string lidar_frame_override_;
  std::string map_topic_;
  std::string cloud_topic_;
  std::string global_map_topic_;
  std::string status_topic_;
  std::string motion_odom_topic_;
  std::string pcd_export_dir_;
  std::string pcd_export_prefix_;
  std::string map2d_export_prefix_;
  std::string trajectory_export_prefix_;

  double voxel_leaf_{0.15};
  int max_points_{500000};
  double keep_seconds_{30.0};
  double min_range_{0.2};
  double max_range_{12.0};
  double tf_timeout_{0.05};
  bool debug_{false};

  double global_voxel_leaf_size_{0.20};
  int max_global_points_{1500000};
  double global_publish_hz_{2.0};
  bool drop_scan_on_excess_motion_{true};
  double max_integration_yaw_rate_{0.8};
  bool tilt_compensation_{true};
  bool enable_relative_pose_gate_{true};
  bool enable_map_rebase_{true};
  bool pcd_export_binary_{true};
  bool export_map2d_on_save_{true};
  bool export_trajectory_on_save_{true};
  double map2d_resolution_{0.10};
  double map2d_padding_m_{1.0};
  double trajectory_min_step_{0.05};
  std::string map_rebase_map_frame_{"map"};
  std::string map_rebase_odom_frame_{"odom"};
  std::string relative_pose_map_frame_{"map"};
  std::string relative_pose_odom_frame_{"odom"};
  double map_rebase_translation_threshold_{0.10};
  double map_rebase_yaw_threshold_{0.03};
  double relative_pose_translation_error_threshold_{0.20};
  double relative_pose_yaw_error_threshold_{0.12};
  double relative_pose_min_motion_xy_{0.05};
  int tf_filter_queue_size_{50};
  std::size_t tf_lookup_window_size_{200};
  std::size_t tf_failures_{0};
  std::size_t tf_filter_drop_count_{0};
  std::size_t dropped_excess_motion_count_{0};
  std::size_t global_revoxelization_count_{0};
  std::size_t total_scans_seen_{0};
  std::size_t total_scans_processed_{0};
  std::size_t total_scans_global_integrated_{0};
  std::size_t pcd_export_count_{0};
  std::string last_pcd_export_path_;
  std::size_t map2d_export_count_{0};
  std::size_t trajectory_export_count_{0};
  std::string last_map2d_export_path_;
  std::string last_map2d_yaml_path_;
  std::string last_trajectory_export_path_;
  std::size_t map_rebase_count_{0};
  double last_map_rebase_translation_m_{0.0};
  double last_map_rebase_yaw_rad_{0.0};
  std::size_t relative_pose_gate_drop_count_{0};
  std::size_t relative_pose_gate_lookup_failures_{0};
  double last_relative_pose_translation_error_m_{0.0};
  double last_relative_pose_yaw_error_rad_{0.0};
  double last_relative_pose_motion_xy_m_{0.0};
};

}  // namespace vertical_lidar_mapper

#endif  // VERTICAL_LIDAR_MAPPER__VERTICAL_LIDAR_MAPPER_HPP_
