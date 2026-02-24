#pragma once

#include "core/CloudAsset.h"
#include "core/CloudProcessor.h"

#include <pcl/visualization/pcl_visualizer.h>

#include <memory>
#include <string>
#include <vector>

class vtkRenderWindow;

namespace pc_env_viewer {

class CloudRenderer {
public:
  CloudRenderer();

  void initialize(vtkRenderWindow * render_window);
  bool display(const PreviewCloud & preview, ColorMode mode);
  bool setColorMode(ColorMode mode);
  bool supportsColorMode(ColorMode mode) const;
  void setAxisRainbowAxis(AxisColorAxis axis);
  void setTrajectoryPoints(const std::vector<Eigen::Vector3f> & points);
  void clearTrajectory();

  void setPointSize(float point_size);
  void setBackgroundColor(double r, double g, double b);
  void resetCamera();
  void autoFitCamera();
  bool saveScreenshot(const std::string & output_path) const;

  pcl::visualization::PCLVisualizer::Ptr viewer() const
  {
    return viewer_;
  }

private:
  bool addCurrentCloud(ColorMode mode);
  void applyRenderProperties();
  bool isInitialized() const;
  bool computeBounds(Eigen::Vector3f & out_min, Eigen::Vector3f & out_max) const;
  void redrawTrajectory();

  pcl::visualization::PCLVisualizer::Ptr viewer_;
  PreviewCloud current_preview_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr axis_rainbow_cloud_{new pcl::PointCloud<pcl::PointXYZRGB>()};
  std::vector<Eigen::Vector3f> trajectory_points_;
  std::size_t trajectory_line_count_{0};
  bool has_preview_{false};
  ColorMode current_mode_{ColorMode::Height};
  AxisColorAxis axis_rainbow_axis_{AxisColorAxis::Z};
  float point_size_{1.0F};
  double background_[3]{0.08, 0.08, 0.1};
  std::string cloud_actor_id_{"cloud"};
  std::string trajectory_line_prefix_{"trajectory_line_"};
};

}  // namespace pc_env_viewer
