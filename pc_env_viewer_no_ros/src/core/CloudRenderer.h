#pragma once

#include "core/CloudAsset.h"
#include "core/CloudProcessor.h"

#include <pcl/visualization/pcl_visualizer.h>

#include <memory>
#include <string>

class vtkRenderWindow;

namespace pc_env_viewer {

class CloudRenderer {
public:
  CloudRenderer();

  void initialize(vtkRenderWindow * render_window);
  bool display(const PreviewCloud & preview, ColorMode mode);
  bool setColorMode(ColorMode mode);
  bool supportsColorMode(ColorMode mode) const;

  void setPointSize(float point_size);
  void setBackgroundColor(double r, double g, double b);
  void resetCamera();
  bool saveScreenshot(const std::string & output_path) const;

  pcl::visualization::PCLVisualizer::Ptr viewer() const
  {
    return viewer_;
  }

private:
  bool addCurrentCloud(ColorMode mode);
  void applyRenderProperties();

  pcl::visualization::PCLVisualizer::Ptr viewer_;
  PreviewCloud current_preview_;
  bool has_preview_{false};
  ColorMode current_mode_{ColorMode::Height};
  float point_size_{1.0F};
  double background_[3]{0.08, 0.08, 0.1};
  std::string cloud_actor_id_{"cloud"};
};

}  // namespace pc_env_viewer
