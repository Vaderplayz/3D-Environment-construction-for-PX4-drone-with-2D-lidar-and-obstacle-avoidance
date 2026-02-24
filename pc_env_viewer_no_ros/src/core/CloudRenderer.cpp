#include "core/CloudRenderer.h"

#include <pcl/visualization/point_cloud_color_handlers.h>

#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>

#include <filesystem>

namespace pc_env_viewer {

CloudRenderer::CloudRenderer()
: viewer_(new pcl::visualization::PCLVisualizer("pc_env_viewer", false))
{
  viewer_->setBackgroundColor(background_[0], background_[1], background_[2]);
}

void CloudRenderer::initialize(vtkRenderWindow * render_window)
{
  if (render_window == nullptr) {
    return;
  }

  vtkRenderer * renderer = viewer_->getRendererCollection()->GetFirstRenderer();
  render_window->AddRenderer(renderer);
  viewer_->setupInteractor(render_window->GetInteractor(), render_window);
  render_window->Render();
}

bool CloudRenderer::display(const PreviewCloud & preview, ColorMode mode)
{
  current_preview_ = preview;
  has_preview_ = true;
  current_mode_ = mode;
  return addCurrentCloud(mode);
}

bool CloudRenderer::setColorMode(ColorMode mode)
{
  if (!supportsColorMode(mode) || !has_preview_) {
    return false;
  }
  current_mode_ = mode;
  return addCurrentCloud(mode);
}

bool CloudRenderer::supportsColorMode(ColorMode mode) const
{
  if (!has_preview_) {
    return false;
  }

  switch (mode) {
    case ColorMode::RGB:
      return current_preview_.hasRgb();
    case ColorMode::Intensity:
      return current_preview_.hasIntensity();
    case ColorMode::Height:
      return true;
    default:
      return false;
  }
}

void CloudRenderer::setPointSize(float point_size)
{
  point_size_ = point_size;
  applyRenderProperties();
}

void CloudRenderer::setBackgroundColor(double r, double g, double b)
{
  background_[0] = r;
  background_[1] = g;
  background_[2] = b;
  viewer_->setBackgroundColor(background_[0], background_[1], background_[2]);
  if (viewer_->getRenderWindow()) {
    viewer_->getRenderWindow()->Render();
  }
}

void CloudRenderer::resetCamera()
{
  viewer_->resetCamera();
  if (viewer_->getRenderWindow()) {
    viewer_->getRenderWindow()->Render();
  }
}

bool CloudRenderer::saveScreenshot(const std::string & output_path) const
{
  if (!viewer_ || !viewer_->getRenderWindow()) {
    return false;
  }
  viewer_->saveScreenshot(output_path);
  return std::filesystem::exists(output_path);
}

bool CloudRenderer::addCurrentCloud(ColorMode mode)
{
  if (!has_preview_) {
    return false;
  }

  viewer_->removePointCloud(cloud_actor_id_);

  bool ok = false;
  if (current_preview_.type == PreviewType::XYZRGB && current_preview_.xyzrgb) {
    if (mode == ColorMode::RGB) {
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(current_preview_.xyzrgb);
      ok = viewer_->addPointCloud<pcl::PointXYZRGB>(current_preview_.xyzrgb, rgb, cloud_actor_id_);
    } else {
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> by_height(current_preview_.xyzrgb, "z");
      ok = viewer_->addPointCloud<pcl::PointXYZRGB>(current_preview_.xyzrgb, by_height, cloud_actor_id_);
    }
  } else if (current_preview_.type == PreviewType::XYZI && current_preview_.xyzi) {
    if (mode == ColorMode::Intensity) {
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> by_intensity(current_preview_.xyzi, "intensity");
      ok = viewer_->addPointCloud<pcl::PointXYZI>(current_preview_.xyzi, by_intensity, cloud_actor_id_);
    } else {
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> by_height(current_preview_.xyzi, "z");
      ok = viewer_->addPointCloud<pcl::PointXYZI>(current_preview_.xyzi, by_height, cloud_actor_id_);
    }
  } else if (current_preview_.xyz) {
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> by_height(current_preview_.xyz, "z");
    ok = viewer_->addPointCloud<pcl::PointXYZ>(current_preview_.xyz, by_height, cloud_actor_id_);
  }

  if (!ok) {
    return false;
  }

  applyRenderProperties();
  viewer_->resetCamera();
  if (viewer_->getRenderWindow()) {
    viewer_->getRenderWindow()->Render();
  }
  return true;
}

void CloudRenderer::applyRenderProperties()
{
  if (!viewer_->contains(cloud_actor_id_)) {
    return;
  }
  viewer_->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
    point_size_,
    cloud_actor_id_);
  viewer_->setBackgroundColor(background_[0], background_[1], background_[2]);
  if (viewer_->getRenderWindow()) {
    viewer_->getRenderWindow()->Render();
  }
}

}  // namespace pc_env_viewer
