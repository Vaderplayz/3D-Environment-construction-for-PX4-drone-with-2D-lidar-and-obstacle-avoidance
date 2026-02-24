#include "core/CloudRenderer.h"

#include <pcl/common/common.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>

namespace pc_env_viewer {
namespace {

std::uint8_t ToByte(float value)
{
  const float clamped = std::clamp(value, 0.0F, 1.0F);
  return static_cast<std::uint8_t>(clamped * 255.0F);
}

void RainbowColor(float t, std::uint8_t & out_r, std::uint8_t & out_g, std::uint8_t & out_b)
{
  const float clamped = std::clamp(t, 0.0F, 1.0F);
  const float hue = (1.0F - clamped) * 240.0F;
  const float c = 1.0F;
  const float x = c * (1.0F - std::fabs(std::fmod(hue / 60.0F, 2.0F) - 1.0F));

  float r = 0.0F;
  float g = 0.0F;
  float b = 0.0F;

  if (hue < 60.0F) {
    r = c;
    g = x;
  } else if (hue < 120.0F) {
    r = x;
    g = c;
  } else if (hue < 180.0F) {
    g = c;
    b = x;
  } else {
    g = x;
    b = c;
  }

  out_r = ToByte(r);
  out_g = ToByte(g);
  out_b = ToByte(b);
}

template <typename PointT>
float AxisValue(const PointT & point, AxisColorAxis axis)
{
  switch (axis) {
    case AxisColorAxis::X:
      return point.x;
    case AxisColorAxis::Y:
      return point.y;
    case AxisColorAxis::Z:
    default:
      return point.z;
  }
}

template <typename PointT>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr BuildAxisRainbowCloud(
  const typename pcl::PointCloud<PointT>::Ptr & input,
  AxisColorAxis axis)
{
  auto output = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  if (!input || input->empty()) {
    return output;
  }

  float min_v = std::numeric_limits<float>::infinity();
  float max_v = -std::numeric_limits<float>::infinity();

  for (const auto & p : input->points) {
    const float v = AxisValue(p, axis);
    min_v = std::min(min_v, v);
    max_v = std::max(max_v, v);
  }

  const float range = std::max(1e-6F, max_v - min_v);

  output->points.resize(input->points.size());
  output->width = static_cast<std::uint32_t>(input->points.size());
  output->height = 1;
  output->is_dense = false;

  for (std::size_t i = 0; i < input->points.size(); ++i) {
    const auto & src = input->points[i];
    auto & dst = output->points[i];

    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;

    const float t = (AxisValue(src, axis) - min_v) / range;
    RainbowColor(t, dst.r, dst.g, dst.b);
  }

  return output;
}

template <typename PointT>
bool ComputeBoundsFromCloud(
  const typename pcl::PointCloud<PointT>::Ptr & cloud,
  Eigen::Vector3f & out_min,
  Eigen::Vector3f & out_max)
{
  if (!cloud || cloud->empty()) {
    return false;
  }

  PointT min_pt;
  PointT max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  out_min = Eigen::Vector3f(min_pt.x, min_pt.y, min_pt.z);
  out_max = Eigen::Vector3f(max_pt.x, max_pt.y, max_pt.z);
  return true;
}

}  // namespace

CloudRenderer::CloudRenderer()
{
}

void CloudRenderer::initialize(vtkRenderWindow * render_window)
{
  if (render_window == nullptr) {
    return;
  }

  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  viewer_.reset(new pcl::visualization::PCLVisualizer(renderer, render_window, "pc_env_viewer", false));
  viewer_->setBackgroundColor(background_[0], background_[1], background_[2]);
  viewer_->setupInteractor(render_window->GetInteractor(), render_window);
  render_window->Render();
}

bool CloudRenderer::display(const PreviewCloud & preview, ColorMode mode)
{
  if (!isInitialized()) {
    return false;
  }
  current_preview_ = preview;
  has_preview_ = true;
  current_mode_ = mode;
  return addCurrentCloud(mode);
}

bool CloudRenderer::setColorMode(ColorMode mode)
{
  if (!isInitialized()) {
    return false;
  }
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
    case ColorMode::AxisRainbow:
      return true;
    default:
      return false;
  }
}

void CloudRenderer::setAxisRainbowAxis(AxisColorAxis axis)
{
  axis_rainbow_axis_ = axis;
  if (has_preview_ && current_mode_ == ColorMode::AxisRainbow) {
    addCurrentCloud(current_mode_);
  }
}

void CloudRenderer::setTrajectoryPoints(const std::vector<Eigen::Vector3f> & points)
{
  trajectory_points_ = points;
  redrawTrajectory();
}

void CloudRenderer::clearTrajectory()
{
  trajectory_points_.clear();
  redrawTrajectory();
}

void CloudRenderer::setPointSize(float point_size)
{
  if (!isInitialized()) {
    return;
  }
  point_size_ = point_size;
  applyRenderProperties();
}

void CloudRenderer::setBackgroundColor(double r, double g, double b)
{
  if (!isInitialized()) {
    return;
  }
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
  autoFitCamera();
}

void CloudRenderer::autoFitCamera()
{
  if (!isInitialized() || !has_preview_) {
    return;
  }

  Eigen::Vector3f min_v;
  Eigen::Vector3f max_v;
  if (!computeBounds(min_v, max_v)) {
    viewer_->resetCamera();
    if (viewer_->getRenderWindow()) {
      viewer_->getRenderWindow()->Render();
    }
    return;
  }

  const Eigen::Vector3f center = (min_v + max_v) * 0.5F;
  const Eigen::Vector3f extent = max_v - min_v;

  const float max_extent = std::max({extent.x(), extent.y(), extent.z(), 1.0F});
  const float distance = std::max(3.0F, max_extent * 3.0F);

  Eigen::Vector3f eye_dir(1.25F, -1.0F, 0.85F);
  eye_dir.normalize();
  const Eigen::Vector3f eye = center + eye_dir * distance;

  viewer_->setCameraPosition(
    static_cast<double>(eye.x()),
    static_cast<double>(eye.y()),
    static_cast<double>(eye.z()),
    static_cast<double>(center.x()),
    static_cast<double>(center.y()),
    static_cast<double>(center.z()),
    0.0,
    0.0,
    1.0);

  viewer_->setCameraClipDistances(0.01, static_cast<double>(distance) * 50.0);

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
  if (!isInitialized()) {
    return false;
  }
  if (!has_preview_) {
    return false;
  }

  viewer_->removePointCloud(cloud_actor_id_);

  bool ok = false;
  if (mode == ColorMode::AxisRainbow) {
    if (current_preview_.type == PreviewType::XYZRGB && current_preview_.xyzrgb) {
      axis_rainbow_cloud_ = BuildAxisRainbowCloud<pcl::PointXYZRGB>(current_preview_.xyzrgb, axis_rainbow_axis_);
    } else if (current_preview_.type == PreviewType::XYZI && current_preview_.xyzi) {
      axis_rainbow_cloud_ = BuildAxisRainbowCloud<pcl::PointXYZI>(current_preview_.xyzi, axis_rainbow_axis_);
    } else {
      axis_rainbow_cloud_ = BuildAxisRainbowCloud<pcl::PointXYZ>(current_preview_.xyz, axis_rainbow_axis_);
    }
    ok = axis_rainbow_cloud_ && !axis_rainbow_cloud_->empty() &&
      viewer_->addPointCloud<pcl::PointXYZRGB>(axis_rainbow_cloud_, cloud_actor_id_);
  } else if (current_preview_.type == PreviewType::XYZRGB && current_preview_.xyzrgb) {
    if (mode == ColorMode::RGB) {
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(current_preview_.xyzrgb);
      if (rgb.isCapable()) {
        ok = viewer_->addPointCloud<pcl::PointXYZRGB>(current_preview_.xyzrgb, rgb, cloud_actor_id_);
      }
      if (!ok) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> fallback(current_preview_.xyzrgb, 230, 230, 230);
        ok = viewer_->addPointCloud<pcl::PointXYZRGB>(current_preview_.xyzrgb, fallback, cloud_actor_id_);
      }
    } else {
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> by_height(current_preview_.xyzrgb, "z");
      if (by_height.isCapable()) {
        ok = viewer_->addPointCloud<pcl::PointXYZRGB>(current_preview_.xyzrgb, by_height, cloud_actor_id_);
      }
      if (!ok) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> fallback(current_preview_.xyzrgb, 255, 160, 50);
        ok = viewer_->addPointCloud<pcl::PointXYZRGB>(current_preview_.xyzrgb, fallback, cloud_actor_id_);
      }
    }
  } else if (current_preview_.type == PreviewType::XYZI && current_preview_.xyzi) {
    if (mode == ColorMode::Intensity) {
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> by_intensity(current_preview_.xyzi, "intensity");
      if (by_intensity.isCapable()) {
        ok = viewer_->addPointCloud<pcl::PointXYZI>(current_preview_.xyzi, by_intensity, cloud_actor_id_);
      }
      if (!ok) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> fallback(current_preview_.xyzi, 70, 220, 220);
        ok = viewer_->addPointCloud<pcl::PointXYZI>(current_preview_.xyzi, fallback, cloud_actor_id_);
      }
    } else {
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> by_height(current_preview_.xyzi, "z");
      if (by_height.isCapable()) {
        ok = viewer_->addPointCloud<pcl::PointXYZI>(current_preview_.xyzi, by_height, cloud_actor_id_);
      }
      if (!ok) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> fallback(current_preview_.xyzi, 255, 180, 60);
        ok = viewer_->addPointCloud<pcl::PointXYZI>(current_preview_.xyzi, fallback, cloud_actor_id_);
      }
    }
  } else if (current_preview_.xyz) {
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> by_height(current_preview_.xyz, "z");
    if (by_height.isCapable()) {
      ok = viewer_->addPointCloud<pcl::PointXYZ>(current_preview_.xyz, by_height, cloud_actor_id_);
    }
    if (!ok) {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> fallback(current_preview_.xyz, 60, 210, 255);
      ok = viewer_->addPointCloud<pcl::PointXYZ>(current_preview_.xyz, fallback, cloud_actor_id_);
    }
  }

  if (!ok) {
    return false;
  }

  applyRenderProperties();
  redrawTrajectory();
  autoFitCamera();
  return true;
}

void CloudRenderer::applyRenderProperties()
{
  if (!isInitialized()) {
    return;
  }
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

bool CloudRenderer::isInitialized() const
{
  return viewer_ != nullptr;
}

void CloudRenderer::redrawTrajectory()
{
  if (!isInitialized()) {
    return;
  }

  for (std::size_t i = 0; i < trajectory_line_count_; ++i) {
    viewer_->removeShape(trajectory_line_prefix_ + std::to_string(i));
  }
  trajectory_line_count_ = 0;

  if (trajectory_points_.size() < 2U) {
    if (viewer_->getRenderWindow()) {
      viewer_->getRenderWindow()->Render();
    }
    return;
  }

  for (std::size_t i = 1; i < trajectory_points_.size(); ++i) {
    const Eigen::Vector3f & p0 = trajectory_points_[i - 1];
    const Eigen::Vector3f & p1 = trajectory_points_[i];
    pcl::PointXYZ a(p0.x(), p0.y(), p0.z());
    pcl::PointXYZ b(p1.x(), p1.y(), p1.z());

    const std::string id = trajectory_line_prefix_ + std::to_string(trajectory_line_count_);
    if (viewer_->addLine<pcl::PointXYZ>(a, b, 1.0, 0.1, 0.1, id)) {
      viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, id);
      ++trajectory_line_count_;
    }
  }

  if (viewer_->getRenderWindow()) {
    viewer_->getRenderWindow()->Render();
  }
}

bool CloudRenderer::computeBounds(Eigen::Vector3f & out_min, Eigen::Vector3f & out_max) const
{
  if (!has_preview_) {
    return false;
  }

  if (current_preview_.type == PreviewType::XYZRGB) {
    return ComputeBoundsFromCloud<pcl::PointXYZRGB>(current_preview_.xyzrgb, out_min, out_max);
  }
  if (current_preview_.type == PreviewType::XYZI) {
    return ComputeBoundsFromCloud<pcl::PointXYZI>(current_preview_.xyzi, out_min, out_max);
  }
  return ComputeBoundsFromCloud<pcl::PointXYZ>(current_preview_.xyz, out_min, out_max);
}

}  // namespace pc_env_viewer
