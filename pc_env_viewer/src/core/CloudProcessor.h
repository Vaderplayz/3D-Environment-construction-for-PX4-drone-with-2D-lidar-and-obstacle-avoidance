#pragma once

#include "core/CloudAsset.h"
#include "core/CloudLoader.h"
#include "core/Result.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>
#include <functional>
#include <string>
#include <vector>

namespace pc_env_viewer {

enum class ProcessErrorCode {
  Cancelled,
  ConversionFailure,
  EmptyCloud,
  DownsampleFailure,
  ExportFailure
};

struct ProcessError {
  ProcessErrorCode code{ProcessErrorCode::ConversionFailure};
  std::string message;
};

enum class PreviewType {
  XYZ,
  XYZI,
  XYZRGB
};

struct PreviewCloud {
  PreviewType type{PreviewType::XYZ};
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz{new pcl::PointCloud<pcl::PointXYZ>()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgb{new pcl::PointCloud<pcl::PointXYZRGB>()};

  bool hasRgb() const
  {
    return type == PreviewType::XYZRGB;
  }

  bool hasIntensity() const
  {
    return type == PreviewType::XYZI;
  }

  std::size_t pointCount() const
  {
    if (type == PreviewType::XYZRGB && xyzrgb) {
      return xyzrgb->size();
    }
    if (type == PreviewType::XYZI && xyzi) {
      return xyzi->size();
    }
    return xyz ? xyz->size() : 0U;
  }
};

struct ProcessedCloud {
  CloudAsset asset;
  PreviewCloud preview;
  std::vector<std::string> fields;
  ColorMode default_color_mode{ColorMode::Height};
};

class CloudProcessor {
public:
  using ProgressCallback = std::function<void(const std::string &)>;

  static Result<ProcessedCloud, ProcessError> buildPreview(
    const LoadedCloud & loaded,
    std::size_t max_preview_points,
    std::atomic_bool * cancel_token = nullptr,
    ProgressCallback progress = {});

  static Result<bool, ProcessError> savePreviewAsPcd(
    const PreviewCloud & preview,
    const std::string & output_path,
    bool binary = true);
};

}  // namespace pc_env_viewer
