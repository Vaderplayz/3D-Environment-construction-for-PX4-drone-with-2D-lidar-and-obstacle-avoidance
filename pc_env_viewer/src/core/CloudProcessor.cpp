#include "core/CloudProcessor.h"

#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <cmath>
#include <limits>
#include <utility>

namespace pc_env_viewer {
namespace {

bool IsCancelled(const std::atomic_bool * cancel_token)
{
  return cancel_token != nullptr && cancel_token->load();
}

ProcessError MakeError(ProcessErrorCode code, const std::string & message)
{
  ProcessError err;
  err.code = code;
  err.message = message;
  return err;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ConvertAndClean(const pcl::PCLPointCloud2 & input)
{
  auto converted = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  pcl::fromPCLPointCloud2(input, *converted);

  auto cleaned = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  std::vector<int> valid_indices;
  pcl::removeNaNFromPointCloud(*converted, *cleaned, valid_indices);
  return cleaned;
}

template <typename PointT>
std::pair<Eigen::Vector3f, Eigen::Vector3f> ComputeBounds(const pcl::PointCloud<PointT> & cloud)
{
  PointT min_pt;
  PointT max_pt;
  pcl::getMinMax3D(cloud, min_pt, max_pt);
  return {
    Eigen::Vector3f(min_pt.x, min_pt.y, min_pt.z),
    Eigen::Vector3f(max_pt.x, max_pt.y, max_pt.z)
  };
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr KeepFirstN(
  const typename pcl::PointCloud<PointT>::Ptr & input,
  std::size_t max_points)
{
  auto output = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  if (!input) {
    return output;
  }

  const std::size_t count = std::min<std::size_t>(input->size(), max_points);
  output->points.assign(input->points.begin(), input->points.begin() + static_cast<std::ptrdiff_t>(count));
  output->width = static_cast<std::uint32_t>(count);
  output->height = 1;
  output->is_dense = input->is_dense;
  return output;
}

template <typename PointT>
Result<typename pcl::PointCloud<PointT>::Ptr, ProcessError> DownsampleAdaptive(
  const typename pcl::PointCloud<PointT>::Ptr & input,
  std::size_t max_points,
  std::atomic_bool * cancel_token)
{
  if (!input || input->empty()) {
    return Result<typename pcl::PointCloud<PointT>::Ptr, ProcessError>::Err(
      MakeError(ProcessErrorCode::EmptyCloud, "Cannot downsample an empty cloud"));
  }

  if (max_points == 0U) {
    return Result<typename pcl::PointCloud<PointT>::Ptr, ProcessError>::Err(
      MakeError(ProcessErrorCode::DownsampleFailure, "max_points must be > 0"));
  }

  if (input->size() <= max_points) {
    auto copy = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>(*input));
    return Result<typename pcl::PointCloud<PointT>::Ptr, ProcessError>::Ok(std::move(copy));
  }

  PointT min_pt;
  PointT max_pt;
  pcl::getMinMax3D(*input, min_pt, max_pt);
  const Eigen::Vector3f span(max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z);
  const float diag = span.norm();
  if (!std::isfinite(diag) || diag <= std::numeric_limits<float>::epsilon()) {
    auto fallback = KeepFirstN<PointT>(input, max_points);
    return Result<typename pcl::PointCloud<PointT>::Ptr, ProcessError>::Ok(std::move(fallback));
  }

  const double ratio = std::cbrt(static_cast<double>(input->size()) / static_cast<double>(max_points));
  float leaf = std::max(0.0001F, (diag / 1200.0F) * static_cast<float>(std::max(1.0, ratio)));

  typename pcl::PointCloud<PointT>::Ptr best(new pcl::PointCloud<PointT>(*input));
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(input);

  constexpr int kMaxIterations = 12;
  for (int i = 0; i < kMaxIterations; ++i) {
    if (IsCancelled(cancel_token)) {
      return Result<typename pcl::PointCloud<PointT>::Ptr, ProcessError>::Err(
        MakeError(ProcessErrorCode::Cancelled, "Downsampling cancelled"));
    }

    auto candidate = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*candidate);

    if (!candidate->empty()) {
      best = candidate;
      if (candidate->size() <= max_points) {
        break;
      }
    }

    leaf *= 1.6F;
  }

  if (!best || best->empty()) {
    return Result<typename pcl::PointCloud<PointT>::Ptr, ProcessError>::Err(
      MakeError(ProcessErrorCode::DownsampleFailure, "Downsampling produced zero points"));
  }

  return Result<typename pcl::PointCloud<PointT>::Ptr, ProcessError>::Ok(std::move(best));
}

template <typename PointT>
CloudAsset BuildAsset(
  const LoadedCloud & loaded,
  const pcl::PointCloud<PointT> & raw_cloud,
  const pcl::PointCloud<PointT> & preview_cloud,
  bool has_rgb,
  bool has_intensity)
{
  CloudAsset asset;
  asset.path = loaded.path;
  asset.raw_point_count = raw_cloud.size();
  asset.preview_point_count = preview_cloud.size();
  asset.has_rgb = has_rgb;
  asset.has_intensity = has_intensity;
  asset.loaded_at = std::chrono::system_clock::now();

  const auto [min_v, max_v] = ComputeBounds(raw_cloud);
  asset.bounds_min = min_v;
  asset.bounds_max = max_v;

  return asset;
}

}  // namespace

Result<ProcessedCloud, ProcessError> CloudProcessor::buildPreview(
  const LoadedCloud & loaded,
  std::size_t max_preview_points,
  std::atomic_bool * cancel_token,
  ProgressCallback progress)
{
  if (max_preview_points == 0U) {
    return Result<ProcessedCloud, ProcessError>::Err(
      MakeError(ProcessErrorCode::DownsampleFailure, "max_preview_points must be > 0"));
  }

  if (IsCancelled(cancel_token)) {
    return Result<ProcessedCloud, ProcessError>::Err(
      MakeError(ProcessErrorCode::Cancelled, "Processing cancelled"));
  }

  ProcessedCloud output;
  output.fields = loaded.fields;

  try {
    if (loaded.has_rgb) {
      auto clean_rgb = ConvertAndClean<pcl::PointXYZRGB>(*loaded.raw_cloud);
      if (!clean_rgb || clean_rgb->empty()) {
        return Result<ProcessedCloud, ProcessError>::Err(
          MakeError(ProcessErrorCode::EmptyCloud, "Cloud has no valid XYZRGB points after NaN filtering"));
      }

      if (progress) {
        progress("Downsampling");
      }
      auto preview_result = DownsampleAdaptive<pcl::PointXYZRGB>(clean_rgb, max_preview_points, cancel_token);
      if (!preview_result.ok()) {
        return Result<ProcessedCloud, ProcessError>::Err(preview_result.take_error());
      }

      if (progress) {
        progress("Preparing Render");
      }
      output.preview.type = PreviewType::XYZRGB;
      output.preview.xyzrgb = preview_result.take_value();
      output.asset = BuildAsset(loaded, *clean_rgb, *output.preview.xyzrgb, true, false);
      output.default_color_mode = ColorMode::RGB;
      return Result<ProcessedCloud, ProcessError>::Ok(std::move(output));
    }

    if (loaded.has_intensity) {
      auto clean_xyzi = ConvertAndClean<pcl::PointXYZI>(*loaded.raw_cloud);
      if (!clean_xyzi || clean_xyzi->empty()) {
        return Result<ProcessedCloud, ProcessError>::Err(
          MakeError(ProcessErrorCode::EmptyCloud, "Cloud has no valid XYZI points after NaN filtering"));
      }

      if (progress) {
        progress("Downsampling");
      }
      auto preview_result = DownsampleAdaptive<pcl::PointXYZI>(clean_xyzi, max_preview_points, cancel_token);
      if (!preview_result.ok()) {
        return Result<ProcessedCloud, ProcessError>::Err(preview_result.take_error());
      }

      if (progress) {
        progress("Preparing Render");
      }
      output.preview.type = PreviewType::XYZI;
      output.preview.xyzi = preview_result.take_value();
      output.asset = BuildAsset(loaded, *clean_xyzi, *output.preview.xyzi, false, true);
      output.default_color_mode = ColorMode::Intensity;
      return Result<ProcessedCloud, ProcessError>::Ok(std::move(output));
    }

    auto clean_xyz = ConvertAndClean<pcl::PointXYZ>(*loaded.raw_cloud);
    if (!clean_xyz || clean_xyz->empty()) {
      return Result<ProcessedCloud, ProcessError>::Err(
        MakeError(ProcessErrorCode::EmptyCloud, "Cloud has no valid XYZ points after NaN filtering"));
    }

    if (progress) {
      progress("Downsampling");
    }
    auto preview_result = DownsampleAdaptive<pcl::PointXYZ>(clean_xyz, max_preview_points, cancel_token);
    if (!preview_result.ok()) {
      return Result<ProcessedCloud, ProcessError>::Err(preview_result.take_error());
    }

    if (progress) {
      progress("Preparing Render");
    }
    output.preview.type = PreviewType::XYZ;
    output.preview.xyz = preview_result.take_value();
    output.asset = BuildAsset(loaded, *clean_xyz, *output.preview.xyz, false, false);
    output.default_color_mode = ColorMode::Height;
    return Result<ProcessedCloud, ProcessError>::Ok(std::move(output));
  } catch (const std::exception & ex) {
    return Result<ProcessedCloud, ProcessError>::Err(
      MakeError(ProcessErrorCode::ConversionFailure, std::string("Conversion error: ") + ex.what()));
  }
}

Result<bool, ProcessError> CloudProcessor::savePreviewAsPcd(
  const PreviewCloud & preview,
  const std::string & output_path,
  bool binary)
{
  int rc = -1;
  if (preview.type == PreviewType::XYZRGB && preview.xyzrgb) {
    rc = binary ?
      pcl::io::savePCDFileBinary(output_path, *preview.xyzrgb) :
      pcl::io::savePCDFileASCII(output_path, *preview.xyzrgb);
  } else if (preview.type == PreviewType::XYZI && preview.xyzi) {
    rc = binary ?
      pcl::io::savePCDFileBinary(output_path, *preview.xyzi) :
      pcl::io::savePCDFileASCII(output_path, *preview.xyzi);
  } else if (preview.xyz) {
    rc = binary ?
      pcl::io::savePCDFileBinary(output_path, *preview.xyz) :
      pcl::io::savePCDFileASCII(output_path, *preview.xyz);
  }

  if (rc < 0) {
    return Result<bool, ProcessError>::Err(
      MakeError(ProcessErrorCode::ExportFailure, "Failed to save preview cloud to: " + output_path));
  }

  return Result<bool, ProcessError>::Ok(true);
}

}  // namespace pc_env_viewer
