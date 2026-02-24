#include "core/CloudLoader.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>

namespace pc_env_viewer {
namespace {

bool IsCancelled(const std::atomic_bool * cancel_token)
{
  return cancel_token != nullptr && cancel_token->load();
}

std::string ToLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

bool HasField(const pcl::PCLPointCloud2 & cloud, const std::string & field_name)
{
  const std::string needle = ToLower(field_name);
  for (const auto & field : cloud.fields) {
    if (ToLower(field.name) == needle) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> CollectFields(const pcl::PCLPointCloud2 & cloud)
{
  std::vector<std::string> names;
  names.reserve(cloud.fields.size());
  for (const auto & field : cloud.fields) {
    names.push_back(field.name);
  }
  return names;
}

LoadError MakeError(LoadErrorCode code, const std::string & message)
{
  LoadError error;
  error.code = code;
  error.message = message;
  return error;
}

}  // namespace

Result<LoadedCloud, LoadError> CloudLoader::load(
  const std::string & path,
  std::atomic_bool * cancel_token,
  ProgressCallback progress)
{
  if (progress) {
    progress("Loading");
  }

  if (!std::filesystem::exists(path)) {
    return Result<LoadedCloud, LoadError>::Err(
      MakeError(LoadErrorCode::IoFailure, "File does not exist: " + path));
  }

  const std::filesystem::path fs_path(path);
  const std::string extension = ToLower(fs_path.extension().string());

  auto raw_cloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());
  int rc = -1;

  if (extension == ".pcd") {
    rc = pcl::io::loadPCDFile(path, *raw_cloud);
  } else if (extension == ".ply") {
    rc = pcl::io::loadPLYFile(path, *raw_cloud);
  } else {
    return Result<LoadedCloud, LoadError>::Err(
      MakeError(LoadErrorCode::UnsupportedExtension, "Unsupported file extension: " + extension));
  }

  if (rc < 0) {
    return Result<LoadedCloud, LoadError>::Err(
      MakeError(LoadErrorCode::IoFailure, "Failed to load file: " + path));
  }

  if (IsCancelled(cancel_token)) {
    return Result<LoadedCloud, LoadError>::Err(
      MakeError(LoadErrorCode::IoFailure, "Loading cancelled"));
  }

  const std::size_t point_count = static_cast<std::size_t>(raw_cloud->width) * static_cast<std::size_t>(raw_cloud->height);
  if (point_count == 0U) {
    return Result<LoadedCloud, LoadError>::Err(
      MakeError(LoadErrorCode::EmptyCloud, "Cloud contains zero points"));
  }

  if (progress) {
    progress("Validating");
  }

  const bool has_x = HasField(*raw_cloud, "x");
  const bool has_y = HasField(*raw_cloud, "y");
  const bool has_z = HasField(*raw_cloud, "z");
  if (!has_x || !has_y || !has_z) {
    return Result<LoadedCloud, LoadError>::Err(
      MakeError(LoadErrorCode::MissingXYZ, "Cloud is missing one or more required fields: x, y, z"));
  }

  LoadedCloud loaded;
  loaded.path = path;
  loaded.raw_cloud = raw_cloud;
  loaded.fields = CollectFields(*raw_cloud);
  loaded.has_rgb = HasField(*raw_cloud, "rgb") || HasField(*raw_cloud, "rgba");
  loaded.has_intensity = HasField(*raw_cloud, "intensity");
  loaded.source_point_count = point_count;

  return Result<LoadedCloud, LoadError>::Ok(std::move(loaded));
}

}  // namespace pc_env_viewer
