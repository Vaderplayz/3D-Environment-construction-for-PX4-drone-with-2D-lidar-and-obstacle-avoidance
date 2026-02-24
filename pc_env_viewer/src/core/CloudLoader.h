#pragma once

#include "core/Result.h"

#include <pcl/PCLPointCloud2.h>

#include <atomic>
#include <functional>
#include <string>
#include <vector>

namespace pc_env_viewer {

enum class LoadErrorCode {
  UnsupportedExtension,
  IoFailure,
  EmptyCloud,
  MissingXYZ,
  CorruptData
};

struct LoadError {
  LoadErrorCode code{LoadErrorCode::IoFailure};
  std::string message;
};

struct LoadedCloud {
  std::string path;
  pcl::PCLPointCloud2::Ptr raw_cloud{new pcl::PCLPointCloud2};
  std::vector<std::string> fields;
  bool has_rgb{false};
  bool has_intensity{false};
  std::size_t source_point_count{0};
};

class CloudLoader {
public:
  using ProgressCallback = std::function<void(const std::string &)>;

  static Result<LoadedCloud, LoadError> load(
    const std::string & path,
    std::atomic_bool * cancel_token = nullptr,
    ProgressCallback progress = {});
};

}  // namespace pc_env_viewer
