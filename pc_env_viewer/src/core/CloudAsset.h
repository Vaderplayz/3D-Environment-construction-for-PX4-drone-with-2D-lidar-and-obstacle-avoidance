#pragma once

#include <Eigen/Core>

#include <chrono>
#include <cstddef>
#include <string>

namespace pc_env_viewer {

enum class ColorMode {
  RGB,
  Intensity,
  Height,
  AxisRainbow
};

enum class AxisColorAxis {
  X = 0,
  Y = 1,
  Z = 2
};

struct CloudAsset {
  std::string path;
  std::size_t raw_point_count{0};
  std::size_t preview_point_count{0};
  bool has_rgb{false};
  bool has_intensity{false};
  Eigen::Vector3f bounds_min{0.0F, 0.0F, 0.0F};
  Eigen::Vector3f bounds_max{0.0F, 0.0F, 0.0F};
  std::chrono::system_clock::time_point loaded_at{};
};

}  // namespace pc_env_viewer
