#pragma once

#include "core/Result.h"

#include <array>
#include <cstddef>
#include <optional>
#include <string>

namespace pc_env_viewer {

struct CliOptions {
  std::optional<std::string> file_path;
  std::size_t max_preview_points{1500000};
  std::optional<std::array<double, 3>> background_rgb;
  float point_size{1.0F};
  bool headless_check{false};
  bool show_help{false};
};

Result<CliOptions, std::string> ParseCliOptions(int argc, char ** argv);
std::string BuildHelpText(const std::string & app_name);

}  // namespace pc_env_viewer
