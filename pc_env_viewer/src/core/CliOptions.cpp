#include "core/CliOptions.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <sstream>
#include <string_view>
#include <vector>

namespace pc_env_viewer {
namespace {

bool ParseSizeT(const std::string & text, std::size_t & out)
{
  if (text.empty()) {
    return false;
  }

  std::size_t parsed = 0;
  const char * begin = text.data();
  const char * end = begin + text.size();
  const auto [ptr, ec] = std::from_chars(begin, end, parsed);
  if (ec != std::errc() || ptr != end) {
    return false;
  }
  out = parsed;
  return true;
}

bool ParseFloat(const std::string & text, float & out)
{
  try {
    const float value = std::stof(text);
    if (!std::isfinite(value)) {
      return false;
    }
    out = value;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseBackground(const std::string & text, std::array<double, 3> & out)
{
  std::stringstream ss(text);
  std::string token;
  std::vector<double> values;

  while (std::getline(ss, token, ',')) {
    if (token.empty()) {
      return false;
    }
    try {
      const double value = std::stod(token);
      if (!std::isfinite(value)) {
        return false;
      }
      values.push_back(value);
    } catch (...) {
      return false;
    }
  }

  if (values.size() != 3) {
    return false;
  }

  const bool uses_byte_range = std::any_of(values.begin(), values.end(), [](double v) {return v > 1.0;});
  for (std::size_t i = 0; i < 3; ++i) {
    double normalized = values[i];
    if (uses_byte_range) {
      normalized /= 255.0;
    }
    normalized = std::clamp(normalized, 0.0, 1.0);
    out[i] = normalized;
  }

  return true;
}

bool NextValue(int argc, char ** argv, int & i, std::string & out)
{
  if (i + 1 >= argc) {
    return false;
  }
  ++i;
  out = argv[i];
  return true;
}

}  // namespace

Result<CliOptions, std::string> ParseCliOptions(int argc, char ** argv)
{
  CliOptions options;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);

    if (arg == "-h" || arg == "--help") {
      options.show_help = true;
      continue;
    }
    if (arg == "--headless-check") {
      options.headless_check = true;
      continue;
    }
    if (arg == "--file") {
      std::string value;
      if (!NextValue(argc, argv, i, value)) {
        return Result<CliOptions, std::string>::Err("--file requires a path argument");
      }
      options.file_path = value;
      continue;
    }
    if (arg == "--max-preview-points") {
      std::string value;
      if (!NextValue(argc, argv, i, value)) {
        return Result<CliOptions, std::string>::Err("--max-preview-points requires an integer argument");
      }
      std::size_t parsed = 0;
      if (!ParseSizeT(value, parsed) || parsed == 0) {
        return Result<CliOptions, std::string>::Err("--max-preview-points must be a positive integer");
      }
      options.max_preview_points = parsed;
      continue;
    }
    if (arg == "--point-size") {
      std::string value;
      if (!NextValue(argc, argv, i, value)) {
        return Result<CliOptions, std::string>::Err("--point-size requires a numeric argument");
      }
      float parsed = 0.0F;
      if (!ParseFloat(value, parsed) || parsed <= 0.0F) {
        return Result<CliOptions, std::string>::Err("--point-size must be a positive number");
      }
      options.point_size = parsed;
      continue;
    }
    if (arg == "--bg") {
      std::string value;
      if (!NextValue(argc, argv, i, value)) {
        return Result<CliOptions, std::string>::Err("--bg requires r,g,b");
      }
      std::array<double, 3> parsed{};
      if (!ParseBackground(value, parsed)) {
        return Result<CliOptions, std::string>::Err("--bg expects r,g,b in either [0,1] or [0,255]");
      }
      options.background_rgb = parsed;
      continue;
    }

    return Result<CliOptions, std::string>::Err("Unknown argument: " + arg);
  }

  if (options.headless_check && !options.file_path.has_value()) {
    return Result<CliOptions, std::string>::Err("--headless-check requires --file <path>");
  }

  return Result<CliOptions, std::string>::Ok(std::move(options));
}

std::string BuildHelpText(const std::string & app_name)
{
  std::ostringstream oss;
  oss << "Usage: " << app_name
      << " [--file <path>] [--max-preview-points <int>] [--bg <r,g,b>] [--point-size <float>] [--headless-check]\n"
      << "\n"
      << "Options:\n"
      << "  --file <path>              Auto-load file at startup\n"
      << "  --max-preview-points <n>   Preview point cap (default: 1500000)\n"
      << "  --bg <r,g,b>               Background color in [0..1] or [0..255]\n"
      << "  --point-size <float>       Initial point size (default: 1.0)\n"
      << "  --headless-check           Run load+preview pipeline without opening UI\n"
      << "  -h, --help                 Show this help\n";
  return oss.str();
}

}  // namespace pc_env_viewer
