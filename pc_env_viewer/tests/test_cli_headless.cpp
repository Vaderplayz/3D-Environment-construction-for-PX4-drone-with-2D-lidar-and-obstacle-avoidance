#include "core/CliOptions.h"
#include "core/CloudLoader.h"
#include "core/CloudProcessor.h"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>

namespace {

std::string TestData(const std::string & file_name)
{
  return (std::filesystem::path(TEST_DATA_DIR) / file_name).string();
}

}  // namespace

TEST_CASE("ParseCliOptions parses headless check flags", "[cli]")
{
  const std::string file = TestData("simple_xyz.pcd");
  const char * argv[] = {
    "pc_env_viewer",
    "--headless-check",
    "--file",
    file.c_str(),
    "--max-preview-points",
    "10"
  };

  auto parsed = pc_env_viewer::ParseCliOptions(static_cast<int>(std::size(argv)), const_cast<char **>(argv));
  REQUIRE(parsed.ok());
  CHECK(parsed.value().headless_check);
  REQUIRE(parsed.value().file_path.has_value());
  CHECK(parsed.value().file_path.value() == file);
  CHECK(parsed.value().max_preview_points == 10U);
}

TEST_CASE("ParseCliOptions rejects unknown options", "[cli]")
{
  const char * argv[] = {"pc_env_viewer", "--bad"};
  auto parsed = pc_env_viewer::ParseCliOptions(static_cast<int>(std::size(argv)), const_cast<char **>(argv));
  REQUIRE_FALSE(parsed.ok());
}

TEST_CASE("Headless pipeline loads and processes file", "[cli]")
{
  auto loaded = pc_env_viewer::CloudLoader::load(TestData("simple_xyz.pcd"));
  REQUIRE(loaded.ok());

  auto processed = pc_env_viewer::CloudProcessor::buildPreview(loaded.value(), 10);
  REQUIRE(processed.ok());
  CHECK(processed.value().asset.raw_point_count == 5U);
  CHECK(processed.value().asset.preview_point_count == 5U);
}
