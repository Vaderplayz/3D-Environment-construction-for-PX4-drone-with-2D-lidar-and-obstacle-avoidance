#include "core/CloudLoader.h"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>

namespace {

std::string TestData(const std::string & file_name)
{
  return (std::filesystem::path(TEST_DATA_DIR) / file_name).string();
}

}  // namespace

TEST_CASE("CloudLoader loads valid XYZ PCD", "[loader]")
{
  auto result = pc_env_viewer::CloudLoader::load(TestData("simple_xyz.pcd"));
  REQUIRE(result.ok());
  CHECK(result.value().source_point_count == 5U);
  CHECK_FALSE(result.value().has_rgb);
  CHECK_FALSE(result.value().has_intensity);
}

TEST_CASE("CloudLoader loads valid RGB PLY", "[loader]")
{
  auto result = pc_env_viewer::CloudLoader::load(TestData("simple_rgb.ply"));
  REQUIRE(result.ok());
  CHECK(result.value().source_point_count == 4U);
  CHECK(result.value().has_rgb);
}

TEST_CASE("CloudLoader rejects unsupported extension", "[loader]")
{
  auto result = pc_env_viewer::CloudLoader::load(TestData("nope.txt"));
  REQUIRE_FALSE(result.ok());
  CHECK(result.error().code == pc_env_viewer::LoadErrorCode::UnsupportedExtension);
}

TEST_CASE("CloudLoader handles corrupt file", "[loader]")
{
  auto result = pc_env_viewer::CloudLoader::load(TestData("corrupt.pcd"));
  REQUIRE_FALSE(result.ok());
  CHECK(
    (result.error().code == pc_env_viewer::LoadErrorCode::IoFailure ||
    result.error().code == pc_env_viewer::LoadErrorCode::EmptyCloud));
}
