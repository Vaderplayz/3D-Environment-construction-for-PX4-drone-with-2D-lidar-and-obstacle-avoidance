#include "app/MainWindow.h"
#include "core/CliOptions.h"
#include "core/CloudLoader.h"
#include "core/CloudProcessor.h"

#include <QApplication>

#include <iostream>

namespace pc_env_viewer {
namespace {

int RunHeadlessCheck(const CliOptions & options)
{
  if (!options.file_path.has_value()) {
    std::cerr << "--headless-check requires --file <path>" << std::endl;
    return 2;
  }

  auto loaded = CloudLoader::load(options.file_path.value());
  if (!loaded.ok()) {
    std::cerr << "load failed: " << loaded.error().message << std::endl;
    return 3;
  }

  auto processed = CloudProcessor::buildPreview(loaded.value(), options.max_preview_points);
  if (!processed.ok()) {
    std::cerr << "process failed: " << processed.error().message << std::endl;
    return 3;
  }

  std::cout << "ok: raw=" << processed.value().asset.raw_point_count
            << " preview=" << processed.value().asset.preview_point_count << std::endl;
  return 0;
}

}  // namespace
}  // namespace pc_env_viewer

int main(int argc, char ** argv)
{
  auto parsed = pc_env_viewer::ParseCliOptions(argc, argv);
  if (!parsed.ok()) {
    std::cerr << parsed.error() << std::endl;
    std::cerr << pc_env_viewer::BuildHelpText(argv[0]) << std::endl;
    return 2;
  }

  const auto options = parsed.value();
  if (options.show_help) {
    std::cout << pc_env_viewer::BuildHelpText(argv[0]) << std::endl;
    return 0;
  }

  if (options.headless_check) {
    return pc_env_viewer::RunHeadlessCheck(options);
  }

  QApplication app(argc, argv);
  app.setApplicationName("pc_env_viewer");
  app.setOrganizationName("pc_env_viewer");

  pc_env_viewer::MainWindow window(options);
  window.show();
  return app.exec();
}
