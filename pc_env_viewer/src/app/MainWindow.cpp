#include "app/MainWindow.h"

#include "core/CloudLoader.h"

#include <QCloseEvent>
#include <QColorDialog>
#include <QComboBox>
#include <QDesktopServices>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFile>
#include <QDragEnterEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QImage>
#include <QLabel>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMimeData>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QPolygon>
#include <QRegularExpression>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QStatusBar>
#include <QTextStream>
#include <QTimer>
#include <QToolBar>
#include <QUrl>

#include <QtConcurrent/QtConcurrent>

#include <cmath>
#include <filesystem>

namespace pc_env_viewer {
namespace {

constexpr int kMaxRecentFiles = 10;

bool LooksLikePointCloudFile(const QString & path)
{
  const QString lower = path.toLower();
  return lower.endsWith(".pcd") || lower.endsWith(".ply");
}

}  // namespace

MainWindow::MainWindow(const CliOptions & cli_options, QWidget * parent)
: QMainWindow(parent), options_(cli_options)
{
  setWindowTitle("PC Environment Viewer");
  setAcceptDrops(true);
  resize(1400, 900);

  viewer_widget_ = new ViewerWidget(this);
  setCentralWidget(viewer_widget_);

  auto * file_menu = menuBar()->addMenu("&File");
  open_action_ = file_menu->addAction("Open...");
  reload_action_ = file_menu->addAction("Reload");
  open_2d_map_action_ = file_menu->addAction("Open 2D Map...");
  open_trajectory_action_ = file_menu->addAction("Open Trajectory CSV...");
  screenshot_action_ = file_menu->addAction("Save Screenshot...");
  export_preview_action_ = file_menu->addAction("Export Preview as PCD...");
  recent_files_menu_ = file_menu->addMenu("Recent Files");
  file_menu->addSeparator();
  file_menu->addAction("Quit", this, &QWidget::close);

  auto * view_menu = menuBar()->addMenu("&View");
  reset_camera_action_ = view_menu->addAction("Reset Camera");
  auto_fit_action_ = view_menu->addAction("Auto Fit");
  auto_fit_action_->setShortcut(QKeySequence(Qt::Key_F));

  auto * toolbar = addToolBar("Main");
  toolbar->addAction(open_action_);
  toolbar->addAction(reload_action_);
  toolbar->addAction(auto_fit_action_);
  toolbar->addAction(reset_camera_action_);
  toolbar->addAction(screenshot_action_);

  auto * dock = new QDockWidget("Cloud Controls", this);
  auto * dock_content = new QWidget(dock);
  auto * layout = new QFormLayout(dock_content);

  raw_points_label_ = new QLabel("-", dock_content);
  preview_points_label_ = new QLabel("-", dock_content);
  bounds_label_ = new QLabel("-", dock_content);

  fields_text_ = new QPlainTextEdit(dock_content);
  fields_text_->setReadOnly(true);
  fields_text_->setMinimumHeight(110);

  color_mode_combo_ = new QComboBox(dock_content);
  color_mode_combo_->setEnabled(false);

  rainbow_axis_combo_ = new QComboBox(dock_content);
  rainbow_axis_combo_->addItem("X", static_cast<int>(AxisColorAxis::X));
  rainbow_axis_combo_->addItem("Y", static_cast<int>(AxisColorAxis::Y));
  rainbow_axis_combo_->addItem("Z", static_cast<int>(AxisColorAxis::Z));
  rainbow_axis_combo_->setCurrentIndex(2);
  rainbow_axis_combo_->setEnabled(false);

  point_size_spin_ = new QDoubleSpinBox(dock_content);
  point_size_spin_->setRange(1.0, 10.0);
  point_size_spin_->setSingleStep(0.5);
  point_size_spin_->setValue(static_cast<double>(options_.point_size));

  auto * background_button = new QPushButton("Pick...", dock_content);

  layout->addRow("Raw points", raw_points_label_);
  layout->addRow("Preview points", preview_points_label_);
  layout->addRow("Bounds", bounds_label_);
  layout->addRow("Fields", fields_text_);
  layout->addRow("Color mode", color_mode_combo_);
  layout->addRow("Rainbow axis", rainbow_axis_combo_);
  layout->addRow("Point size", point_size_spin_);
  layout->addRow("Background", background_button);

  dock_content->setLayout(layout);
  dock->setWidget(dock_content);
  addDockWidget(Qt::RightDockWidgetArea, dock);

  auto * map_dock = new QDockWidget("2D Map + Trajectory", this);
  map2d_scroll_area_ = new QScrollArea(map_dock);
  map2d_scroll_area_->setWidgetResizable(true);
  map2d_label_ = new QLabel(map2d_scroll_area_);
  map2d_label_->setAlignment(Qt::AlignCenter);
  map2d_label_->setMinimumSize(320, 240);
  map2d_label_->setText("No 2D map loaded.");
  map2d_scroll_area_->setWidget(map2d_label_);
  map_dock->setWidget(map2d_scroll_area_);
  addDockWidget(Qt::LeftDockWidgetArea, map_dock);

  progress_bar_ = new QProgressBar(this);
  progress_bar_->setRange(0, 0);
  progress_bar_->setVisible(false);
  statusBar()->addPermanentWidget(progress_bar_);
  statusBar()->showMessage("Ready");

  if (options_.background_rgb.has_value()) {
    const auto & bg = options_.background_rgb.value();
    viewer_widget_->setBackgroundColor(QColor::fromRgbF(bg[0], bg[1], bg[2]));
  }
  viewer_widget_->setPointSize(options_.point_size);

  connect(open_action_, &QAction::triggered, this, &MainWindow::openFileDialog);
  connect(reload_action_, &QAction::triggered, this, &MainWindow::reloadCurrentFile);
  connect(open_2d_map_action_, &QAction::triggered, this, &MainWindow::open2DMapDialog);
  connect(open_trajectory_action_, &QAction::triggered, this, &MainWindow::openTrajectoryDialog);
  connect(screenshot_action_, &QAction::triggered, this, &MainWindow::saveScreenshot);
  connect(export_preview_action_, &QAction::triggered, this, &MainWindow::exportPreviewAsPcd);
  connect(reset_camera_action_, &QAction::triggered, this, &MainWindow::resetCamera);
  connect(auto_fit_action_, &QAction::triggered, this, &MainWindow::autoFitCamera);
  connect(background_button, &QPushButton::clicked, this, &MainWindow::pickBackgroundColor);
  connect(color_mode_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::onColorModeChanged);
  connect(rainbow_axis_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::onRainbowAxisChanged);
  connect(point_size_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onPointSizeChanged);
  connect(&load_watcher_, &QFutureWatcher<PipelineResult>::finished, this, &MainWindow::onLoadWatcherFinished);
  connect(this, &MainWindow::loadProgress, this, &MainWindow::onLoadProgressMessage);

  loadRecentFiles();
  rebuildRecentFileMenu();

  if (options_.file_path.has_value()) {
    QTimer::singleShot(0, this, [this]() {
      startLoad(QString::fromStdString(options_.file_path.value()));
    });
  }
}

MainWindow::~MainWindow()
{
  if (cancel_token_) {
    cancel_token_->store(true);
  }
  if (load_watcher_.isRunning()) {
    load_watcher_.waitForFinished();
  }
}

void MainWindow::dragEnterEvent(QDragEnterEvent * event)
{
  if (!event->mimeData()->hasUrls()) {
    event->ignore();
    return;
  }

  for (const QUrl & url : event->mimeData()->urls()) {
    const QString path = url.toLocalFile();
    if (!path.isEmpty() && LooksLikePointCloudFile(path)) {
      event->acceptProposedAction();
      return;
    }
  }

  event->ignore();
}

void MainWindow::dropEvent(QDropEvent * event)
{
  for (const QUrl & url : event->mimeData()->urls()) {
    const QString path = url.toLocalFile();
    if (!path.isEmpty() && LooksLikePointCloudFile(path)) {
      startLoad(path);
      event->acceptProposedAction();
      return;
    }
  }
  event->ignore();
}

void MainWindow::closeEvent(QCloseEvent * event)
{
  if (cancel_token_) {
    cancel_token_->store(true);
  }
  if (load_watcher_.isRunning()) {
    load_watcher_.waitForFinished();
  }
  QMainWindow::closeEvent(event);
}

void MainWindow::openFileDialog()
{
  const QString file = QFileDialog::getOpenFileName(
    this,
    "Open Point Cloud",
    defaultOpenDirectory(),
    "Point Clouds (*.pcd *.ply)");
  if (!file.isEmpty()) {
    startLoad(file);
  }
}

void MainWindow::reloadCurrentFile()
{
  if (current_file_path_.isEmpty()) {
    return;
  }
  startLoad(current_file_path_);
}

void MainWindow::open2DMapDialog()
{
  const QString file = QFileDialog::getOpenFileName(
    this,
    "Open 2D Map",
    defaultOpenDirectory(),
    "2D Map Files (*.yaml *.yml *.pgm)");
  if (file.isEmpty()) {
    return;
  }

  QString error;
  if (!load2DMapAsset(file, &error)) {
    QMessageBox::critical(this, "2D Map Load Failed", error);
    return;
  }
  render2DMapWithTrajectory();
}

void MainWindow::openTrajectoryDialog()
{
  const QString file = QFileDialog::getOpenFileName(
    this,
    "Open Trajectory CSV",
    defaultOpenDirectory(),
    "CSV Files (*.csv)");
  if (file.isEmpty()) {
    return;
  }

  QString error;
  if (!loadTrajectoryCsv(file, &error)) {
    QMessageBox::critical(this, "Trajectory Load Failed", error);
    return;
  }
  render2DMapWithTrajectory();
}

void MainWindow::saveScreenshot()
{
  if (!current_cloud_.has_value()) {
    QMessageBox::information(this, "No Cloud", "Load a cloud before exporting a screenshot.");
    return;
  }

  QString default_path = QFileInfo(current_file_path_).completeBaseName() + "_screenshot.png";
  const QString output = QFileDialog::getSaveFileName(this, "Save Screenshot", default_path, "PNG (*.png)");
  if (output.isEmpty()) {
    return;
  }

  if (!viewer_widget_->saveScreenshot(output)) {
    QMessageBox::critical(this, "Export Failed", "Could not save screenshot.");
    return;
  }
  statusBar()->showMessage("Saved screenshot: " + output, 3000);
}

void MainWindow::exportPreviewAsPcd()
{
  if (!current_cloud_.has_value()) {
    QMessageBox::information(this, "No Cloud", "Load a cloud before exporting preview.");
    return;
  }

  QString default_path = QFileInfo(current_file_path_).completeBaseName() + "_preview.pcd";
  const QString output = QFileDialog::getSaveFileName(this, "Export Preview as PCD", default_path, "PCD (*.pcd)");
  if (output.isEmpty()) {
    return;
  }

  const auto result = CloudProcessor::savePreviewAsPcd(current_cloud_->preview, output.toStdString(), true);
  if (!result.ok()) {
    QMessageBox::critical(this, "Export Failed", QString::fromStdString(result.error().message));
    return;
  }
  statusBar()->showMessage("Saved preview: " + output, 3000);
}

void MainWindow::resetCamera()
{
  viewer_widget_->resetCamera();
}

void MainWindow::autoFitCamera()
{
  viewer_widget_->autoFitCamera();
}

void MainWindow::pickBackgroundColor()
{
  const QColor color = QColorDialog::getColor(Qt::black, this, "Select Background Color");
  if (!color.isValid()) {
    return;
  }
  viewer_widget_->setBackgroundColor(color);
}

void MainWindow::onColorModeChanged(int index)
{
  if (index < 0 || !current_cloud_.has_value()) {
    return;
  }

  const QVariant data = color_mode_combo_->itemData(index);
  if (!data.isValid()) {
    return;
  }

  const auto mode = static_cast<ColorMode>(data.toInt());
  viewer_widget_->setColorMode(mode);
  rainbow_axis_combo_->setEnabled(mode == ColorMode::AxisRainbow && current_cloud_.has_value());
}

void MainWindow::onRainbowAxisChanged(int index)
{
  if (index < 0 || !current_cloud_.has_value()) {
    return;
  }

  const QVariant data = rainbow_axis_combo_->itemData(index);
  if (!data.isValid()) {
    return;
  }

  const auto axis = static_cast<AxisColorAxis>(data.toInt());
  viewer_widget_->setAxisRainbowAxis(axis);
}

void MainWindow::onPointSizeChanged(double value)
{
  viewer_widget_->setPointSize(static_cast<float>(value));
}

void MainWindow::onLoadWatcherFinished()
{
  setBusy(false);

  const PipelineResult result = load_watcher_.result();
  if (!result.ok || !result.processed.has_value()) {
    const QString message = result.error_message.isEmpty() ? "Unknown load error" : result.error_message;
    emit loadFailed(message);
    QMessageBox::critical(this, "Load Failed", message);
    statusBar()->showMessage("Load failed", 3000);
    return;
  }

  current_cloud_ = result.processed;
  current_file_path_ = QString::fromStdString(current_cloud_->asset.path);

  if (!viewer_widget_->displayCloud(current_cloud_->preview, current_cloud_->default_color_mode)) {
    QMessageBox::critical(this, "Render Failed", "Loaded cloud but failed to render it.");
    statusBar()->showMessage("Render failed", 3000);
    return;
  }

  viewer_widget_->setPointSize(static_cast<float>(point_size_spin_->value()));
  if (rainbow_axis_combo_->currentData().isValid()) {
    const auto axis = static_cast<AxisColorAxis>(rainbow_axis_combo_->currentData().toInt());
    viewer_widget_->setAxisRainbowAxis(axis);
  }
  updateStatsPanel(*current_cloud_);
  updateColorModeControls(*current_cloud_);
  viewer_widget_->autoFitCamera();
  tryLoadAssociatedMapAssets(current_file_path_);
  addRecentFile(current_file_path_);

  emit loadFinished(current_file_path_);
  statusBar()->showMessage("Loaded: " + current_file_path_, 3000);
}

void MainWindow::onLoadProgressMessage(const QString & phase)
{
  statusBar()->showMessage("Working: " + phase);
}

MainWindow::PipelineResult MainWindow::runLoadPipeline(
  const QString & path,
  std::size_t max_preview_points,
  const std::shared_ptr<std::atomic_bool> & cancel_token,
  const QPointer<MainWindow> & receiver)
{
  PipelineResult out;

  auto report = [receiver](const std::string & phase) {
      if (!receiver) {
        return;
      }
      const QString phase_text = QString::fromStdString(phase);
      QMetaObject::invokeMethod(
        receiver,
        [receiver, phase_text]() {
          if (!receiver) {
            return;
          }
          emit receiver->loadProgress(phase_text);
        },
        Qt::QueuedConnection);
    };

  auto loaded_result = CloudLoader::load(path.toStdString(), cancel_token.get(), report);
  if (!loaded_result.ok()) {
    out.ok = false;
    out.error_message = QString::fromStdString(loaded_result.error().message);
    return out;
  }

  auto processed_result = CloudProcessor::buildPreview(
    loaded_result.value(),
    max_preview_points,
    cancel_token.get(),
    report);

  if (!processed_result.ok()) {
    out.ok = false;
    out.error_message = QString::fromStdString(processed_result.error().message);
    return out;
  }

  out.ok = true;
  out.processed = processed_result.take_value();
  return out;
}

void MainWindow::startLoad(const QString & path)
{
  if (path.isEmpty()) {
    return;
  }

  if (!LooksLikePointCloudFile(path)) {
    QMessageBox::warning(this, "Unsupported File", "Only .pcd and .ply files are supported.");
    return;
  }

  if (cancel_token_) {
    cancel_token_->store(true);
  }
  if (load_watcher_.isRunning()) {
    load_watcher_.waitForFinished();
  }

  cancel_token_ = std::make_shared<std::atomic_bool>(false);
  setBusy(true);
  emit requestLoad(path);
  emit loadStarted();

  const QPointer<MainWindow> receiver(this);
  auto future = QtConcurrent::run(
    [path, max_preview_points = options_.max_preview_points, cancel = cancel_token_, receiver]() {
      return MainWindow::runLoadPipeline(path, max_preview_points, cancel, receiver);
    });

  load_watcher_.setFuture(future);
}

void MainWindow::setBusy(bool busy)
{
  progress_bar_->setVisible(busy);
  open_action_->setEnabled(!busy);
  reload_action_->setEnabled(!busy && !current_file_path_.isEmpty());
  open_2d_map_action_->setEnabled(!busy);
  open_trajectory_action_->setEnabled(!busy);
  screenshot_action_->setEnabled(!busy && current_cloud_.has_value());
  export_preview_action_->setEnabled(!busy && current_cloud_.has_value());
  reset_camera_action_->setEnabled(!busy && current_cloud_.has_value());
  auto_fit_action_->setEnabled(!busy && current_cloud_.has_value());
  color_mode_combo_->setEnabled(!busy && current_cloud_.has_value());
  const bool axis_mode = color_mode_combo_->currentData().toInt() == static_cast<int>(ColorMode::AxisRainbow);
  rainbow_axis_combo_->setEnabled(!busy && current_cloud_.has_value() && axis_mode);
}

void MainWindow::updateStatsPanel(const ProcessedCloud & cloud)
{
  raw_points_label_->setText(QString::number(static_cast<qlonglong>(cloud.asset.raw_point_count)));
  preview_points_label_->setText(QString::number(static_cast<qlonglong>(cloud.asset.preview_point_count)));

  const auto & min_v = cloud.asset.bounds_min;
  const auto & max_v = cloud.asset.bounds_max;
  const QString bounds = QString("min=(%1, %2, %3) max=(%4, %5, %6)")
                           .arg(min_v.x(), 0, 'f', 3)
                           .arg(min_v.y(), 0, 'f', 3)
                           .arg(min_v.z(), 0, 'f', 3)
                           .arg(max_v.x(), 0, 'f', 3)
                           .arg(max_v.y(), 0, 'f', 3)
                           .arg(max_v.z(), 0, 'f', 3);
  bounds_label_->setText(bounds);

  QStringList fields;
  for (const auto & field : cloud.fields) {
    fields.push_back(QString::fromStdString(field));
  }
  fields_text_->setPlainText(fields.join("\n"));
}

void MainWindow::updateColorModeControls(const ProcessedCloud & cloud)
{
  color_mode_combo_->blockSignals(true);
  color_mode_combo_->clear();

  auto add_mode = [this](ColorMode mode) {
      color_mode_combo_->addItem(colorModeLabel(mode), static_cast<int>(mode));
    };

  if (cloud.preview.hasRgb()) {
    add_mode(ColorMode::RGB);
  }
  if (cloud.preview.hasIntensity()) {
    add_mode(ColorMode::Intensity);
  }
  add_mode(ColorMode::Height);
  add_mode(ColorMode::AxisRainbow);

  for (int i = 0; i < color_mode_combo_->count(); ++i) {
    if (color_mode_combo_->itemData(i).toInt() == static_cast<int>(cloud.default_color_mode)) {
      color_mode_combo_->setCurrentIndex(i);
      break;
    }
  }

  color_mode_combo_->setEnabled(true);
  const bool axis_mode = color_mode_combo_->currentData().toInt() == static_cast<int>(ColorMode::AxisRainbow);
  rainbow_axis_combo_->setEnabled(axis_mode);
  color_mode_combo_->blockSignals(false);
}

void MainWindow::tryLoadAssociatedMapAssets(const QString & cloud_path)
{
  current_map2d_path_.clear();
  current_trajectory_path_.clear();
  map2d_base_image_ = QImage();
  trajectory_points_world_.clear();
  viewer_widget_->clearTrajectory();

  if (map2d_label_ != nullptr) {
    map2d_label_->setPixmap(QPixmap());
    map2d_label_->setText("No 2D map loaded.");
  }

  const QFileInfo cloud_info(cloud_path);
  if (!cloud_info.exists()) {
    return;
  }

  const QRegularExpression pattern(".*_(\\d+)_(\\d{9})\\.pcd$", QRegularExpression::CaseInsensitiveOption);
  const QRegularExpressionMatch match = pattern.match(cloud_info.fileName());
  if (!match.hasMatch()) {
    return;
  }

  const QString sec = match.captured(1);
  const QString nsec = match.captured(2);
  const QDir dir(cloud_info.absolutePath());

  const QString yaml_candidate = dir.filePath(QString("vertical_map2d_%1_%2.yaml").arg(sec, nsec));
  const QString pgm_candidate = dir.filePath(QString("vertical_map2d_%1_%2.pgm").arg(sec, nsec));
  const QString trajectory_candidate = dir.filePath(QString("vertical_trajectory_%1_%2.csv").arg(sec, nsec));

  QString err;
  if (QFileInfo::exists(yaml_candidate)) {
    if (!load2DMapAsset(yaml_candidate, &err)) {
      statusBar()->showMessage("Associated 2D map load failed: " + err, 5000);
    }
  } else if (QFileInfo::exists(pgm_candidate)) {
    if (!load2DMapAsset(pgm_candidate, &err)) {
      statusBar()->showMessage("Associated 2D map load failed: " + err, 5000);
    }
  }

  if (QFileInfo::exists(trajectory_candidate)) {
    if (!loadTrajectoryCsv(trajectory_candidate, &err)) {
      statusBar()->showMessage("Associated trajectory load failed: " + err, 5000);
    }
  }

  render2DMapWithTrajectory();
}

bool MainWindow::loadMapMetadataFromYaml(
  const QString & yaml_path,
  QString & image_path,
  double & resolution,
  double & origin_x,
  double & origin_y,
  QString * error_message) const
{
  QFile file(yaml_path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    if (error_message != nullptr) {
      *error_message = "Failed to open YAML: " + yaml_path;
    }
    return false;
  }

  image_path.clear();
  resolution = 0.10;
  origin_x = 0.0;
  origin_y = 0.0;

  QTextStream in(&file);
  while (!in.atEnd()) {
    const QString line = in.readLine().trimmed();
    if (line.startsWith('#') || line.isEmpty()) {
      continue;
    }

    if (line.startsWith("image:")) {
      image_path = line.section(':', 1).trimmed();
      image_path.remove('"');
      image_path.remove('\'');
      continue;
    }

    if (line.startsWith("resolution:")) {
      bool ok = false;
      const double parsed = line.section(':', 1).trimmed().toDouble(&ok);
      if (ok && parsed > 0.0) {
        resolution = parsed;
      }
      continue;
    }

    if (line.startsWith("origin:")) {
      QString values = line.section(':', 1).trimmed();
      values.remove('[');
      values.remove(']');
      const QStringList parts = values.split(',', Qt::SkipEmptyParts);
      if (parts.size() >= 2) {
        bool ok_x = false;
        bool ok_y = false;
        const double parsed_x = parts[0].trimmed().toDouble(&ok_x);
        const double parsed_y = parts[1].trimmed().toDouble(&ok_y);
        if (ok_x && ok_y) {
          origin_x = parsed_x;
          origin_y = parsed_y;
        }
      }
    }
  }

  if (image_path.isEmpty()) {
    if (error_message != nullptr) {
      *error_message = "YAML missing image path: " + yaml_path;
    }
    return false;
  }

  QFileInfo image_info(image_path);
  if (image_info.isRelative()) {
    image_path = QFileInfo(yaml_path).dir().filePath(image_path);
  }

  return true;
}

bool MainWindow::load2DMapAsset(const QString & path, QString * error_message)
{
  const QFileInfo info(path);
  if (!info.exists()) {
    if (error_message != nullptr) {
      *error_message = "2D map file does not exist: " + path;
    }
    return false;
  }

  QString image_path = path;
  double resolution = 0.10;
  double origin_x = 0.0;
  double origin_y = 0.0;

  const QString suffix = info.suffix().toLower();
  if (suffix == "yaml" || suffix == "yml") {
    if (!loadMapMetadataFromYaml(path, image_path, resolution, origin_x, origin_y, error_message)) {
      return false;
    }
  } else if (suffix == "pgm") {
    const QString yaml_sidecar = info.dir().filePath(info.completeBaseName() + ".yaml");
    if (QFileInfo::exists(yaml_sidecar)) {
      QString yaml_image_path;
      double yaml_resolution = resolution;
      double yaml_origin_x = origin_x;
      double yaml_origin_y = origin_y;
      QString yaml_error;
      if (loadMapMetadataFromYaml(
          yaml_sidecar, yaml_image_path, yaml_resolution, yaml_origin_x, yaml_origin_y, &yaml_error)) {
        resolution = yaml_resolution;
        origin_x = yaml_origin_x;
        origin_y = yaml_origin_y;
      }
    }
  } else {
    if (error_message != nullptr) {
      *error_message = "Unsupported 2D map extension: " + suffix;
    }
    return false;
  }

  const QImage loaded(image_path);
  if (loaded.isNull()) {
    if (error_message != nullptr) {
      *error_message = "Failed to decode map image: " + image_path;
    }
    return false;
  }

  map2d_base_image_ = loaded.convertToFormat(QImage::Format_RGB32);
  current_map2d_path_ = path;
  map2d_resolution_m_ = resolution;
  map2d_origin_x_ = origin_x;
  map2d_origin_y_ = origin_y;
  return true;
}

bool MainWindow::loadTrajectoryCsv(const QString & csv_path, QString * error_message)
{
  QFile file(csv_path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    if (error_message != nullptr) {
      *error_message = "Failed to open trajectory CSV: " + csv_path;
    }
    return false;
  }

  std::vector<Eigen::Vector3f> points;
  QTextStream in(&file);
  bool first_line = true;
  while (!in.atEnd()) {
    const QString line = in.readLine().trimmed();
    if (line.isEmpty()) {
      continue;
    }

    if (first_line && line.contains("stamp_sec")) {
      first_line = false;
      continue;
    }
    first_line = false;

    const QStringList parts = line.split(',', Qt::SkipEmptyParts);
    if (parts.size() < 5) {
      continue;
    }

    bool ok_x = false;
    bool ok_y = false;
    bool ok_z = false;
    const float x = static_cast<float>(parts[2].trimmed().toDouble(&ok_x));
    const float y = static_cast<float>(parts[3].trimmed().toDouble(&ok_y));
    const float z = static_cast<float>(parts[4].trimmed().toDouble(&ok_z));
    if (!ok_x || !ok_y || !ok_z) {
      continue;
    }
    points.emplace_back(x, y, z);
  }

  if (points.empty()) {
    if (error_message != nullptr) {
      *error_message = "No valid trajectory points in CSV: " + csv_path;
    }
    return false;
  }

  trajectory_points_world_ = std::move(points);
  current_trajectory_path_ = csv_path;
  viewer_widget_->setTrajectoryPoints(trajectory_points_world_);
  return true;
}

void MainWindow::render2DMapWithTrajectory()
{
  if (map2d_label_ == nullptr) {
    return;
  }

  if (map2d_base_image_.isNull()) {
    map2d_label_->setPixmap(QPixmap());
    if (trajectory_points_world_.empty()) {
      map2d_label_->setText("No 2D map loaded.");
    } else {
      map2d_label_->setText("2D map not loaded.\nTrajectory is shown in 3D.");
    }
    return;
  }

  QImage canvas = map2d_base_image_.copy();
  QPainter painter(&canvas);
  painter.setRenderHint(QPainter::Antialiasing, true);

  if (!trajectory_points_world_.empty()) {
    QPolygon polyline;
    const double resolution = std::max(1e-6, map2d_resolution_m_);
    for (const auto & point : trajectory_points_world_) {
      const int col = static_cast<int>(std::llround((point.x() - map2d_origin_x_) / resolution));
      const int row = static_cast<int>(std::llround((point.y() - map2d_origin_y_) / resolution));
      const int img_row = canvas.height() - 1 - row;
      if (col < 0 || col >= canvas.width() || img_row < 0 || img_row >= canvas.height()) {
        continue;
      }
      polyline << QPoint(col, img_row);
    }

    if (polyline.size() >= 2) {
      QPen line_pen(QColor(255, 40, 40));
      line_pen.setWidth(2);
      painter.setPen(line_pen);
      painter.drawPolyline(polyline);
    }

    if (!polyline.isEmpty()) {
      painter.setPen(Qt::NoPen);
      painter.setBrush(QColor(40, 220, 60));
      painter.drawEllipse(polyline.first(), 4, 4);
      painter.setBrush(QColor(255, 200, 20));
      painter.drawEllipse(polyline.last(), 4, 4);
    }
  }
  painter.end();

  QPixmap pixmap = QPixmap::fromImage(canvas);
  QSize target_size(640, 480);
  if (map2d_scroll_area_ != nullptr && map2d_scroll_area_->viewport() != nullptr) {
    const QSize viewport_size = map2d_scroll_area_->viewport()->size();
    if (viewport_size.width() > 32 && viewport_size.height() > 32) {
      target_size = viewport_size;
    }
  }

  map2d_label_->setText(QString());
  map2d_label_->setPixmap(pixmap.scaled(target_size, Qt::KeepAspectRatio, Qt::SmoothTransformation));
  map2d_label_->setToolTip(
    "Map: " + (current_map2d_path_.isEmpty() ? "-" : current_map2d_path_) +
    "\nTrajectory: " + (current_trajectory_path_.isEmpty() ? "-" : current_trajectory_path_));
}

void MainWindow::loadRecentFiles()
{
  QSettings settings("pc_env_viewer", "pc_env_viewer");
  recent_files_ = settings.value("recentFiles").toStringList();
}

void MainWindow::saveRecentFiles() const
{
  QSettings settings("pc_env_viewer", "pc_env_viewer");
  settings.setValue("recentFiles", recent_files_);
}

void MainWindow::addRecentFile(const QString & path)
{
  recent_files_.removeAll(path);
  recent_files_.prepend(path);
  while (recent_files_.size() > kMaxRecentFiles) {
    recent_files_.removeLast();
  }
  saveRecentFiles();
  rebuildRecentFileMenu();
}

void MainWindow::rebuildRecentFileMenu()
{
  recent_files_menu_->clear();
  if (recent_files_.isEmpty()) {
    auto * empty = recent_files_menu_->addAction("(None)");
    empty->setEnabled(false);
    return;
  }

  for (const QString & file : recent_files_) {
    auto * action = recent_files_menu_->addAction(file);
    connect(action, &QAction::triggered, this, [this, file]() {
      startLoad(file);
    });
  }
}

QString MainWindow::defaultOpenDirectory() const
{
  const std::filesystem::path preferred("/tmp/vertical_mapper_exports");
  if (std::filesystem::exists(preferred) && std::filesystem::is_directory(preferred)) {
    return QString::fromStdString(preferred.string());
  }
  return QDir::homePath();
}

QString MainWindow::colorModeLabel(ColorMode mode)
{
  switch (mode) {
    case ColorMode::RGB:
      return "RGB";
    case ColorMode::Intensity:
      return "Intensity";
    case ColorMode::Height:
      return "Height";
    case ColorMode::AxisRainbow:
      return "Axis Rainbow";
    default:
      return "Unknown";
  }
}

}  // namespace pc_env_viewer
