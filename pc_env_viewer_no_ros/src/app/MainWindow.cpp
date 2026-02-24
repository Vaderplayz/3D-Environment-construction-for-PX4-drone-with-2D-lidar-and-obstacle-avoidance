#include "app/MainWindow.h"

#include "core/CloudLoader.h"

#include <QCloseEvent>
#include <QColorDialog>
#include <QComboBox>
#include <QDesktopServices>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QDir>
#include <QDragEnterEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMimeData>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSettings>
#include <QStatusBar>
#include <QTimer>
#include <QToolBar>
#include <QUrl>

#include <QtConcurrent/QtConcurrent>

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
  screenshot_action_ = file_menu->addAction("Save Screenshot...");
  export_preview_action_ = file_menu->addAction("Export Preview as PCD...");
  recent_files_menu_ = file_menu->addMenu("Recent Files");
  file_menu->addSeparator();
  file_menu->addAction("Quit", this, &QWidget::close);

  auto * view_menu = menuBar()->addMenu("&View");
  reset_camera_action_ = view_menu->addAction("Reset Camera");

  auto * toolbar = addToolBar("Main");
  toolbar->addAction(open_action_);
  toolbar->addAction(reload_action_);
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
  layout->addRow("Point size", point_size_spin_);
  layout->addRow("Background", background_button);

  dock_content->setLayout(layout);
  dock->setWidget(dock_content);
  addDockWidget(Qt::RightDockWidgetArea, dock);

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
  connect(screenshot_action_, &QAction::triggered, this, &MainWindow::saveScreenshot);
  connect(export_preview_action_, &QAction::triggered, this, &MainWindow::exportPreviewAsPcd);
  connect(reset_camera_action_, &QAction::triggered, this, &MainWindow::resetCamera);
  connect(background_button, &QPushButton::clicked, this, &MainWindow::pickBackgroundColor);
  connect(color_mode_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::onColorModeChanged);
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
  updateStatsPanel(*current_cloud_);
  updateColorModeControls(*current_cloud_);
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
  screenshot_action_->setEnabled(!busy && current_cloud_.has_value());
  export_preview_action_->setEnabled(!busy && current_cloud_.has_value());
  reset_camera_action_->setEnabled(!busy && current_cloud_.has_value());
  color_mode_combo_->setEnabled(!busy && current_cloud_.has_value());
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

  for (int i = 0; i < color_mode_combo_->count(); ++i) {
    if (color_mode_combo_->itemData(i).toInt() == static_cast<int>(cloud.default_color_mode)) {
      color_mode_combo_->setCurrentIndex(i);
      break;
    }
  }

  color_mode_combo_->setEnabled(true);
  color_mode_combo_->blockSignals(false);
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
    default:
      return "Unknown";
  }
}

}  // namespace pc_env_viewer
