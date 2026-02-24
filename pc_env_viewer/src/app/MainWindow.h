#pragma once

#include "app/ViewerWidget.h"
#include "core/CliOptions.h"
#include "core/CloudProcessor.h"
#include "core/Result.h"

#include <QFutureWatcher>
#include <QImage>
#include <QMainWindow>
#include <QPointer>
#include <QProgressBar>

#include <atomic>
#include <memory>
#include <optional>
#include <vector>

class QAction;
class QLabel;
class QComboBox;
class QMenu;
class QPlainTextEdit;
class QDoubleSpinBox;
class QScrollArea;

namespace pc_env_viewer {

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(const CliOptions & cli_options, QWidget * parent = nullptr);
  ~MainWindow() override;

signals:
  void requestLoad(const QString & path);
  void loadStarted();
  void loadProgress(const QString & phase);
  void loadFinished(const QString & path);
  void loadFailed(const QString & message);

protected:
  void dragEnterEvent(QDragEnterEvent * event) override;
  void dropEvent(QDropEvent * event) override;
  void closeEvent(QCloseEvent * event) override;

private slots:
  void openFileDialog();
  void reloadCurrentFile();
  void open2DMapDialog();
  void openTrajectoryDialog();
  void saveScreenshot();
  void exportPreviewAsPcd();
  void resetCamera();
  void autoFitCamera();
  void pickBackgroundColor();
  void onColorModeChanged(int index);
  void onRainbowAxisChanged(int index);
  void onPointSizeChanged(double value);
  void onLoadWatcherFinished();
  void onLoadProgressMessage(const QString & phase);

private:
  struct PipelineResult {
    bool ok{false};
    std::optional<ProcessedCloud> processed;
    QString error_message;
  };

  static PipelineResult runLoadPipeline(
    const QString & path,
    std::size_t max_preview_points,
    const std::shared_ptr<std::atomic_bool> & cancel_token,
    const QPointer<MainWindow> & receiver);

  void startLoad(const QString & path);
  void setBusy(bool busy);
  void updateStatsPanel(const ProcessedCloud & cloud);
  void updateColorModeControls(const ProcessedCloud & cloud);
  void tryLoadAssociatedMapAssets(const QString & cloud_path);
  bool load2DMapAsset(const QString & path, QString * error_message = nullptr);
  bool loadMapMetadataFromYaml(
    const QString & yaml_path,
    QString & image_path,
    double & resolution,
    double & origin_x,
    double & origin_y,
    QString * error_message = nullptr) const;
  bool loadTrajectoryCsv(const QString & csv_path, QString * error_message = nullptr);
  void render2DMapWithTrajectory();
  void loadRecentFiles();
  void saveRecentFiles() const;
  void addRecentFile(const QString & path);
  void rebuildRecentFileMenu();
  QString defaultOpenDirectory() const;
  static QString colorModeLabel(ColorMode mode);

  CliOptions options_;

  ViewerWidget * viewer_widget_{nullptr};
  QProgressBar * progress_bar_{nullptr};

  QAction * open_action_{nullptr};
  QAction * reload_action_{nullptr};
  QAction * open_2d_map_action_{nullptr};
  QAction * open_trajectory_action_{nullptr};
  QAction * reset_camera_action_{nullptr};
  QAction * auto_fit_action_{nullptr};
  QAction * screenshot_action_{nullptr};
  QAction * export_preview_action_{nullptr};
  QMenu * recent_files_menu_{nullptr};

  QLabel * raw_points_label_{nullptr};
  QLabel * preview_points_label_{nullptr};
  QLabel * bounds_label_{nullptr};
  QPlainTextEdit * fields_text_{nullptr};
  QLabel * map2d_label_{nullptr};
  QScrollArea * map2d_scroll_area_{nullptr};
  QComboBox * color_mode_combo_{nullptr};
  QComboBox * rainbow_axis_combo_{nullptr};
  QDoubleSpinBox * point_size_spin_{nullptr};

  QFutureWatcher<PipelineResult> load_watcher_;
  std::shared_ptr<std::atomic_bool> cancel_token_;

  QString current_file_path_;
  QString current_map2d_path_;
  QString current_trajectory_path_;
  QImage map2d_base_image_;
  double map2d_resolution_m_{0.10};
  double map2d_origin_x_{0.0};
  double map2d_origin_y_{0.0};
  std::vector<Eigen::Vector3f> trajectory_points_world_;
  QStringList recent_files_;
  std::optional<ProcessedCloud> current_cloud_;
};

}  // namespace pc_env_viewer
