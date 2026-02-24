#pragma once

#include "app/ViewerWidget.h"
#include "core/CliOptions.h"
#include "core/CloudProcessor.h"
#include "core/Result.h"

#include <QFutureWatcher>
#include <QMainWindow>
#include <QPointer>
#include <QProgressBar>

#include <atomic>
#include <memory>
#include <optional>

class QAction;
class QLabel;
class QComboBox;
class QMenu;
class QPlainTextEdit;
class QDoubleSpinBox;

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
  void saveScreenshot();
  void exportPreviewAsPcd();
  void resetCamera();
  void pickBackgroundColor();
  void onColorModeChanged(int index);
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
  QAction * reset_camera_action_{nullptr};
  QAction * screenshot_action_{nullptr};
  QAction * export_preview_action_{nullptr};
  QMenu * recent_files_menu_{nullptr};

  QLabel * raw_points_label_{nullptr};
  QLabel * preview_points_label_{nullptr};
  QLabel * bounds_label_{nullptr};
  QPlainTextEdit * fields_text_{nullptr};
  QComboBox * color_mode_combo_{nullptr};
  QDoubleSpinBox * point_size_spin_{nullptr};

  QFutureWatcher<PipelineResult> load_watcher_;
  std::shared_ptr<std::atomic_bool> cancel_token_;

  QString current_file_path_;
  QStringList recent_files_;
  std::optional<ProcessedCloud> current_cloud_;
};

}  // namespace pc_env_viewer
