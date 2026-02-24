#pragma once

#include "core/CloudAsset.h"
#include "core/CloudProcessor.h"
#include "core/CloudRenderer.h"

#include <QWidget>
#include <vector>

class QVTKOpenGLNativeWidget;

namespace pc_env_viewer {

class ViewerWidget : public QWidget {
  Q_OBJECT

public:
  explicit ViewerWidget(QWidget * parent = nullptr);

  bool displayCloud(const PreviewCloud & preview, ColorMode mode);
  bool setColorMode(ColorMode mode);
  bool supportsColorMode(ColorMode mode) const;
  void setAxisRainbowAxis(AxisColorAxis axis);
  void setTrajectoryPoints(const std::vector<Eigen::Vector3f> & points);
  void clearTrajectory();
  void setPointSize(float point_size);
  void setBackgroundColor(const QColor & color);
  bool saveScreenshot(const QString & path) const;
  void resetCamera();
  void autoFitCamera();

private:
  QVTKOpenGLNativeWidget * vtk_widget_;
  CloudRenderer renderer_;
};

}  // namespace pc_env_viewer
