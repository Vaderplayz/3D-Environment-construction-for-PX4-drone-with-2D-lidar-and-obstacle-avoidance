#pragma once

#include "core/CloudAsset.h"
#include "core/CloudProcessor.h"
#include "core/CloudRenderer.h"

#include <QWidget>

class QVTKOpenGLNativeWidget;

namespace pc_env_viewer {

class ViewerWidget : public QWidget {
  Q_OBJECT

public:
  explicit ViewerWidget(QWidget * parent = nullptr);

  bool displayCloud(const PreviewCloud & preview, ColorMode mode);
  bool setColorMode(ColorMode mode);
  bool supportsColorMode(ColorMode mode) const;
  void setPointSize(float point_size);
  void setBackgroundColor(const QColor & color);
  bool saveScreenshot(const QString & path) const;
  void resetCamera();

private:
  QVTKOpenGLNativeWidget * vtk_widget_;
  CloudRenderer renderer_;
};

}  // namespace pc_env_viewer
