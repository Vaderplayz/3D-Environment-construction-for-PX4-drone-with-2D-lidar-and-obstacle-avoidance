#include "app/ViewerWidget.h"

#include <QColor>
#include <QVBoxLayout>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>

namespace pc_env_viewer {

ViewerWidget::ViewerWidget(QWidget * parent)
: QWidget(parent), vtk_widget_(new QVTKOpenGLNativeWidget(this))
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(vtk_widget_);

  auto render_window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  vtk_widget_->setRenderWindow(render_window);
  renderer_.initialize(vtk_widget_->renderWindow());
}

bool ViewerWidget::displayCloud(const PreviewCloud & preview, ColorMode mode)
{
  return renderer_.display(preview, mode);
}

bool ViewerWidget::setColorMode(ColorMode mode)
{
  return renderer_.setColorMode(mode);
}

bool ViewerWidget::supportsColorMode(ColorMode mode) const
{
  return renderer_.supportsColorMode(mode);
}

void ViewerWidget::setAxisRainbowAxis(AxisColorAxis axis)
{
  renderer_.setAxisRainbowAxis(axis);
}

void ViewerWidget::setTrajectoryPoints(const std::vector<Eigen::Vector3f> & points)
{
  renderer_.setTrajectoryPoints(points);
}

void ViewerWidget::clearTrajectory()
{
  renderer_.clearTrajectory();
}

void ViewerWidget::setTrajectoryVisible(bool visible)
{
  renderer_.setTrajectoryVisible(visible);
}

void ViewerWidget::setPointSize(float point_size)
{
  renderer_.setPointSize(point_size);
}

void ViewerWidget::setBackgroundColor(const QColor & color)
{
  renderer_.setBackgroundColor(color.redF(), color.greenF(), color.blueF());
}

bool ViewerWidget::saveScreenshot(const QString & path) const
{
  return renderer_.saveScreenshot(path.toStdString());
}

void ViewerWidget::resetCamera()
{
  renderer_.resetCamera();
}

void ViewerWidget::autoFitCamera()
{
  renderer_.autoFitCamera();
}

}  // namespace pc_env_viewer
