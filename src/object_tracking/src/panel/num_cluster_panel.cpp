#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <rviz/panel.h>
#include <QVBoxLayout>
#include <QLabel>
#include <pluginlib/class_list_macros.h>
#include "num_cluster_panel.h"

namespace object_tracking
{

NumClusterPanel::NumClusterPanel(QWidget* parent)
  : rviz::Panel(parent), nh_()
{
  layout_ = new QVBoxLayout;
  label_ = new QLabel("当前聚类数: 0");
  layout_->addWidget(label_);
  setLayout(layout_);

  cluster_sub_ = nh_.subscribe("/num_cluster", 10, &NumClusterPanel::callback, this);  // Subscribe to topic
}

void NumClusterPanel::updateLabel(int num)
{
  label_->setText(QString("当前聚类数: %1").arg(num));
}

void NumClusterPanel::callback(const std_msgs::Int32::ConstPtr& msg)
{
  Q_EMIT updateLabel(msg->data);  // Emit signal to update the label
}

// Plugin export macro
PLUGINLIB_EXPORT_CLASS(NumClusterPanel, rviz::Panel)

} // namespace object_tracking
