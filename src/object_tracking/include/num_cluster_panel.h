#ifndef OBJECT_TRACKING_NUM_CLUSTER_PANEL_H
#define OBJECT_TRACKING_NUM_CLUSTER_PANEL_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <rviz/panel.h>

#include <QLabel>
#include <QVBoxLayout>

namespace object_tracking
{

class NumClusterPanel : public rviz::Panel
{
  Q_OBJECT

public:
  NumClusterPanel(QWidget* parent = 0);

  // Called when this panel is loaded
  virtual void onInitialize() override;

  // Save and load settings (optional, can be empty if not needed)
  virtual void load(const rviz::Config& config) override;
  virtual void save(rviz::Config config) const override;

protected Q_SLOTS:
  void updateLabel(int num);
private:
  void callback(const std_msgs::Int32::ConstPtr& msg);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber cluster_sub_;

  QLabel* label_;
  QVBoxLayout* layout_;
};

} // namespace object_tracking

#endif // OBJECT_TRACKING_NUM_CLUSTER_PANEL_H
