//xugong5 problem:
//1 tracking miss
//2 tf_old_data and process die
//xugong6 solve tf_old_data problem , tf::TransformListener lr(ros::Duration(100));
// changshu_2 base on xugong6 ,change parameter  related with vehicle ,Lidar height

#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/impl/transforms.hpp"
// #include <pcl_ros/transforms.h>


#include <vector>
#include <iostream>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/buffer.h>
// #include <tf2/transform_datatypes.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <message_filters/subscriber.h>  
#include <message_filters/time_synchronizer.h>  
#include <message_filters/sync_policies/approximate_time.h>
#include <object_tracking/trackbox.h>

#include "imm_ukf_jpda.h"

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace message_filters;

int counta = 0;

ros::Publisher pub;

tf::TransformListener* tran;
// tf::TransformListener* tran2;

double yaw_gps;

double v_gps;

// 回调函数-- 话题 track_box
void  cloud_cb (const object_tracking::trackbox& input){
// void  cloud_cb (const object_tracking::trackboxConstPtr& track_box, const sensor_msgs::PointCloud2ConstPtr& velodyne_points){
  counta ++;
  cout << "Frame: "<<counta << "----------------------------------------"<< endl;
  
  // debug: 检查是否帧同步
  // double delta = fabs(track_box->header.stamp.toSec() - velodyne_points->header.stamp.toSec());
  // if (delta > 0.1) ROS_WARN("Time delta: %.3fs", delta);

  // const object_tracking::trackbox& input = *track_box;

  // convert local to global-------------------------
  double timestamp = input.header.stamp.toSec();  // 报错： error: cannot convert ‘const _stamp_type {aka const ros::Time}’ to ‘double’
  vector<vector<double>> egoPoints;
  getOriginPoints(timestamp, egoPoints,v_gps,yaw_gps);   // v_gps,yaw_gps都是从回调函数cloud_cb计算得来
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(egoPoints[0][0], egoPoints[0][1], 0.0) );
  tf::Quaternion q;
  ros::Time input_time = input.header.stamp;
  q.setRPY(0, 0, egoPoints[0][2]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, input_time, "velo_link", "global"));

//   tf::StampedTransform transform2;
  int box_num = input.box_num;

  vector<PointCloud<PointXYZ>> bBoxes;
  PointCloud<PointXYZ> oneBbox;
//  bBoxes.header = input.header;
  for(int box_i = 0;box_i < box_num; box_i++)
  {
      PointXYZ o;
      o.x = input.x1[3*box_i];
      o.y = input.x1[3*box_i + 1];
      o.z = input.x1[3*box_i + 2];
      oneBbox.push_back(o);
      o.x = input.x2[3*box_i];
      o.y = input.x2[3*box_i + 1];
      o.z = input.x2[3*box_i + 2];   
      oneBbox.push_back(o);
      o.x = input.x3[3*box_i + 0];
      o.y = input.x3[3*box_i + 1];
      o.z = input.x3[3*box_i + 2];   
      oneBbox.push_back(o);
      o.x = input.x4[3*box_i + 0];
      o.y = input.x4[3*box_i + 1];
      o.z = input.x4[3*box_i + 2];   
      oneBbox.push_back(o);
      o.x = input.y1[3*box_i + 0];
      o.y = input.y1[3*box_i + 1];
      o.z = input.y1[3*box_i + 2];   
      oneBbox.push_back(o);
      o.x = input.y2[3*box_i + 0];
      o.y = input.y2[3*box_i + 1];
      o.z = input.y2[3*box_i + 2];   
      oneBbox.push_back(o);
      o.x = input.y3[3*box_i + 0];
      o.y = input.y3[3*box_i + 1];
      o.z = input.y3[3*box_i + 2];   
      oneBbox.push_back(o);
      o.x = input.y4[3*box_i + 0];
      o.y = input.y4[3*box_i + 1];
      o.z = input.y4[3*box_i + 2];   
      oneBbox.push_back(o);
      bBoxes.push_back(oneBbox);
      oneBbox.clear();
  }
  

  PointCloud<PointXYZ> newBox;
  for(int i = 0; i < bBoxes.size(); i++ ){
    bBoxes[i].header.frame_id = "velo_link";
    
    // tran->waitForTransform("/global", "/velodyne", input_time, ros::Duration(10.0));
    tran->waitForTransform("global", "velo_link", input_time, ros::Duration(10.0));
    pcl_ros::transformPointCloud("global", bBoxes[i], newBox, *tran);
    bBoxes[i] = newBox;
  }
  //end converting----------------------------------------
  PointCloud<PointXYZ> targetPoints;
  vector<vector<double>> targetVandYaw;
  vector<int> trackManage;  // trackManage???  大量具有相关不确定性的跟踪对象需要有效地实施跟踪管理。跟踪管理的主要目的是动态限制虚假跟踪列表的数量（从而防止错误的数据关联），并在丢失检测的情况下保持对象跟踪
  vector<bool> isStaticVec;
  vector<bool> isVisVec;
  vector<PointCloud<PointXYZ>> visBBs;
  immUkfJpdaf(bBoxes, timestamp, targetPoints, targetVandYaw, trackManage, isStaticVec, isVisVec, visBBs);

//   cout << "size is "<<visBBs.size()<<endl;
  // cout << "x1:"<<visBBs[0][0].x<<"y1:"<<visBBs[0][0].y<<endl;
  // cout << "x2:"<<visBBs[0][1].x<<"y2:"<<visBBs[0][1].y<<endl;
  // cout << "x3:"<<visBBs[0][2].x<<"y3:"<<visBBs[0][2].y<<endl;
  // cout << "x4:"<<visBBs[0][3].x<<"y4:"<<visBBs[0][3].y<<endl;


  assert(targetPoints.size() == trackManage.size());
  assert(targetPoints.size()== targetVandYaw.size());

  

  // converting from global to ego tf for visualization
  // processing targetPoints
  PointCloud<PointXYZ> egoTFPoints;
  targetPoints.header.frame_id = "global";
  pcl_ros::transformPointCloud("velo_link", targetPoints, egoTFPoints, *tran);

  //processing visBBs
  PointCloud<PointXYZ> visEgoBB;
  for(int i = 0; i < visBBs.size(); i++){
    visBBs[i].header.frame_id = "global";
    pcl_ros::transformPointCloud("velo_link", visBBs[i], visEgoBB, *tran);
    
    visBBs[i] = visEgoBB;
  }
  //end converting to ego tf-------------------------



  // tracking arrows visualizing start---------------------------------------------
  // for(int i = 0; i < targetPoints.size(); i++){
  //   visualization_msgs::Marker arrowsG;
  //   arrowsG.lifetime = ros::Duration(0.1);
  //   if(trackManage[i] == 0 ) {
  //     continue;
  //   }
  //   if(isVisVec[i] == false ) {
  //     continue;
  //   }
  //   if(isStaticVec[i] == true){
  //     continue;
  //   }
  //   arrowsG.header.frame_id = "velo_link";
    
  //   arrowsG.header.stamp= ros::Time::now();
  //   arrowsG.ns = "arrows";
  //   arrowsG.action = visualization_msgs::Marker::ADD;
  //   arrowsG.type =  visualization_msgs::Marker::ARROW;
  //   // green  设置颜色
  //   arrowsG.color.g = 1.0f; // 绿色
  //   // arrowsG.color.r = 1.0f; // 红色
  //   arrowsG.color.a = 1.0;  
  //   arrowsG.id = i;
  //   geometry_msgs::Point p;
  //   // assert(targetPoints[i].size()==4);
  //   p.x = egoTFPoints[i].x;
  //   p.y = egoTFPoints[i].y;
  //   p.z = -1.73/2;
  //   double tv   = targetVandYaw[i][0];
  //   double tyaw = targetVandYaw[i][1];

  //   // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  //   arrowsG.pose.position.x = p.x;
  //   arrowsG.pose.position.y = p.y;
  //   arrowsG.pose.position.z = p.z;

  //   // convert from 3 angles to quartenion
  //   tf::Matrix3x3 obs_mat;
  //   obs_mat.setEulerYPR(tyaw, 0, 0); // yaw, pitch, roll
  //   tf::Quaternion q_tf;
  //   obs_mat.getRotation(q_tf);
  //   arrowsG.pose.orientation.x = q_tf.getX();
  //   arrowsG.pose.orientation.y = q_tf.getY();
  //   arrowsG.pose.orientation.z = q_tf.getZ();
  //   arrowsG.pose.orientation.w = q_tf.getW();

  //   // Set the scale of the arrowsG -- 1x1x1 here means 1m on a side
  //   arrowsG.scale.x = tv;
  //   arrowsG.scale.y = 0.1;
  //   arrowsG.scale.z = 0.1;

  //   vis_pub.publish(arrowsG);  // 发布箭头消息
  // }
}


// 回调函数-- 话题 /gps/odom
int test=0;
void  cloud_cb2 (const nav_msgs::Odometry msg)
{

  test++;

  double v_x = msg.twist.twist.linear.x;  // 线速度
  double v_y = msg.twist.twist.linear.y;
  
   yaw_gps = msg.pose.pose.orientation.z; // 偏航角

   v_gps = sqrt(v_x*v_x+v_y*v_y);
}


int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "obj_track"); // obj_track--节点
  ros::NodeHandle nh;
  // ros::Subscriber subImu = nh.subscribe ("imu_data", 10, imu_cb);
  tf::TransformListener lr(ros::Duration(100));         //(How long to store transform information)
  tran=&lr;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber box_sub = nh.subscribe ("track_box", 600, cloud_cb);   //订阅者  track_box -- 话题topic名
  ros::Subscriber sub2 = nh.subscribe ("/gps/odom", 1000, cloud_cb2);   //订阅者  /gps/odom -- 话题topic名

  // Spin
  ros::spin ();
}