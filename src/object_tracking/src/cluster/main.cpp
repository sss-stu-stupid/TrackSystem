//base on pingjia3,code adjust

#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/impl/transforms.hpp"

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
#include <utility>
#include <nav_msgs/OccupancyGrid.h>


#include <object_tracking/trackbox.h>  // 没有这个文件？

#include "component_clustering.h"
#include "box_fitting.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

int counta = 0;

ros::Publisher pub;

ros::Publisher vis_pub;

ros::Publisher g_costmap_pub;

ros::Publisher obs_pub;

ros::Publisher marker_array_pub_;

ros::Publisher box_pub;

// 回调函数
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){  // f非地面数据

  // 1. 点云数据预处理
  PointCloud<pcl::PointXYZ>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  fromROSMsg (*input, *raw_cloud);

  int numCluster = 0; // global variable  聚类ID数量？
  array<array<int, numGrid>, numGrid> cartesianData{};  // 笛卡尔坐标数据，二维网格？

  // debug, 检查输入
  // if (raw_cloud && !raw_cloud->empty()) {
  //   ROS_INFO_STREAM("Raw Cloud Info:"
  //       << "\nFrame ID: " << raw_cloud->header.frame_id
  //       << "\nPoints Count: " << raw_cloud->size()
  //       << "\nWidth x Height: " << raw_cloud->width << " x " << raw_cloud->height
  //   );
  // } else {
  //     ROS_WARN("Raw cloud is empty!");
  // }
  // if (!cartesianData.empty()) {
  //   ROS_INFO_STREAM("Cartesian Grid Dimensions: " << numGrid << "x" << numGrid);
  // }

  // 2. 聚类处理
  componentClustering(raw_cloud, cartesianData, numCluster);  // Source: /src/cluster/component_clustering.cpp
  cout << "初始聚类ID数量numCluster is "<<numCluster<<endl; // 聚类的数量

  PointCloud<pcl::PointXYZ>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  makeClusteredCloud(raw_cloud, cartesianData, clusteredCloud);  // 聚类      Source: /src/cluster/component_clustering.cpp

  // Convert from PCL::PointCloud to ROS data type
  clusteredCloud->header.frame_id = raw_cloud->header.frame_id; // add "velo_link"

  static int count = 0;
  static nav_msgs::OccupancyGrid og;   // 
  if (!count)
    setOccupancyGrid(&og);   // 设置参数数值

  og.header.frame_id = raw_cloud->header.frame_id;

  // create cost map with pointcloud    costmap代码地图 简单来说就是为了在这张地图上进行各种加工，方便我们后面进行路径规划而存在的。
  std::vector<int> cost_map = createCostMap(*raw_cloud); //  Source: /src/cluster/component_clustering.cpp


  og.data.insert(og.data.end(), cost_map.begin(), cost_map.end());
  g_costmap_pub.publish(og);  // 发布者
  og.data.clear();
  count++;
  
  counta ++;
  cout << "cluster Frame: "<<counta << "----------------------------------------"<< endl;   // 帧数

  visualization_msgs::MarkerArray ma;  //实体框

  // 3. 边界框拟合
  vector<PointCloud<PointXYZ>> bBoxes = boxFitting(raw_cloud, cartesianData, numCluster,ma);  // bBoxes---- 实体边界框集合 多少个边界框  初始聚类ID数量numCluster

  object_tracking::trackbox boxArray; // boxArray--候选框8个坐标数组 的 数组  msg格式：object_tracking/msg/trackbox.msg
    
  boxArray.header = input->header;
  boxArray.box_num = bBoxes.size();
  // 填充boxArray
  for(int i = 0;i < bBoxes.size();i++)
  {
    boxArray.x1.push_back(bBoxes[i][0].x);
    boxArray.x1.push_back(bBoxes[i][0].y);
    boxArray.x1.push_back(bBoxes[i][0].z);

    boxArray.x2.push_back(bBoxes[i][1].x);
    boxArray.x2.push_back(bBoxes[i][1].y);
    boxArray.x2.push_back(bBoxes[i][1].z);

    boxArray.x3.push_back(bBoxes[i][2].x);
    boxArray.x3.push_back(bBoxes[i][2].y);
    boxArray.x3.push_back(bBoxes[i][2].z);

    boxArray.x4.push_back(bBoxes[i][3].x);
    boxArray.x4.push_back(bBoxes[i][3].y);
    boxArray.x4.push_back(bBoxes[i][3].z);

    boxArray.y1.push_back(bBoxes[i][4].x);
    boxArray.y1.push_back(bBoxes[i][4].y);
    boxArray.y1.push_back(bBoxes[i][4].z);

    boxArray.y2.push_back(bBoxes[i][5].x);
    boxArray.y2.push_back(bBoxes[i][5].y);
    boxArray.y2.push_back(bBoxes[i][5].z);

    boxArray.y3.push_back(bBoxes[i][6].x);
    boxArray.y3.push_back(bBoxes[i][6].y);
    boxArray.y3.push_back(bBoxes[i][6].z);

    boxArray.y4.push_back(bBoxes[i][7].x);
    boxArray.y4.push_back(bBoxes[i][7].y);
    boxArray.y4.push_back(bBoxes[i][7].z);
  }

//************************************cube visualiaztion 立方体可视化******************************
  box_pub.publish(boxArray);   //  发布者  boxArray--候选框8个坐标数组 的 数组

  // cout << "boxArray is " << boxArray<< endl;  // bBoxes
  cout << "size of bBoxes is " << bBoxes.size() << endl;  //bBoxes边界框的数量 size of bBoxes is 2
  cout << "size of marker is " << ma.markers.size() << endl; // marker数量 size of marker is 2
  
  marker_array_pub_.publish(ma);   // 发布者


//*********************************************bBoxes visualization***************************************

  //methd3: 只用ID文本
  // visualization_msgs::MarkerArray text_markers;

  // for(int objectI = 0; objectI < bBoxes.size(); objectI++) {
  //     visualization_msgs::Marker text_marker;
  //     text_marker.header.frame_id = "velo_link";
  //     text_marker.header.stamp = ros::Time::now();
  //     text_marker.ns = "object_ids";
  //     text_marker.id = objectI;
  //     text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  //     text_marker.action = visualization_msgs::Marker::ADD;
      
  //     // 设置文本位置（使用边界框的中心点）
  //     text_marker.pose.position.x = (bBoxes[objectI][0].x + bBoxes[objectI][6].x) / 2;
  //     text_marker.pose.position.y = (bBoxes[objectI][0].y + bBoxes[objectI][6].y) / 2;
  //     text_marker.pose.position.z = (bBoxes[objectI][0].z + bBoxes[objectI][6].z) / 2;
      
  //     // 设置文本内容
  //     text_marker.text = "ID:" + std::to_string(objectI);
      
  //     // 设置文本样式
  //     text_marker.scale.z = 2.5;  // 文字大小
  //     text_marker.color.r = 0.0f; // 红色
  //     text_marker.color.g = 1.0f;
  //     text_marker.color.b = 0.0f;
  //     text_marker.color.a = 1.0f; // 不透明
      
  //     text_marker.lifetime = ros::Duration(0.1);
  //     text_markers.markers.push_back(text_marker);
  // }

  // // 发布文本标记数组
  // marker_array_pub_.publish(text_markers);

  // method 2: 用cube
  // visualization_msgs::MarkerArray simplified_markers;

  // for(int objectI = 0; objectI < bBoxes.size(); objectI++) {
  //     visualization_msgs::Marker box;
  //     box.header.frame_id = "velo_link";
  //     box.header.stamp = ros::Time::now();
  //     box.ns = "simplified_boxes";
  //     box.id = objectI;
  //     box.type = visualization_msgs::Marker::CUBE;
  //     box.action = visualization_msgs::Marker::ADD;
      
  //     // 计算边界框的中心和尺寸
  //     box.pose.position.x = (bBoxes[objectI][0].x + bBoxes[objectI][6].x) / 2;
  //     box.pose.position.y = (bBoxes[objectI][0].y + bBoxes[objectI][6].y) / 2;
  //     box.pose.position.z = (bBoxes[objectI][0].z + bBoxes[objectI][6].z) / 2;
      
  //     box.scale.x = fabs(bBoxes[objectI][0].x - bBoxes[objectI][1].x);
  //     box.scale.y = fabs(bBoxes[objectI][0].y - bBoxes[objectI][3].y);
  //     box.scale.z = fabs(bBoxes[objectI][0].z - bBoxes[objectI][4].z);
      
  //     // 设置颜色和透明度
  //     box.color.r = 0.0f;
  //     box.color.g = 1.0f;
  //     box.color.b = 0.0f;
  //     box.color.a = 0.5f;
      
  //     box.lifetime = ros::Duration(0.1);
  //     simplified_markers.markers.push_back(box);
  // }

  // marker_array_pub_.publish(simplified_markers);

  // method 1: 用边界框
  visualization_msgs::Marker line_list; //将候选框8个点连线
  // line_list.lifetime = ros::Duration(0.1);
  line_list.header.frame_id = "velo_link";   // 定义frame_id (rviz需要设置世界坐标系为velodyne)
  line_list.header.stamp = ros::Time::now();
  line_list.ns =  "boxes";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST; //线条序列

  //LINE_LIST markers use only the x component of scale, for the line width  仅将比例的x分量用于线宽
  line_list.scale.x = 0.2;
  // Points are green
  line_list.color.g = 1.0f;
  line_list.color.a = 1.0;

  int id = 0;string ids;
  for(int objectI = 0; objectI < bBoxes.size(); objectI ++){  // 多少个边界框,循环几次
    for(int pointI = 0; pointI < 4; pointI++){ //内循环4次??
      assert((pointI+1)%4 < bBoxes[objectI].size());
      assert((pointI+4) < bBoxes[objectI].size());
      assert((pointI+1)%4+4 < bBoxes[objectI].size());
      id ++; ids = to_string(id);
      geometry_msgs::Point p;  // 定义p
      p.x = bBoxes[objectI][pointI].x;
      p.y = bBoxes[objectI][pointI].y;
      p.z = bBoxes[objectI][pointI].z;
      line_list.points.push_back(p);  // 给line_lists添加点!!!!
      p.x = bBoxes[objectI][(pointI+1)%4].x;  // 取余4
      p.y = bBoxes[objectI][(pointI+1)%4].y;
      p.z = bBoxes[objectI][(pointI+1)%4].z;
      line_list.points.push_back(p);

      p.x = bBoxes[objectI][pointI].x;
      p.y = bBoxes[objectI][pointI].y;
      p.z = bBoxes[objectI][pointI].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][pointI+4].x;// 加4?
      p.y = bBoxes[objectI][pointI+4].y;
      p.z = bBoxes[objectI][pointI+4].z;
      line_list.points.push_back(p);

      p.x = bBoxes[objectI][pointI+4].x;
      p.y = bBoxes[objectI][pointI+4].y;
      p.z = bBoxes[objectI][pointI+4].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][(pointI+1)%4+4].x;
      p.y = bBoxes[objectI][(pointI+1)%4+4].y;
      p.z = bBoxes[objectI][(pointI+1)%4+4].z;
      line_list.points.push_back(p);
    }
  }

  //line list end
  vis_pub.publish(line_list);   //发布者  cluster_obs -- 对应话题名topic  visualization_marker
  // bounding box visualizing end---------------------------------------------
}


int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "cluster");  // "cluster"--节点名
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/raw_pointcloud", 160, cloud_cb);  //订阅者  none_ground_topic -- 话题topic名

  // Create a ROS publisher for the output point cloud
  vis_pub = nh.advertise<visualization_msgs::Marker>( "/visualization_marker", 500);  //发布者  visualization_marker -- 话题topic名
  marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/cluster_ma", 500);   //发布者  cluster_ma -- 话题topic名
  g_costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("realtime_cost_map", 100);    //全局代价地图？？ 发布者  realtime_cost_map -- 话题topic名
  box_pub = nh.advertise<object_tracking::trackbox>("track_box",500);   //发布者  track_box -- 话题topic名
  // Spin
  ros::spin ();
}