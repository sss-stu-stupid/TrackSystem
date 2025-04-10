#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_to_rosbag_node");
    ros::NodeHandle nh;

    // --- 参数配置 ---
    std::string pcd_dir = "/home/cyq/simulation_dataset/no_treelawn_filtered";  // PCD 文件目录
    std::string output_bag = "/home/cyq/catkin_wp/path/no_treelawn_filtered_256car.bag";       // 输出 bag 文件名
    std::string topic_name = "/simulation/velo/pointcloud";  // 话题名称（与 KITTI 一致）
    std::string frame_id = "velo_link";          // 坐标系名称

    // --- 读取 PCD 文件列表 ---
    std::vector<std::string> pcd_files;
    boost::filesystem::directory_iterator it(pcd_dir);
    for (; it != boost::filesystem::directory_iterator(); ++it) {
        if (it->path().extension() == ".pcd") {
            pcd_files.push_back(it->path().string());
        }
    }
    std::sort(pcd_files.begin(), pcd_files.end());  // 按文件名排序

    // --- 创建 ROS Bag 文件 ---
    rosbag::Bag bag;
    bag.open(output_bag, rosbag::bagmode::Write);

    // --- 遍历并转换 PCD 文件 ---
    ros::Time timestamp = ros::Time::now();  // 初始时间戳
    // tf::TransformBroadcaster br;             // TF 广播器实例
    for (const auto& pcd_file : pcd_files) {
        // 1. 加载 PCD 文件
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file, *cloud) == -1) {
            ROS_ERROR("Failed to load PCD file: %s", pcd_file.c_str());
            continue;
        }

        // 2. 转换为 ROS PointCloud2 消息
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = timestamp;  // 设置时间戳
        cloud_msg.header.frame_id = frame_id;  // 设置坐标系

        // 3. 写入 Bag 文件
        bag.write(topic_name, timestamp, cloud_msg);

        // 4. 创建并写入 TF 变换到 Bag
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = timestamp;
        transformStamped.header.frame_id = "base_link";      // 父坐标系
        transformStamped.child_frame_id = frame_id;         // 子坐标系（velo_link）
        transformStamped.transform.translation.x = 0.0;      // 平移
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0;         // 旋转（单位四元数）
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        tf2_msgs::TFMessage tf_msg;
        tf_msg.transforms.push_back(transformStamped);
        bag.write("/tf", timestamp, tf_msg);

        ROS_INFO("Sent TF transform from 'base_link' to '%s' at time %f.", frame_id.c_str(), timestamp.toSec());

        // 更新时间戳（例如：按固定频率 10Hz）
        timestamp += ros::Duration(0.1);
    }

    // --- 关闭 Bag 文件 ---
    bag.close();
    ROS_INFO("Conversion completed. Bag saved to: %s", output_bag.c_str());
    return 0;
}


// #include <ros/ros.h>
// #include <rosbag/bag.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <string>
// #include <vector>
// #include <boost/filesystem.hpp>
// #include <tf2_msgs/TFMessage.h>
// #include <geometry_msgs/TransformStamped.h>

// void processPCDFiles(const std::string& pcd_dir, const std::string& topic_name, 
//                      const std::string& frame_id, rosbag::Bag& bag, ros::Time& timestamp) {
//     std::vector<std::string> pcd_files;
//     boost::filesystem::directory_iterator it(pcd_dir);
//     for (; it != boost::filesystem::directory_iterator(); ++it) {
//         if (it->path().extension() == ".pcd") {
//             pcd_files.push_back(it->path().string());
//         }
//     }
//     std::sort(pcd_files.begin(), pcd_files.end());

//     for (const auto& pcd_file : pcd_files) {
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//         if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file, *cloud) == -1) {
//             ROS_ERROR("Failed to load PCD file: %s", pcd_file.c_str());
//             continue;
//         }

//         sensor_msgs::PointCloud2 cloud_msg;
//         pcl::toROSMsg(*cloud, cloud_msg);
//         cloud_msg.header.stamp = timestamp;
//         cloud_msg.header.frame_id = frame_id;

//         bag.write(topic_name, timestamp, cloud_msg);

//         geometry_msgs::TransformStamped transformStamped;
//         transformStamped.header.stamp = timestamp;
//         transformStamped.header.frame_id = "base_link";
//         transformStamped.child_frame_id = frame_id;
//         transformStamped.transform.translation.x = 0.0;
//         transformStamped.transform.translation.y = 0.0;
//         transformStamped.transform.translation.z = 0.0;
//         transformStamped.transform.rotation.x = 0.0;
//         transformStamped.transform.rotation.y = 0.0;
//         transformStamped.transform.rotation.z = 0.0;
//         transformStamped.transform.rotation.w = 1.0;

//         tf2_msgs::TFMessage tf_msg;
//         tf_msg.transforms.push_back(transformStamped);
//         bag.write("/tf", timestamp, tf_msg);

//         ROS_INFO("Sent TF transform from 'base_link' to '%s' at time %f.", frame_id.c_str(), timestamp.toSec());

//         timestamp += ros::Duration(0.1);
//     }
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "pcd_to_rosbag_node");
//     ros::NodeHandle nh;

//     std::string pcd_dir_1 = "/home/cyq/simulation_dataset/scene2_filtered";  // fittered
//     std::string pcd_dir_2 = "/home/cyq/simulation_dataset/scene2_ori";
//     std::string output_bag = "/home/cyq/catkin_wp/path/simulation_scene2.bag";

//     std::string topic_1 = "/simulation/velo/pointcloud";
//     std::string topic_2 = "/simulation/velo_ori/pointcloud";
//     std::string frame_1 = "velo_link";
//     std::string frame_2 = "velo_link";

//     rosbag::Bag bag;
//     bag.open(output_bag, rosbag::bagmode::Write);

//     ros::Time timestamp = ros::Time::now();

//     processPCDFiles(pcd_dir_1, topic_1, frame_1, bag, timestamp);
//     processPCDFiles(pcd_dir_2, topic_2, frame_2, bag, timestamp);

//     bag.close();
//     ROS_INFO("Conversion completed. Bag saved to: %s", output_bag.c_str());
//     return 0;
// }
