#!/bin/bash

# 项目自动运行脚本
# 使用方法：./run_project.sh

# 进入catkin工作空间目录（根据实际情况修改路径）
CATKIN_WS="/home/cyq/catkin_wp"
echo "[1/5] 进入工作目录: $CATKIN_WS"
cd $CATKIN_WS

# 编译项目
echo "[2/5] 编译项目..."
source /opt/ros/noetic/setup.bash  # 根据ROS版本修改
catkin_make
source devel/setup.bash

# 启动roscore
echo "[3/5] 启动roscore..."
new_terminal "ROS Core" "source $CATKIN_WS/devel/setup.bash && roscore"

# 播放数据集
BAG_PATH="/home/cyq/catkin_wp/path/simulation_l.bag"  # 修改为实际路径
echo "[4/5] 播放数据集: $BAG_PATH"
new_terminal "Rosbag Play" "source $CATKIN_WS/devel/setup.bash && rosbag play $BAG_PATH --loop"

# 启动算法节点
echo "[5/5] 启动算法节点..."
new_terminal "Launch File" "source $CATKIN_WS/devel/setup.bash && roslaunch object_tracking test.launch"

echo "所有组件已启动！"