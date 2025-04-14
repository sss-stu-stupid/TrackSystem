cmake_minimum_required(VERSION 2.8.3)
project(object_tracking)

# set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(Qt5_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs 
  std_msgs
  rviz
)

# include Qt headers
include_directories(${Qt5Widgets_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


add_message_files(
  FILES
  Obstacle.msg
  ObstacleList.msg
  Road_extract.msg
  trackbox.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_generation std_msgs message_runtime sensor_msgs rviz
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}  # RViz 使用 OGRE 进行渲染
)


# Qt 插件部分
set(PANEL_SOURCES src/panel/num_cluster_panel.cpp)
add_library(num_cluster_panel ${PANEL_SOURCES})
target_link_libraries(num_cluster_panel
  ${catkin_LIBRARIES}
  Qt5::Widgets      # 加上 Qt5::Widgets 的链接
)
# 安装插件库
install(TARGETS num_cluster_panel
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


# 让 CMake 知道使用 MOC（元对象编译器）
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(SOURCE_FILES1 src/groundremove/main.cpp src/groundremove/gaus_blur.cpp src/groundremove/ground_removal.cpp )
add_executable(ground ${SOURCE_FILES1})
target_link_libraries(ground ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#src/groundremove/gaus_blur.h src/groundremove/ground_removal.h src/cluster/component_clustering.h  src/cluster/box_fitting.h

set(SOURCE_FILES2 src/cluster/main.cpp src/cluster/component_clustering.cpp src/cluster/box_fitting.cpp )
add_executable(cluster ${SOURCE_FILES2})
target_link_libraries(cluster ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

set(SOURCE_FILES3 tracking/main.cpp tracking/imm_ukf_jpda.cpp tracking/ukf.cpp )
add_executable(tracking ${SOURCE_FILES3})
target_link_libraries(tracking ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


