cmake_minimum_required(VERSION 3.0.2)
project(syn_py)

## 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  message_filters
  tf2_ros
  cv_bridge
)

## 声明Catkin包元数据
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs message_filters tf2_ros cv_bridge
)

## 安装Python节点到可执行路径
catkin_install_python(
  PROGRAMS
    scripts/fliter_node  # 确保路径与实际文件位置一致
    scripts/projection_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 包含头文件目录（若需C++代码）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)