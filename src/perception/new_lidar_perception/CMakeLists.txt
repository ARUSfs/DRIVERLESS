cmake_minimum_required(VERSION 3.0.2)
project(new_lidar_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  common_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

catkin_package(
  # INCLUDE_DIRS include
#  LIBRARIES lidar_cpp
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
)


add_executable(new_main src/new_main.cpp src/lidar_handle.cpp)
target_link_libraries(new_main ${catkin_LIBRARIES})

