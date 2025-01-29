cmake_minimum_required(VERSION 3.0.2)
project(lidar_perception)

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


add_executable(main src/main.cpp src/lidar_handle.cpp)
add_executable(limovelo_interface src/limovelo_interface.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
target_link_libraries(limovelo_interface ${catkin_LIBRARIES})

