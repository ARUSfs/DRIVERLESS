cmake_minimum_required(VERSION 3.0.2)
project(car_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
        DEPENDS common_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
