cmake_minimum_required(VERSION 3.0.2)
project(fssim_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
        DEPENDS common_msgs, fssim_common

)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fssim_interface
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


