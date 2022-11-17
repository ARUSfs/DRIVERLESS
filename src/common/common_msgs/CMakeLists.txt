cmake_minimum_required(VERSION 3.0.2)
project(common_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Cone.msg
   Map.msg
   Trajectory.msg
   Controls.msg
   velState.msg
   Camera_dofs.msg
   Mission.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Spawn.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs
 )


 catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES common_msgs
#  CATKIN_DEPENDS genmsg geometry_msgs message_generation rospy sensor_msgs std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
 )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

