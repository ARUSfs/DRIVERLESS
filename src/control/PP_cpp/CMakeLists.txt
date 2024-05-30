cmake_minimum_required(VERSION 3.0.2)
project(PP_cpp)


find_package(catkin REQUIRED COMPONENTS
    common_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

catkin_package( DEPENDS common_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        common_msgs
        std_msgs
)


add_executable(main src/main.cpp src/PurePursuit.cpp src/ControlHandle.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${tf2_ros_LIBRARIES})

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  "./include"
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
