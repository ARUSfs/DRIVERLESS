cmake_minimum_required(VERSION 3.0.2)
project(PP_cpp)


find_package(catkin REQUIRED COMPONENTS)

find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

catkin_package( DEPENDS common_msgs
        tf2_ros
)


add_executable(main src/main.cpp src/PurePursuit.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${tf2_ros_LIBRARIES})

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
