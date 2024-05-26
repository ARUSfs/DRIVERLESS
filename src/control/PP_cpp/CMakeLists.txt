cmake_minimum_required(VERSION 3.0.2)
project(PP_cpp)


find_package(catkin REQUIRED COMPONENTS
)

find_package(PCL REQUIRED)

catkin_package(
        DEPENDS common_msgs
)

add_executable(main src/main.cpp src/PurePursuit.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${PCL_LIBRARIES})
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(PCL_DEFINITIONS="${PCL_DEFINITIONS}")
