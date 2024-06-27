cmake_minimum_required(VERSION 3.0.2)
project(steering_c)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  common_msgs
)

catkin_package(
      CATKIN_DEPENDS roscpp std_msgs common_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include
)

add_executable(main src/main.cpp src/steering.cpp)

target_link_libraries(main -lEposCmd)
