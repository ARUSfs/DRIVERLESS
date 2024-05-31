cmake_minimum_required(VERSION 3.0.2)
project(can_c)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  common_msgs
)

# find_library(CANLIB_LIBRARY NAMES canlib PATHS /usr/lib)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

catkin_package(

  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  # ${CANLIB_LIBRARY}

)

add_executable(main src/main.cpp src/can_handle.cpp)
target_link_libraries(main ${catkin_LIBRARIES})

