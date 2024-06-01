cmake_minimum_required(VERSION 3.0.2)
project(can_c)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  common_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs common_msgs
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include # Asegúrate de que la ruta a las cabeceras de canlib es correcta
)

## Declare the C++ executable
add_executable(can_c_node src/main.cpp src/can_handle.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(can_c_node
  ${catkin_LIBRARIES}
  /usr/lib/libcanlib.so # Asegúrate de que la ruta a la biblioteca canlib es correcta
)
