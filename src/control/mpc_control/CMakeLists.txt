
cmake_minimum_required(VERSION 2.8.3)
project(mpc_control)

find_package(catkin_simple REQUIRED)

catkin_simple()

# activate c++ 11
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

# ARM NEON flags
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  message("enabling ARM neon optimizations")
endif()

# flags for speed (should already be enabled by default)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3")

catkin_package(
        DEPENDS common_msgs, fssim_common
)

cs_add_library(utrilla_solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/prueba_kin_export/acado_qpoases_interface.cpp
  model/prueba_kin_export/acado_integrator.c
  model/prueba_kin_export/acado_solver.c
  model/prueba_kin_export/acado_auxiliary_functions.c)

target_include_directories(utrilla_solver PUBLIC
  model/prueba_kin_export/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC)

cs_add_library(utrilla_wrapper
  src/utrilla_wrapper.cpp)

target_link_libraries(utrilla_wrapper
  utrilla_solver)

cs_add_executable(utrilla_node
  src/utrilla_node.cpp)
target_link_libraries(utrilla_node
  utrilla_wrapper
  utrilla_solver)


# make an executable
cs_install()
cs_export()
  

