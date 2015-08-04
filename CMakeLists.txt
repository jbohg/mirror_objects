cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)

include_directories(
  include/mirror_objects 
  include/mirror_objects/poisson
  ${OpenCV_INCLUDE_DIRS}
  )


set(cc_sources 
  src/quat.cpp 
  src/predictObject.cc 
  src/parse.cpp 
  src/utilities.cc)
FILE(GLOB poisson_sources src/poisson/*.cpp)

#common commands for building c++ executables and libraries
rosbuild_add_library(poisson ${poisson_sources})
set_target_properties(poisson PROPERTIES COMPILE_FLAGS " -g -Wno-deprecated")
rosbuild_add_library(${PROJECT_NAME} ${cc_sources})
target_link_libraries(${PROJECT_NAME} poisson ${OpenCV_LIBS})

rosbuild_add_executable(mirrorAndReconstruct src/run_mirror_and_meshing.cpp)
target_link_libraries(mirrorAndReconstruct ${PROJECT_NAME} ${OpenCV_LIBS})

rosbuild_add_executable(mirrorCloud src/run_mirroring.cpp)
target_link_libraries(mirrorCloud ${PROJECT_NAME} ${OpenCV_LIBS})

rosbuild_add_executable(meshCloud src/run_meshing.cpp)
target_link_libraries(meshCloud ${PROJECT_NAME} ${OpenCV_LIBS})
