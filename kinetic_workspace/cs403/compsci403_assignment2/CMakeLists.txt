cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#ROSBUILD_GENMSG()
#uncomment if you have defined services
ROSBUILD_GENSRV()

SET(libs roslib roscpp)

ROSBUILD_ADD_EXECUTABLE(assignment2 src/assignment2.cpp)
TARGET_LINK_LIBRARIES(assignment2 ${libs})

ROSBUILD_ADD_EXECUTABLE(kinect_intrinsics_service src/kinect_intrinsics_service)
TARGET_LINK_LIBRARIES(kinect_intrinsics_service ${libs})
