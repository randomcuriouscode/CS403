cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

ROSBUILD_INIT()

ROSBUILD_ADD_BOOST_DIRECTORIES()
include_directories(include)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_compile_options(-std=c++11)

ROSBUILD_GENMSG()
ROSBUILD_GENSRV()

SET(libs roslib roscpp)

ROSBUILD_ADD_EXECUTABLE(assignment4 src/assignment4.cpp)
TARGET_LINK_LIBRARIES(assignment4 ${libs})

ROSBUILD_ADD_EXECUTABLE(assignment4_tester src/assignment4_tester.cpp)
TARGET_LINK_LIBRARIES(assignment4_tester ${libs})

rosbuild_add_gtest(utest test/utest.cpp)

ROSBUILD_LINK_BOOST(assignment4 system)
