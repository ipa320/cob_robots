cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_find_ros_package(actionlib_msgs)
#include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
#genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# rostest for cob3-1
rosbuild_add_roslaunch_check(robots/cob3-1.xml ROBOT=cob3-1 ROBOT_ENV=ipa-apartment)
rosbuild_add_roslaunch_check(tools ROBOT=cob3-1 ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(tools ROBOT=cob3-1 ROBOT_ENV=ipa-apartment)

# rostest for cob3-2
rosbuild_add_roslaunch_check(robots/cob3-2.xml ROBOT=cob3-2 ROBOT_ENV=ipa-apartment)
rosbuild_add_roslaunch_check(tools ROBOT=cob3-2 ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(tools ROBOT=cob3-2 ROBOT_ENV=ipa-apartment)

# rostest for cob3-3
rosbuild_add_roslaunch_check(robots/cob3-3.xml ROBOT=cob3-3 ROBOT_ENV=ipa-apartment)
rosbuild_add_roslaunch_check(tools ROBOT=cob3-3 ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(tools ROBOT=cob3-3 ROBOT_ENV=ipa-apartment)

# rostest for cob3-4
rosbuild_add_roslaunch_check(robots/cob3-4.xml ROBOT=cob3-4 ROBOT_ENV=ipa-apartment)
rosbuild_add_roslaunch_check(tools ROBOT=cob3-4 ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(tools ROBOT=cob3-4 ROBOT_ENV=ipa-apartment)

# rostest for cob3-5
rosbuild_add_roslaunch_check(robots/cob3-5.xml ROBOT=cob3-5 ROBOT_ENV=ipa-apartment)
rosbuild_add_roslaunch_check(tools ROBOT=cob3-5 ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(tools ROBOT=cob3-5 ROBOT_ENV=ipa-apartment)

# rostest for cob3-6
rosbuild_add_roslaunch_check(robots/cob3-6.xml ROBOT=cob3-6 ROBOT_ENV=ipa-apartment)
rosbuild_add_roslaunch_check(tools ROBOT=cob3-6 ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(tools ROBOT=cob3-6 ROBOT_ENV=ipa-apartment)

# rostest for raw3-1
rosbuild_add_roslaunch_check(robots/raw3-1.xml ROBOT=raw3-1 ROBOT_ENV=ipa-apartment)
rosbuild_add_roslaunch_check(tools ROBOT=raw3-1 ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(tools ROBOT=raw3-1 ROBOT_ENV=ipa-apartment)

# rostest for raw3-2
rosbuild_add_roslaunch_check(robots/raw3-2.xml ROBOT=raw3-2 ROBOT_ENV=ipa-apartment)
rosbuild_add_roslaunch_check(tools ROBOT=raw3-2 ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(tools ROBOT=raw3-2 ROBOT_ENV=ipa-apartment)

# rostest for desire
rosbuild_add_roslaunch_check(robots/desire.xml ROBOT=desire ROBOT_ENV=ipa-apartment)
rosbuild_add_roslaunch_check(tools ROBOT=desire ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(tools ROBOT=desire ROBOT_ENV=ipa-apartment)

#rosbuild_add_roslaunch_check(ros/launch/sim.launch ROBOT=cob3-3 ROBOT_ENV=ipa-kitchen)

# Check whether there's an X display.  Intended to use in gating tests that
# require a display.
#rosbuild_check_for_display(disp)

#if(disp)
#  rosbuild_add_rostest(ros/launch/sim.launch TIMEOUT 100.0)
#endif(disp)


