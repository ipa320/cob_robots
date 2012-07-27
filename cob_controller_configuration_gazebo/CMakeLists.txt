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
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/lib)

# rostest cob
rosbuild_add_roslaunch_check(ros/launch/default_controllers_cob3-1.launch ROBOT=cob3-1)
rosbuild_add_roslaunch_check(ros/launch/default_controllers_cob3-2.launch ROBOT=cob3-2)
rosbuild_add_roslaunch_check(ros/launch/default_controllers_cob3-3.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(ros/launch/default_controllers_cob3-4.launch ROBOT=cob3-4)
rosbuild_add_roslaunch_check(ros/launch/default_controllers_cob3-5.launch ROBOT=cob3-5)
rosbuild_add_roslaunch_check(ros/launch/default_controllers_cob3-6.launch ROBOT=cob3-6)

# rostest raw
rosbuild_add_roslaunch_check(ros/launch/default_controllers_raw3-1.launch ROBOT=raw3-1)
rosbuild_add_roslaunch_check(ros/launch/default_controllers_raw3-2.launch ROBOT=raw3-2)

# rostest desire
rosbuild_add_roslaunch_check(ros/launch/default_controllers_desire.launch ROBOT=desire)
