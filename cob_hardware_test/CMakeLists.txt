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
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# check test files (offline)
# cob3-1
rosbuild_add_roslaunch_check(robots/cob3-1_manual.test.xml robot:=cob3-1)
rosbuild_add_roslaunch_check(robots/cob3-1_auto.test.xml robot:=cob3-1)
rosbuild_add_roslaunch_check(robots/cob3-1_manual.test.xml ROBOT=cob3-1)
rosbuild_add_roslaunch_check(robots/cob3-1_auto.test.xml ROBOT=cob3-1)

# cob3-2
rosbuild_add_roslaunch_check(robots/cob3-2_manual.test.xml robot:=cob3-2)
rosbuild_add_roslaunch_check(robots/cob3-2_auto.test.xml robot:=cob3-2)
rosbuild_add_roslaunch_check(robots/cob3-2_manual.test.xml ROBOT=cob3-2)
rosbuild_add_roslaunch_check(robots/cob3-2_auto.test.xml ROBOT=cob3-2)

# cob3-3
rosbuild_add_roslaunch_check(robots/cob3-3_manual.test.xml robot:=cob3-3)
rosbuild_add_roslaunch_check(robots/cob3-3_auto.test.xml robot:=cob3-3)
rosbuild_add_roslaunch_check(robots/cob3-3_manual.test.xml ROBOT=cob3-3)
rosbuild_add_roslaunch_check(robots/cob3-3_auto.test.xml ROBOT=cob3-3)

# cob3-4
rosbuild_add_roslaunch_check(robots/cob3-4_manual.test.xml robot:=cob3-4)
rosbuild_add_roslaunch_check(robots/cob3-4_auto.test.xml robot:=cob3-4)
rosbuild_add_roslaunch_check(robots/cob3-4_manual.test.xml ROBOT=cob3-4)
rosbuild_add_roslaunch_check(robots/cob3-4_auto.test.xml ROBOT=cob3-4)

# cob3-5
rosbuild_add_roslaunch_check(robots/cob3-5_manual.test.xml robot:=cob3-5)
rosbuild_add_roslaunch_check(robots/cob3-5_auto.test.xml robot:=cob3-5)
rosbuild_add_roslaunch_check(robots/cob3-5_manual.test.xml ROBOT=cob3-5)
rosbuild_add_roslaunch_check(robots/cob3-5_auto.test.xml ROBOT=cob3-5)

# cob3-6
rosbuild_add_roslaunch_check(robots/cob3-6_manual.test.xml robot:=cob3-6)
rosbuild_add_roslaunch_check(robots/cob3-6_auto.test.xml robot:=cob3-6)
rosbuild_add_roslaunch_check(robots/cob3-6_manual.test.xml ROBOT=cob3-6)
rosbuild_add_roslaunch_check(robots/cob3-6_auto.test.xml ROBOT=cob3-6)

# raw3-1
rosbuild_add_roslaunch_check(robots/raw3-1_manual.test.xml robot:=raw3-1)
rosbuild_add_roslaunch_check(robots/raw3-1_auto.test.xml robot:=raw3-1)
rosbuild_add_roslaunch_check(robots/raw3-1_manual.test.xml ROBOT=raw3-1)
rosbuild_add_roslaunch_check(robots/raw3-1_auto.test.xml ROBOT=raw3-1)

# raw3-2
rosbuild_add_roslaunch_check(robots/raw3-2_manual.test.xml robot:=raw3-2)
rosbuild_add_roslaunch_check(robots/raw3-2_auto.test.xml robot:=raw3-2)
rosbuild_add_roslaunch_check(robots/raw3-2_manual.test.xml ROBOT=raw3-2)
rosbuild_add_roslaunch_check(robots/raw3-2_auto.test.xml ROBOT=raw3-2)

# desire
rosbuild_add_roslaunch_check(robots/desire_manual.test.xml robot:=desire)
rosbuild_add_roslaunch_check(robots/desire_auto.test.xml robot:=desire)
rosbuild_add_roslaunch_check(robots/desire_manual.test.xml ROBOT=desire)
rosbuild_add_roslaunch_check(robots/desire_auto.test.xml ROBOT=desire)

# rostest for current robot (online)
# start tests with 
# "rostest cob_hardware_test robot.test test_auto=true"
# or
# "rostest cob_hardware_test robot.test test_manual=true"
