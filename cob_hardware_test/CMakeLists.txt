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
rosbuild_add_roslaunch_check(robots/robot.test.xml ROBOT=cob3-1)
rosbuild_add_roslaunch_check(robots/robot.test.xml ROBOT=cob3-2)
rosbuild_add_roslaunch_check(robots/robot.test.xml ROBOT=cob3-3)
rosbuild_add_roslaunch_check(robots/robot.test.xml ROBOT=cob3-4)
rosbuild_add_roslaunch_check(robots/robot.test.xml ROBOT=cob3-5)
rosbuild_add_roslaunch_check(robots/robot.test.xml ROBOT=cob3-6)
rosbuild_add_roslaunch_check(robots/robot.test.xml ROBOT=raw3-1)
rosbuild_add_roslaunch_check(robots/robot.test.xml ROBOT=raw3-2)
rosbuild_add_roslaunch_check(robots/robot.test.xml ROBOT=desire)

# rostest for current robot (online)
SET(TRIGGER_AUTO $ENV{TEST_AUTO})
SET(TRIGGER_MANUAL $ENV{TEST_MANUAL})

# run AUTO tests if environment variable TEST_AUTO is set, but not set to [false, False, FALSE,OFF,...], see http://www.cmake.org/cmake/help/syntax.html
IF(TRIGGER_AUTO)
 MESSAGE("Fully automated testing. TEST_AUTO is set to $ENV{TEST_AUTO}")
 rosbuild_add_rostest(robots/robot_auto.test)
ENDIF(TRIGGER_AUTO)

# run MANUAL tests if environment variable TEST_MANUAL is set, but not set to [false, False, FALSE,OFF,...], see http://www.cmake.org/cmake/help/syntax.html
IF(TRIGGER_MANUAL)
 MESSAGE("Manual testing. TEST_MANUAL is set to $ENV{TEST_MANUAL}")
 rosbuild_add_rostest(robots/robot_manual.test)
ENDIF(TRIGGER_MANUAL)
