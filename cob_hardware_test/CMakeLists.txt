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
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


# rostest for current robot
SET(TRIGGER $ENV{MANUAL_TEST})

# run MANUAL tests if environment variable MANUAL_TEST is set and not set to [false, False, FALSE,OFF,...], see http://www.cmake.org/cmake/help/syntax.html
IF(TRIGGER)
 MESSAGE("Manual testing. MANUAL_TEST is set to $ENV{MANUAL_TEST}")

# run fully automated tests if environment variable MANUAL_TEST is not set or set to [false, False, FALSE,OFF,...], see http://www.cmake.org/cmake/help/syntax.html
ELSE(TRIGGER)
 MESSAGE("Fully automated testing. MANUAL_TEST is set to $ENV{MANUAL_TEST}")
 rosbuild_add_roslaunch_check(launch)

ENDIF(TRIGGER)
