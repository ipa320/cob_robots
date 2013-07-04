# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_hardware_test)


find_package(catkin REQUIRED COMPONENTS)

# check test files (offline)
# cob3-1
#rosbuild_add_roslaunch_check(robots/cob3-1_manual.test.xml robot:=cob3-1)
#rosbuild_add_roslaunch_check(robots/cob3-1_auto.test.xml robot:=cob3-1)
#rosbuild_add_roslaunch_check(robots/cob3-1_manual.test.xml ROBOT=cob3-1)
#rosbuild_add_roslaunch_check(robots/cob3-1_auto.test.xml ROBOT=cob3-1)

# cob3-2
#rosbuild_add_roslaunch_check(robots/cob3-2_manual.test.xml robot:=cob3-2)
#rosbuild_add_roslaunch_check(robots/cob3-2_auto.test.xml robot:=cob3-2)
#rosbuild_add_roslaunch_check(robots/cob3-2_manual.test.xml ROBOT=cob3-2)
#rosbuild_add_roslaunch_check(robots/cob3-2_auto.test.xml ROBOT=cob3-2)

# cob3-3
#rosbuild_add_roslaunch_check(robots/cob3-3_manual.test.xml robot:=cob3-3)
#rosbuild_add_roslaunch_check(robots/cob3-3_auto.test.xml robot:=cob3-3)
#rosbuild_add_roslaunch_check(robots/cob3-3_manual.test.xml ROBOT=cob3-3)
#rosbuild_add_roslaunch_check(robots/cob3-3_auto.test.xml ROBOT=cob3-3)

# cob3-4
#rosbuild_add_roslaunch_check(robots/cob3-4_manual.test.xml robot:=cob3-4)
#rosbuild_add_roslaunch_check(robots/cob3-4_auto.test.xml robot:=cob3-4)
#rosbuild_add_roslaunch_check(robots/cob3-4_manual.test.xml ROBOT=cob3-4)
#rosbuild_add_roslaunch_check(robots/cob3-4_auto.test.xml ROBOT=cob3-4)

# cob3-5
#rosbuild_add_roslaunch_check(robots/cob3-5_manual.test.xml robot:=cob3-5)
#rosbuild_add_roslaunch_check(robots/cob3-5_auto.test.xml robot:=cob3-5)
#rosbuild_add_roslaunch_check(robots/cob3-5_manual.test.xml ROBOT=cob3-5)
#rosbuild_add_roslaunch_check(robots/cob3-5_auto.test.xml ROBOT=cob3-5)

# cob3-6
#rosbuild_add_roslaunch_check(robots/cob3-6_manual.test.xml robot:=cob3-6)
##rosbuild_add_roslaunch_check(robots/cob3-6_auto.test.xml robot:=cob3-6)
#rosbuild_add_roslaunch_check(robots/cob3-6_manual.test.xml ROBOT=cob3-6)
#rosbuild_add_roslaunch_check(robots/cob3-6_auto.test.xml ROBOT=cob3-6)

# raw3-1
#rosbuild_add_roslaunch_check(robots/raw3-1_manual.test.xml robot:=raw3-1)
#rosbuild_add_roslaunch_check(robots/raw3-1_auto.test.xml robot:=raw3-1)
#rosbuild_add_roslaunch_check(robots/raw3-1_manual.test.xml ROBOT=raw3-1)
#rosbuild_add_roslaunch_check(robots/raw3-1_auto.test.xml ROBOT=raw3-1)

# raw3-2
#rosbuild_add_roslaunch_check(robots/raw3-2_manual.test.xml robot:=raw3-2)
#rosbuild_add_roslaunch_check(robots/raw3-2_auto.test.xml robot:=raw3-2)
#rosbuild_add_roslaunch_check(robots/raw3-2_manual.test.xml ROBOT=raw3-2)
#rosbuild_add_roslaunch_check(robots/raw3-2_auto.test.xml ROBOT=raw3-2)

# desire
#rosbuild_add_roslaunch_check(robots/desire_manual.test.xml robot:=desire)
#rosbuild_add_roslaunch_check(robots/desire_auto.test.xml robot:=desire)
#rosbuild_add_roslaunch_check(robots/desire_manual.test.xml ROBOT=desire)
#rosbuild_add_roslaunch_check(robots/desire_auto.test.xml ROBOT=desire)

# rostest for current robot (online)
# start tests with 
# "rostest cob_hardware_test robot.test test_auto=true"
# or
# "rostest cob_hardware_test robot.test test_manual=true"
catkin_package(
    DEPENDS rostest cob_bringup cob_default_robot_config
)