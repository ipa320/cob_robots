# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_hardware_config)

find_package(catkin REQUIRED COMPONENTS)

# rostest for cob3-1
#rosbuild_add_roslaunch_check(common robot:=cob3-1)
#rosbuild_add_roslaunch_check(common ROBOT=cob3-1)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(test/urdf_cob3-1.test)

# rostest for cob3-2
#rosbuild_add_roslaunch_check(common robot:=cob3-2)
#rosbuild_add_roslaunch_check(common ROBOT=cob3-2)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(test/urdf_cob3-2.test)

# rostest for cob3-3
#rosbuild_add_roslaunch_check(common robot:=cob3-3)
#rosbuild_add_roslaunch_check(common ROBOT=cob3-3)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(test/urdf_cob3-3.test)

# rostest for cob3-4
#rosbuild_add_roslaunch_check(common robot:=cob3-4)
#rosbuild_add_roslaunch_check(common ROBOT=cob3-4)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(test/urdf_cob3-4.test)

# rostest for cob3-5
#rosbuild_add_roslaunch_check(common robot:=cob3-5)
#rosbuild_add_roslaunch_check(common ROBOT=cob3-5)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(test/urdf_cob3-5.test)

# rostest for cob3-6
#rosbuild_add_roslaunch_check(common robot:=cob3-6)
#rosbuild_add_roslaunch_check(common ROBOT=cob3-6)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(test/urdf_cob3-6.test)

# rostest for raw3-1
#rosbuild_add_roslaunch_check(common robot:=raw3-1)	--> currently disabled due to wrong usage of ur10_robot macro (no prefix)
#rosbuild_add_roslaunch_check(common ROBOT=raw3-1)	--> currently disabled due to wrong usage of ur10_robot macro (no prefix)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(test/urdf_raw3-1.test)		--> currently disabled due to wrong usage of ur10_robot macro (no prefix)

# rostest for raw3-2
#rosbuild_add_roslaunch_check(common robot:=raw3-2)
#rosbuild_add_roslaunch_check(common ROBOT=raw3-2)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(test/urdf_raw3-2.test)

# rostest for raw3-3
#rosbuild_add_roslaunch_check(common robot:=raw3-3)	--> currently disabled due to not existing frida_description package
#rosbuild_add_roslaunch_check(common ROBOT=raw3-3)	--> currently disabled due to not existing frida_description package
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(test/urdf_raw3-3.test)		--> currently disabled due to not existing frida_description package

# rostest for raw3-4
#rosbuild_add_roslaunch_check(common robot:=raw3-4)
#rosbuild_add_roslaunch_check(common ROBOT=raw3-4)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(test/urdf_raw3-4.test)

# rostest for desire
#rosbuild_add_roslaunch_check(common robot:=desire)
#rosbuild_add_roslaunch_check(common ROBOT=desire)

catkin_package(
    DEPENDS cob_calibration_data cob_description raw_description schunk_description ur_description
)