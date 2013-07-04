# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_controller_configuration_gazebo)




# rostest cob
#rosbuild_add_roslaunch_check(ros/launch/default_controllers_cob3-1.launch robot:=cob3-1)
#rosbuild_add_roslaunch_check(ros/launch/default_controllers_cob3-2.launch robot:=cob3-2)
#rosbuild_add_roslaunch_check(ros/launch/default_controllers_cob3-3.launch robot:=cob3-3)
#rosbuild_add_roslaunch_check(ros/launch/default_controllers_cob3-4.launch robot:=cob3-4)
#rosbuild_add_roslaunch_check(ros/launch/default_controllers_cob3-5.launch robot:=cob3-5)
#rosbuild_add_roslaunch_check(ros/launch/default_controllers_cob3-6.launch robot:=cob3-6)

# rostest raw
#rosbuild_add_roslaunch_check(ros/launch/default_controllers_raw3-1.launch robot:=raw3-1)
##rosbuild_add_roslaunch_check(ros/launch/default_controllers_raw3-2.launch robot:=raw3-2)
##rosbuild_add_roslaunch_check(ros/launch/default_controllers_raw3-3.launch robot:=raw3-3)
#rosbuild_add_roslaunch_check(ros/launch/default_controllers_raw3-4.launch robot:=raw3-4)

# rostest desire
#rosbuild_add_roslaunch_check(ros/launch/default_controllers_desire.launch robot:=desire)

catkin_package(
    DEPENDS rostopic joint_trajectory_action cob_bringup cob_sick_s300 cob_relayboard cob_sound image_proc schunk_simulated_tactile_sensors cob_phidgets
 )