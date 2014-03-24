# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_controller_configuration_gazebo)

find_package(catkin REQUIRED)

# rostest for robots
set(robotlist
cob3-1
cob3-2
cob3-3
cob3-4
cob3-5
cob3-6
cob3-7
raw3-1
raw3-2
raw3-3
raw3-4
raw3-5
raw3-6
desire
)

# loop through list of robots for testing
foreach(robot ${robotlist})
	message("testing for robot: ${robot}")
	#rosbuild_add_roslaunch_check(ros/launch/default_controllers_${robot}.launch robot:=${robot})
endforeach(robot)

catkin_package(
    CATKIN_DEPENDS rostopic joint_trajectory_action cob_bringup cob_sick_s300 cob_relayboard cob_sound image_proc schunk_simulated_tactile_sensors cob_phidgets
 )


## Mark cpp header files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY controller
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/fake_diagnostics.py src/gazebo_services.py src/gazebo_topics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
