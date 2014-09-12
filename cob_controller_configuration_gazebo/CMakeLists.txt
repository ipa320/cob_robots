# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_controller_configuration_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

# rostest for robots
set(robotlist
cob3-1
cob3-2
cob3-3
cob3-4
cob3-5
cob3-6
cob3-7
cob3-8
cob4-1
cob4-2
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
  roslaunch_add_file_check(launch/robots/default_controllers_${robot}.launch)
endforeach(robot)

catkin_package(
  CATKIN_DEPENDS
)


## Mark cpp header files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY controller
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/fake_diagnostics.py src/gazebo_services.py src/gazebo_topics.py src/gazebo_services_base.py src/cob_controller_adapter_gazebo.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

install(PROGRAMS scripts/test_publisher_pos_control.py scripts/test_publisher_vel_control.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
