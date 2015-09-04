cmake_minimum_required(VERSION 2.8.3)
project(cob_controller_configuration_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

set(robotlist
  cob3-6
  cob3-9
  cob4-1
  cob4-2
  cob4-6
  raw3-1
  raw3-2
  raw3-3
  raw3-4
  raw3-5
  raw3-6
)

if(CATKIN_ENABLE_TESTING)
  foreach(robot ${robotlist})
    message("testing for robot: ${robot}")
    roslaunch_add_file_check(launch/robots/default_controllers_${robot}.launch)
  endforeach()
endif()

### INSTALL ###
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/fake_diagnostics.py src/gazebo_services.py src/gazebo_topics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)
