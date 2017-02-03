cmake_minimum_required(VERSION 2.8.3)
project(cob_controller_configuration_gazebo)

find_package(catkin REQUIRED)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(cob_supported_robots REQUIRED)
  find_package(roslaunch REQUIRED)
  foreach(robot ${cob_supported_robots_ROBOTLIST})
    message("testing for robot: ${robot}")
    roslaunch_add_file_check(launch/robots/default_controllers_${robot}.launch)
  endforeach()
endif()

### INSTALL ###
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/gazebo_services.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)
