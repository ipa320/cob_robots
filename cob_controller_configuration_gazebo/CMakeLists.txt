cmake_minimum_required(VERSION 2.8.3)
project(cob_controller_configuration_gazebo)

find_package(catkin REQUIRED COMPONENTS cob_supported_robots roslaunch)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  foreach(robot ${cob_supported_robots_ROBOTLIST})
    message("testing for robot: ${robot}")
    roslaunch_add_file_check(launch/robots/default_controllers_${robot}.launch)
  endforeach()
endif()

### INSTALL ###
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/fake_diagnostics.py src/fake_monitoring.py src/gazebo_services.py src/gazebo_topics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)
