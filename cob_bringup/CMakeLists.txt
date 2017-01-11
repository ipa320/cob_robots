cmake_minimum_required(VERSION 2.8.3)
project(cob_bringup)

find_package(catkin REQUIRED COMPONENTS cob_hardware_config roslaunch)

catkin_package()


### TEST ###
if(CATKIN_ENABLE_TESTING)
  foreach(robot ${cob_hardware_config_ROBOTLIST})
    message("testing for robot: ${robot}")
    roslaunch_add_file_check(robots/${robot}.xml ROBOT_ENV=${env})
  endforeach()
endif()

### INSTALL ###
install(DIRECTORY robots drivers controllers tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS env.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
