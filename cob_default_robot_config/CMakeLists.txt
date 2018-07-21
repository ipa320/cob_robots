cmake_minimum_required(VERSION 2.8.3)
project(cob_default_robot_config)

find_package(catkin REQUIRED)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  foreach(robot ${cob_supported_robots_ROBOTLIST})
    message("testing for robot: ${robot}")
    #roslaunch_add_file_check(upload_param.launch robot:=${robot})
    roslaunch_add_file_check(robots/${robot}/upload_param_${robot}.launch robot:=${robot})
  endforeach()
endif()

### INSTALL ###
install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES upload_param.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
