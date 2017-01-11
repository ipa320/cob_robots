cmake_minimum_required(VERSION 2.8.3)
project(cob_default_robot_config)

find_package(catkin REQUIRED COMPONENTS cob_supported_robots roslaunch)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  foreach(robot ${cob_supported_robots_ROBOTLIST})
    message("testing for robot: ${robot}")
    #roslaunch_add_file_check(upload_param.launch robot:=${robot})
    roslaunch_add_file_check(${robot}/upload_param_${robot}.launch robot:=${robot})
  endforeach()
endif()

### INSTALL ###
foreach(robot ${cob_supported_robots_ROBOTLIST})
  install(DIRECTORY ${robot}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

install(FILES upload_param.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
