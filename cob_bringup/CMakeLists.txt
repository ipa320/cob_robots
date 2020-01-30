cmake_minimum_required(VERSION 2.8.3)
project(cob_bringup)

find_package(catkin REQUIRED)

catkin_package()

### TEST ###
if(CATKIN_ENABLE_TESTING)
  find_package(cob_supported_robots REQUIRED)
  find_package(roslaunch REQUIRED)
  foreach(robot ${cob_supported_robots_ROBOTLIST})
    message("testing for robot: ${robot}")
    roslaunch_add_file_check(robots/${robot}.xml)
  endforeach()

  # additional robots not officially supported
  set(test_list
    raw-mini
  )
  foreach(robot ${test_list})
    message("testing for robot: ${robot}")
    roslaunch_add_file_check(robots/${robot}.xml)
  endforeach()
endif()

### INSTALL ###
install(DIRECTORY components controllers drivers robots tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS 
  env.sh
  env.sh.display
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
