cmake_minimum_required(VERSION 2.8.3)
project(cob_default_robot_config)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

set(robotlist
  cob3-2
  cob3-6
  cob3-9
  cob4-1
  cob4-2
  cob4-4
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
    #roslaunch_add_file_check(upload_param.launch robot:=${robot})
    roslaunch_add_file_check(${robot}/upload_param_${robot}.launch robot:=${robot})
  endforeach()
endif()

### INSTALL ###
foreach(robot ${robotlist})
  install(DIRECTORY ${robot}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

install(FILES upload_param.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
