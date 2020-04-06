cmake_minimum_required(VERSION 2.8.3)
project(cob_hardware_config)

find_package(catkin REQUIRED)
catkin_package()

### TESTING ###
if(CATKIN_ENABLE_TESTING)
    find_package(cob_supported_robots REQUIRED)
    find_package(roslaunch REQUIRED)
    find_package(rostest REQUIRED)
    # loop through list of robots for testing
    foreach(robot ${cob_supported_robots_ROBOTLIST})
        message("testing for robot: ${robot}")
        roslaunch_add_file_check(upload_robot.launch robot:=${robot})
        add_rostest(test/urdf.test ARGS robot:=${robot})
    endforeach()
endif()

### INSTALL ###
install(DIRECTORY robots test
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES upload_robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES display_robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
