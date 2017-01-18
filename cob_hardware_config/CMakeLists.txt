cmake_minimum_required(VERSION 2.8.3)
project(cob_hardware_config)

find_package(catkin REQUIRED COMPONENTS cob_supported_robots)

catkin_package()

### TESTING ###
if(CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    find_package(rostest REQUIRED)
    # loop through list of robots for testing
    foreach(robot ${cob_supported_robots_ROBOTLIST})
        message("testing for robot: ${robot}")
        roslaunch_add_file_check(common robot:=${robot})
        roslaunch_add_file_check(common ROBOT=${robot})
        add_rostest(test/urdf_${robot}.test)
    endforeach()
endif()

### INSTALL ###
foreach(robot ${cob_supported_robots_ROBOTLIST})
    install(DIRECTORY ${robot}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endforeach()

install(DIRECTORY common test
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
