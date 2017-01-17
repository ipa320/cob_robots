cmake_minimum_required(VERSION 2.8.3)
project(cob_moveit_config)

find_package(catkin REQUIRED COMPONENTS cob_hardware_config)

catkin_package()

foreach(robot ${cob_hardware_config_ROBOTLIST})
    install(DIRECTORY ${robot}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endforeach()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS bin/add_robot.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
