cmake_minimum_required(VERSION 2.8.3)
project(cob_default_robot_behavior)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()

set(robotlist
    cob4-5
    cob4-7
    cob4-8
    cob4-10
)

foreach(robot ${robotlist})
    install(PROGRAMS scripts/trigger_srvs_${robot}.py
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach()
