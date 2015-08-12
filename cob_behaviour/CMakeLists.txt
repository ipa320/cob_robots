cmake_minimum_required(VERSION 2.8.3)
project(cob_behaviour)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

set(robotlist
    cob4-2
)

foreach(robot ${robotlist})
    install(PROGRAMS scripts/trigger_srvs_${robot}.py
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach()
