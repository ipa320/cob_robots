cmake_minimum_required(VERSION 2.8.3)
project(cob_behaviour)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cob_script_server
)

catkin_package(
    CATKIN_DEPENDS cob_light rospy std_msgs std_srvs
)

set(robotlist
    cob4-2
)

foreach(robot ${robotlist})
    install(PROGRAMS scripts/trigger_srvs_${robot}.py
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach()
