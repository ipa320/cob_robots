cmake_minimum_required(VERSION 2.8.3)
project(cob_supported_robots)

find_package(catkin REQUIRED)

set(robotlist
    cob4-2
    cob4-3
    cob4-4
    cob4-5
    cob4-6
    cob4-7
    cob4-10
    raw3-1
    raw3-3
    raw3-5
)

catkin_package(
    CFG_EXTRAS cob_supported_robots-extras.cmake
)
