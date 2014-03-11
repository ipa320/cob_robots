# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_monitoring)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    DEPENDS cob_script_server cob_relayboard
)