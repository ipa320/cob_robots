# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_monitoring)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    CATKIN_DEPENDS cob_msgs cob_script_server
)

install(PROGRAMS src/battery_monitor.py src/emergency_stop_monitor.py src/cpu_monitor.py src/hd_monitor.py src/ntp_monitor.py src/wifi_monitor.py src/ddwrt.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

