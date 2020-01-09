cmake_minimum_required(VERSION 2.8.3)
project(cob_default_robot_behavior)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/trigger_srvs.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
