cmake_minimum_required(VERSION 3.0.2)
project(cob_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

