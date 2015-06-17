cmake_minimum_required(VERSION 2.8.3)
project(cob_bringup)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()



set(robotlist
  cob3-2
  cob3-6
  cob3-9
  cob4-1
  cob4-2
  cob4-6
  raw3-1
  raw3-2
  raw3-3
  raw3-4
  raw3-5
  raw3-6
)

set(envlist
  empty
  ipa-kitchen
  ipa-apartment
  ipa-factory
)

if(CATKIN_ENABLE_TESTING)
    foreach(robot ${robotlist})
	#foreach(env ${envlist})
	    message("testing for robot: ${robot} in env: ${env}")
	    #rosbuild_add_roslaunch_check(tools robot:=${robot} robot_env:=${env})
	    #rosbuild_add_roslaunch_check(tools ROBOT=${robot} ROBOT_ENV=${env})
	    roslaunch_add_file_check(robots/${robot}.xml robot_env:=ipa-apartment)
	#endforeach()
    endforeach()
endif()



## Mark launch files for installation
install(DIRECTORY robots drivers controllers tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS env.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

