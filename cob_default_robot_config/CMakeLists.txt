cmake_minimum_required(VERSION 2.8.3)
project(cob_default_robot_config)

# rostest for robots
SET(robotlist
cob3-1
cob3-2
cob3-3
cob3-4
cob3-5
cob3-6
raw3-1
raw3-2
raw3-3
raw3-4
desire
)

# loop through list of robots for testing
FOREACH(robot ${robotlist})
	MESSAGE("testing for robot: ${robot}")
	#rosbuild_add_roslaunch_check(${robot}/upload_param_${robot}.launch)
ENDFOREACH(robot)

catkin_package()

FOREACH(robot ${robotlist})
    install(DIRECTORY ${robot}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${robot}
    )
ENDFOREACH(robot)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  upload_param.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)