cmake_minimum_required(VERSION 2.8.3)
project(cob_default_robot_config)

find_package(catkin REQUIRED)

# rostest for robots
set(robotlist
cob3-1
cob3-2
cob3-3
cob3-4
cob3-5
cob3-6
cob3-7
cob4-1
raw3-1
raw3-2
raw3-3
raw3-4
raw3-5
raw3-6
desire
)

# loop through list of robots for testing
foreach(robot ${robotlist})
	message("testing for robot: ${robot}")
	#rosbuild_add_roslaunch_check(upload_param.launch robot:=${robot})
	#rosbuild_add_roslaunch_check(upload_param.launch ROBOT=${robot})
	#rosbuild_add_roslaunch_check(${robot}/upload_param_${robot}.launch robot:=${robot})
	#rosbuild_add_roslaunch_check(${robot}/upload_param_${robot}.launch ROBOT=${robot})
endforeach(robot)

catkin_package()

foreach(robot ${robotlist})
    install(DIRECTORY ${robot}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endforeach(robot)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES upload_param.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
