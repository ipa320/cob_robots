cmake_minimum_required(VERSION 2.8.3)
project(cob_hardware_config)

find_package(catkin REQUIRED COMPONENTS)

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
	#rosbuild_add_roslaunch_check(common robot:=${robot})
	#rosbuild_add_roslaunch_check(common ROBOT=${robot})
	#rosbuild_add_rostest(test/urdf_${robot}.test)
endforeach(robot)

catkin_package(
    CATKIN_DEPENDS cob_calibration_data cob_description raw_description schunk_description ur_description
)

foreach(robot ${robotlist})
    install(DIRECTORY ${robot}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endforeach(robot)


install(DIRECTORY common
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   )

install(DIRECTORY test
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   )

