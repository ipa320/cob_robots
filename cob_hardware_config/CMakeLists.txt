cmake_minimum_required(VERSION 2.8.3)
project(cob_hardware_config)

find_package(catkin REQUIRED COMPONENTS roslaunch rostest)

# rostest for robots
set(robotlist
cob3-6
cob3-9
cob4-1
cob4-2
raw3-1
raw3-2
raw3-3
raw3-4
raw3-5
raw3-6
)

# loop through list of robots for testing
foreach(robot ${robotlist})
    message("testing for robot: ${robot}")
    #roslaunch_add_file_check(common robot:=${robot})
    #roslaunch_add_file_check(common ROBOT=${robot})
    add_rostest(test/urdf_${robot}.test)
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

