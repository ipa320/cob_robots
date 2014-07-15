# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_bringup)

find_package(catkin REQUIRED COMPONENTS roslaunch)

# rostest for robots
set(robotlist
cob3-1
cob3-2
cob3-3
cob3-4
cob3-5
cob3-6
cob3-7
cob3-8
raw3-1
raw3-2
raw3-3
raw3-4
raw3-5
raw3-6
desire
)

set(envlist
empty
ipa-kitchen
ipa-apartment
ipa-factory
)

# loop through list of robots for testing
foreach(robot ${robotlist})
	#foreach(env ${envlist})
		message("testing for robot: ${robot} in env: ${env}")
		#rosbuild_add_roslaunch_check(tools robot:=${robot} robot_env:=${env})
		#rosbuild_add_roslaunch_check(tools ROBOT=${robot} ROBOT_ENV=${env})
		roslaunch_add_file_check(robots/${robot}.xml robot_env:=ipa-apartment)
	#endforeach(env)
endforeach(robot)


##rosbuild_add_roslaunch_check(ros/launch/sim.launch ROBOT=cob3-3 ROBOT_ENV=ipa-kitchen)

# Check whether there's an X display.  Intended to use in gating tests that
# require a display.
#rosbuild_check_for_display(disp)

#if(disp)
#  #rosbuild_add_rostest(ros/launch/sim.launch TIMEOUT 100.0)
#endif(disp)

catkin_package()

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

