# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_bringup)


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

SET(envlist
ipa-kitchen
ipa-apartment
)

# loop through list of robots for testing
FOREACH(robot ${robotlist})
	FOREACH(env ${envlist})
		MESSAGE("testing for robot: ${robot} in env: ${env}")
		#rosbuild_add_roslaunch_check(tools robot:=${robot} robot_env:=${env})
		#rosbuild_add_roslaunch_check(tools ROBOT=${robot} ROBOT_ENV=${env})
	ENDFOREACH(env)
ENDFOREACH(robot)


##rosbuild_add_roslaunch_check(ros/launch/sim.launch ROBOT=cob3-3 ROBOT_ENV=ipa-kitchen)

# Check whether there's an X display.  Intended to use in gating tests that
# require a display.
#rosbuild_check_for_display(disp)

#if(disp)
#  #rosbuild_add_rostest(ros/launch/sim.launch TIMEOUT 100.0)
#endif(disp)


