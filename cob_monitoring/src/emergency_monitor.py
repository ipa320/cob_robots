#!/usr/bin/python

import roslib
roslib.load_manifest('cob_monitoring')
import rospy

from cob_relayboard.msg import *

from simple_script_server import *
sss = simple_script_server()

status = None

def em_callback(em_status):
	global status
	if status != em_status.emergency_state:
		status = em_status.emergency_state
		rospy.loginfo("emergency change to "+ str(em_status.emergency_state))
		
		if status == 0: # ready
			sss.set_light("green")
		elif status == 1: # em stop
			sss.set_light("red")
			sss.say(["emergency stop executed"])
		elif status == 2: # release
			sss.set_light("yellow")
			sss.say(["emergency stop released"])


if __name__ == "__main__":
	rospy.init_node("emergency_monitor")
	rospy.Subscriber("/emergency_stop_state", EmergencyStopState, em_callback)
	rospy.spin()
