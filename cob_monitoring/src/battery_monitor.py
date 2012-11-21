#!/usr/bin/python

import roslib
roslib.load_manifest('cob_monitoring')
import rospy

from pr2_msgs.msg import *

from simple_script_server import *
sss = simple_script_server()

##################
### TODO: lower rate and speach output
##################

class battery_monitor():
	def __init__(self):
		rospy.Subscriber("/power_state", PowerState, self.power_state_callback)
		self.r_ok    = rospy.Rate(1/60.0) # check every 60 sec
		self.r_warn  = rospy.Rate(1/10.0)
		self.r_error = rospy.Rate(1/30.0)

	## Battery monitoring
	### TODO: make values parametrized through yaml file (depending on env ROBOT)
	def power_state_callback(self,msg):
		if msg.relative_capacity <= 10:
			rospy.logerr("Battery empty, recharge now! Battery state is at" + str(msg.relative_capacity) + "%.")
			#TODO: print "start flashing red fast --> action call to lightmode"
			sss.say(["My battery is low, please recharge."])
			self.r_error.sleep()
		elif msg.relative_capacity <= 30:			
			rospy.logwarn("Battery nearly empty, consider recharging. Battery state is at" + str(msg.relative_capacity) + "%.") 
			#TODO: "start flashing yellow slowly --> action call to lightmode"
			self.r_warn.sleep()
		else:
			rospy.logdebug("Battery level ok.")	
			self.r_ok.sleep()

if __name__ == "__main__":
	rospy.init_node("battery_monitor")
	battery_monitor()
	rospy.spin()
