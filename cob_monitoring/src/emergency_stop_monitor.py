#!/usr/bin/python

import roslib
roslib.load_manifest('cob_monitoring')
import rospy

from cob_relayboard.msg import *

from simple_script_server import *
sss = simple_script_server()

##################
### TODO: add diagnostics for em_stop (probably better to be implemented in relayboard) --> then create a diagnostics_monitor.py with sets leds and sound from diagnostics information (for arm, base, torso, ...)
### which color and flashing code assign to diagnostics?
##################

class emergency_stop_monitor():
	def __init__(self):
		rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.emergency_callback)
		self.em_status = EmergencyStopState()
		self.first_time = True

	## Emergency stop monitoring
	def emergency_callback(self,msg):
		# skip first message to avoid speach output on startup
		if self.first_time:
			self.first_time = False
			self.em_status = msg
			return
	
		if self.em_status.emergency_state != msg.emergency_state:
			self.em_status = msg
			rospy.loginfo("Emergency change to "+ str(self.em_status.emergency_state))
		
			if self.em_status.emergency_state == 0: # ready
				sss.set_light("green")
			elif self.em_status.emergency_state == 1: # em stop
				sss.set_light("red")
				if self.em_status.scanner_stop and not self.em_status.emergency_button_stop:
					sss.say(["laser emergency stop issued"])
				elif not self.em_status.scanner_stop and self.em_status.emergency_button_stop:
					sss.say(["emergency stop button pressed"])
				else:
					sss.say(["emergency stop issued"])
			elif self.em_status.emergency_state == 2: # release
				sss.set_light("yellow")
				sss.say(["emergency stop released"])

if __name__ == "__main__":
	rospy.init_node("emergency_stop_monitor")
	emergency_stop_monitor()
	rospy.spin()
