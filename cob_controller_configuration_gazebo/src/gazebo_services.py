#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_controller_configuration_gazebo')

import rospy

# care-o-bot includes
from std_srvs.srv import *

class gazebo_services():

	def __init__(self):
		self.init_srv = rospy.Service('driver/init', Trigger, self.srv_cb)
		self.recover_srv = rospy.Service('driver/recover', Trigger, self.srv_cb)
		self.halt_srv = rospy.Service('driver/halt', Trigger, self.srv_cb)
		self.shutdown_srv = rospy.Service('driver/shutdown', Trigger, self.srv_cb)

	def srv_cb(self, req):
		resp = TriggerResponse()
		resp.success = True
		return resp


if __name__ == "__main__":
   rospy.init_node('gazebo_services')
   gazebo_services()
   rospy.loginfo("gazebo_services running")
   rospy.spin()

