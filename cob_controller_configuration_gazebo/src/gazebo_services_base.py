#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_controller_configuration_gazebo')

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction

# care-o-bot includes
from cob_srvs.srv import *

class gazebo_services_base():

	def __init__(self):
		# base
		self.base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
		#self.base_init_srv = rospy.Service('/base_controller/init', Trigger, self.base_init_cb)
		self.base_stop_srv = rospy.Service('/base_controller/stop', Trigger, self.base_stop_cb)
		#self.base_recover_srv = rospy.Service('/base_controller/recover', Trigger, self.base_recover_cb)
		#self.base_set_operation_mode_srv = rospy.Service('/base_controller/set_operation_mode', SetOperationMode, self.base_set_operation_mode_cb)

	# base
	def base_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def base_stop_cb(self, req):
		self.base_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def base_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def base_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp


if __name__ == "__main__":
   rospy.init_node('gazebo_services_base')
   gazebo_services_base()
   rospy.loginfo("gazebo_services_base running")
   rospy.spin()

