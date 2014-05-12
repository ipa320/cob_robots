#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_controller_configuration_gazebo')

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction

# care-o-bot includes
from cob_srvs.srv import *

class gazebo_services():

	def __init__(self):
		self.action_client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
		self.init_srv = rospy.Service('init', Trigger, self.init_cb)
		self.stop_srv = rospy.Service('stop', Trigger, self.stop_cb)
		self.recover_srv = rospy.Service('recover', Trigger, self.recover_cb)
		self.set_operation_mode_srv = rospy.Service('set_operation_mode', SetOperationMode, self.set_operation_mode_cb)

	def init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def stop_cb(self, req):
		self.action_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp


if __name__ == "__main__":
   rospy.init_node('gazebo_services')
   gazebo_services()
   rospy.loginfo("gazebo_services running")
   rospy.spin()

