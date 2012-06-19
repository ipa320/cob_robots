#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_bringup')

import sys
import unittest
import rospy
import rostest

from cob_bringup.srv import *
from std_msgs.msg import String
from simple_script_server import *
from pr2_controllers_msgs.msg import *

sss = simple_script_server()


def cb_recieve_position(data):
	rospy.loginfo(rospy.get_name() + ": Actual Position: %s", data.actual.positions)
	rospy.loginfo(rospy.get_name() + ": Goal Position: %s", rospy.get_param("/script_server/arm/home"))
	#self.command_traj = data
	#unregister()

	#rospy.signal_shutdown('done')



def test_move(test, component, position):
	#get_state: 1=active 2=failed 3=succeeded
	#error code 0=succeeded int=failure
	test.assertTrue(dialog_client(0, 'Ready to move my %s %s ?' % (component, position)))
	sss.init(component)
	handle = sss.move(component, position)
	test.assertEqual(handle.get_state(), 3)
	test.assertTrue(check_state(component, position))
	test.assertTrue(dialog_client(1, 'Did my %s move %s ?' % (component, position)))


def check_state(component, position, error_range=0.1):
	topic_name = '/' + component + '_controller/state'
	script_server_name = '/script_server/' + component + '/' + position
	rospy.loginfo('topic:' + topic_name + 'sss:' + script_server_name)
	script_server_position = rospy.get_param(script_server_name)
	r = rospy.Subscriber(topic_name, JointTrajectoryControllerState, cb_recieve_position)


	return True
#    for i in range(state)
#    	if get_state(state_node)=<>rospy.get_param("/script_server/%s"%component_name, parameter_name)-+error_range

def dialog_client(dialog_type, message):
    #dialog type: 0=confirm 1=question
	rospy.wait_for_service('dialog')
	try:
		dialog = rospy.ServiceProxy('dialog', Dialog)
		resp1 = dialog(dialog_type, message)
		return resp1.answer
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e

class HardwareTest(unittest.TestCase):
	def __init__(self, *args):

		super(HardwareTest, self).__init__(*args)
		rospy.init_node('test_hardware_test')
		torso_joint_states = []

	def test_1_move_tray(self):
		test_move(self, 'tray', 'down')
		test_move(self, 'tray', 'up')
		test_move(self, 'arm', 'folded')
		test_move(self, 'arm', 'home')

"""
	def test_3_say(self):
		sss.say(["Hello"])#2 times because first time does not execute		
		handle = sss.say(["Hello"])
		self.assertEqual(handle.get_state(), 3)
		self.assertTrue(dialog_client(1, 'Did you hear Hello?'))

	def test_4_set_light(self):
		sss.set_light("red")
		rospy.sleep(0.7)
		sss.set_light("green")
		rospy.sleep(0.7)
		sss.set_light("blue")
		rospy.sleep(0.7)
		handle = sss.set_light([ 0.0, 0.0, 0.0 ])#turn light off
		self.assertTrue(dialog_client(1, 'Did I light up in red green and blue?'))
"""
if __name__ == '__main__':

	try:
		rostest.run('rostest', 'test_hardware_test', HardwareTest, sys.argv)
	except KeyboardInterrupt, e:
		pass
	print "exiting"
