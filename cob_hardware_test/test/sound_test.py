#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_hardware_test')

import sys
import unittest
import rospy
import rostest

from cob_hardware_test.srv import *
from std_msgs.msg import String
from simple_script_server import *
from pr2_controllers_msgs.msg import *


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
		self.message_received = False
		self.sss = simple_script_server()

	def test_play(self):
		dialog_client(0, 'Listen up for the Sound' )
		rospy.set_param("script_server/sound/language","de")
		handle = self.sss.play("grasp_tutorial_01")
		self.assertEqual(handle.get_state(), 3)
		self.assertTrue(dialog_client(1, 'Did you hear File?'))		
		
		
	def test_say(self):
		dialog_client(0, 'Listen up for the Sound' )
		handle = self.sss.say(["Hello"])
		self.assertEqual(handle.get_state(), 3)
		self.assertTrue(dialog_client(1, 'Did you hear Hello?'))

if __name__ == '__main__':

	try:
		rostest.run('rostest', 'test_hardware_test', HardwareTest, sys.argv)
	except KeyboardInterrupt, e:
		pass
	print "exiting"
