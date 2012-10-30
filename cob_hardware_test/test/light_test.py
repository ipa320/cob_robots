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

	def test_set_light(self):
		self.assertTrue(dialog_client(0, 'Watch out for the Light' ))
		self.sss.set_light("red")
		rospy.sleep(1.0)
		self.sss.set_light("green")
		rospy.sleep(1.0)
		self.sss.set_light("blue")
		rospy.sleep(1.0)
		self.sss.set_light([ 0.0, 0.0, 0.0 ])#turn light off
		self.assertTrue(dialog_client(1, 'Did I light up in red green and blue?'))

if __name__ == '__main__':
    	# Copied from hztest: A dirty hack to work around an apparent race condition at startup
    	# that causes some hztests to fail.  Most evident in the tests of
    	# rosstage.
    	time.sleep(0.75)
	try:
		rostest.run('rostest', 'test_hardware_test', HardwareTest, sys.argv)
	except KeyboardInterrupt, e:
		pass
	print "exiting"
