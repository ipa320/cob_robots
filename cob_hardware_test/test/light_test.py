#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_hardware_test')
import sys
import unittest
import rospy
import rostest
from simple_script_server import *
from dialog_client import dialog_client

class HardwareTest(unittest.TestCase):
    def __init__(self, *args):

        super(HardwareTest, self).__init__(*args)
        rospy.init_node('light_test')
        torso_joint_states = []
        self.message_received = False
        self.sss = simple_script_server()

    def test_set_light(self):
        #self.sss.init("light")
        self.assertTrue(dialog_client(0, 'Watch out for the Light' ))
        self.change_light("red")
        rospy.sleep(3.0)
        self.change_light("green")
        rospy.sleep(3.0)
        self.change_light("blue")
        rospy.sleep(3.0)
        self.change_light("black")
        self.assertTrue(dialog_client(1, 'Did I light up in red, green and blue?'))

    def change_light(self,color):
        handle_light = self.sss.set_light(color)
        self.assertEqual(handle_light.get_state(), 3) # state 3 equals errorcode 0 therefore the following will never be executed
        if handle_light.get_error_code() != 0:
            error_msg = 'Could not set lights'
            self.fail(error_msg + "; errorCode: " + str(handle_light.get_error_code()))


if __name__ == '__main__':
    # Copied from hztest: A dirty hack to work around an apparent race condition at startup
    # that causes some hztests to fail. Most evident in the tests of
    # rosstage.
    time.sleep(0.75)
    try:
        rostest.run('rostest', 'light_test', HardwareTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"
