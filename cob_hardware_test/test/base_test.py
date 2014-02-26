#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_hardware_test')

import sys
import unittest
import rospy
import rostest

from std_msgs.msg import String
from simple_script_server import *
from dialog_client import dialog_client


class HardwareTest(unittest.TestCase):
    def __init__(self, *args):

        super(HardwareTest, self).__init__(*args)
        rospy.init_node('base_test')
        self.message_received = False
        self.sss = simple_script_server()

        try:
                # move_
                if not rospy.has_param('~move_x'):
                    self.fail('Parameter move_x does not exist on ROS Parameter Server')
                self.move_x = rospy.get_param('~move_x')

                if not rospy.has_param('~move_y'):
                    self.fail('Parameter move_ does not exist on ROS Parameter Server')
                self.move_y = rospy.get_param('~move_y')

                if not rospy.has_param('~move_theta'):
                    self.fail('Parameter move_ does not exist on ROS Parameter Server')
                self.move_theta = rospy.get_param('~move_theta')

        except KeyError, e:
                self.fail('Parameters not set properly')


    def test_base(self):
        self.sss.init("base")
        self.assertTrue(dialog_client(0, 'Ready to move base?' ))
        handle = self.sss.move_base_rel("base", [self.move_x, self.move_y, self.move_theta])
        self.assertEqual(handle.get_state(), 3)
        self.assertTrue(dialog_client(1, 'Did I move?'))
        self.assertTrue(dialog_client(0, 'EM Pressed and Released? \n Ready to move my Base ?' ))
        self.sss.recover("base")
        handle = self.sss.move_base_rel("base", [-self.move_x, -self.move_y, -self.move_theta])
        self.assertTrue(dialog_client(1, 'Did I move back?'))

if __name__ == '__main__':
    # Copied from hztest: A dirty hack to work around an apparent race condition at startup
    # that causes some hztests to fail. Most evident in the tests of
    # rosstage.
    time.sleep(0.75)
    try:
        rostest.run('rostest', 'base_test', HardwareTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"
