#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_hardware_test')
#from mpmath.functions.functions import fabs
import sys
import time
import unittest
import math

import rospy
import rostest
from simple_script_server import *
from dialog_client import dialog_client


class UnitTest(unittest.TestCase):
    def __init__(self, *args):
        super(UnitTest, self).__init__(*args)
        rospy.init_node('calibration_test')
        self.message_received = False
        self.sss = simple_script_server()


        # get parameters
        try:
            # movement command
            if not rospy.has_param('~test_target_torso'):
                self.fail('Parameter test_target_torso does not exist on ROS Parameter Server')
            self.test_target_torso = rospy.get_param('~test_target_torso')

            if not rospy.has_param('~test_target_head'):
                 self.fail('Parameter test_target_head does not exist on ROS Parameter Server')
            self.test_target_head = rospy.get_param('~test_target_head')
            
            if not rospy.has_param('~test_target_arm'):
                 self.fail('Parameter test_target_arm does not exist on ROS Parameter Server')
            self.test_target_arm = rospy.get_param('~test_target_arm')            

        except KeyError, e:
            self.fail('Parameters not set properly')


    def test_calibration(self):
        
        self.sss.init('head')
        self.sss.init('torso')
        self.sss.init('arm')

        self.assertTrue(dialog_client(0, 'Ready to move components to home and then to test position?'))

        self.sss.move('arm', 'home')
        self.sss.move('torso', 'home')
        self.sss.move('head', 'home')

        self.sss.move('arm', self.test_target_arm)
        self.sss.move('torso', self.test_target_torso)
        self.sss.move('head', self.test_target_head)        
        
        self.assertTrue(dialog_client(1, ' now check rviz. Is the calibration good?'))



if __name__ == '__main__':
    # Copied from hztest: A dirty hack to work around an apparent race condition at startup
    # that causes some hztests to fail. Most evident in the tests of
    # rosstage.
    time.sleep(0.75)
    try:
        rostest.run('rostest', 'calibration_test', UnitTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"

