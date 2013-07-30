#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_hardware_test')
#from mpmath.functions.functions import fabs
import sys
import time
import unittest

import rospy
import rostest
from cob_hardware_test.srv import *
from sensor_msgs.msg import Joy

def dialog_client(dialog_type, message):
    #dialog type: 0=confirm 1=question
    rospy.wait_for_service('dialog')
    try:
        dialog = rospy.ServiceProxy('dialog', Dialog)
        resp1 = dialog(dialog_type, message)
        return resp1.answer
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

class UnitTest(unittest.TestCase):
    def __init__(self, *args):
        super(UnitTest, self).__init__(*args)
        rospy.init_node('joy_test')
        self.message_received = False
        # init subscribers
        #state_topic = "/joy" 
        #sub_state_topic = rospy.Subscriber(state_topic, JointTrajectoryControllerState, self.cb_state)

    def test_component(self):
        self.assertTrue(dialog_client(0, 'Press at least one Button of the Controller within 60 seconds after Confirm'))
        # init component
        sub = rospy.Subscriber("/joy", Joy, self.cb_state)
        abort_time = time.time() + 60.0 #rospy.Duration(self.wait_time)
        while not self.message_received and time.time() < abort_time:
            time.sleep(0.1)          
        if not self.message_received:
            self.fail('No state message received within 60 seconds')

    # callback functions
    def cb_state(self, msg):
        #self.actual_pos = msg.actual.positions
        self.message_received = True


if __name__ == '__main__':
    try:
        rostest.run('rostest', 'joy_test', UnitTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"

