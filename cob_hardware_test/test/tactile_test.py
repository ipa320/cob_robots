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
from cob_hardware_test.srv import *
from schunk_sdh.msg import TactileSensor

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
        rospy.init_node('teleop_test')
        self.message_received = False

        try:
            # topic
            if not rospy.has_param('~topic'):
                self.fail('Parameter topic does not exist on ROS Parameter Server')
            self.topic = rospy.get_param('~topic')
            # required value
            if not rospy.has_param('~min_tactile_value'):
                self.fail('Parameter min_tactile_value does not exist on ROS Parameter Server')
            self.min_tactile_value = rospy.get_param('~min_tactile_value')
            #test time after confirm
            if not rospy.has_param('~user_time'):
                 self.fail('Parameter user_time does not exist on ROS Parameter Server')
            self.user_time = rospy.get_param('~user_time')
            #test time after confirm
            if not rospy.has_param('~user_message'):
                 self.fail('Parameter user_message does not exist on ROS Parameter Server')
            self.user_message = rospy.get_param('~user_message')

        except KeyError, e:
            self.fail('Parameters not set properly')
            
            
    def test_tactile(self):
        #aks user
        self.assertTrue(dialog_client(0, self.user_message))
        # init component
        sub = rospy.Subscriber(self.topic, TactileSensor, self.cb_state)
        abort_time = time.time() + self.user_time
        while (not self.message_received) and time.time() < abort_time:
            time.sleep(0.1)          
        if not self.message_received:
            self.fail('No state message received within %s seconds' % (self.user_time))

    # callback functions
    def cb_state(self, msg):
        self.actual_values = msg.tactile_array
        if (any(self.actual_value) > self.min_tactile_value):
            self.message_received = True


if __name__ == '__main__':
    try:
        rostest.run('rostest', 'component_test', UnitTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"

