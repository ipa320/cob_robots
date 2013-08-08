#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_hardware_test')
import sys
import time
import unittest
import rospy
import rostest
from geometry_msgs.msg import Twist
from dialog_client import dialog_client

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
            # desired movement commands
            if not rospy.has_param('~min_target_x'):
                self.fail('Parameter min_target_x does not exist on ROS Parameter Server')
            self.min_target_x = rospy.get_param('~min_target_x')
            if not rospy.has_param('~min_target_y'):
                 self.fail('Parameter min_test_target_y does not exist on ROS Parameter Server')
            self.min_target_y = rospy.get_param('~min_target_y')


        except KeyError, e:
            self.fail('Parameters not set properly')
            
            
    def test_teleop(self):
        self.assertTrue(dialog_client(0, 'Move the left base controller stick to the left top and hold the deadmans button after confirm or press W on the Keyboard within 60 seconds'))
        # init component
        sub = rospy.Subscriber(self.topic, Twist, self.cb_state)
        abort_time = time.time() + 60.0
        while (not self.message_received) and time.time() < abort_time:
            time.sleep(0.1)          
        if not self.message_received:
            self.fail('No valid state message received within 60 seconds')

    # callback functions
    def cb_state(self, msg):
        self.actual_value = msg
        if (self.actual_value.linear.x > self.min_target_x) and (self.actual_value.linear.y > self.min_target_y):
            self.message_received = True


if __name__ == '__main__':
    try:
        rostest.run('rostest', 'teleop_test', UnitTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"

