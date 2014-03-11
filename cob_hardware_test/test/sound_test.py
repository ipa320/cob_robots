#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_hardware_test')

import sys
import unittest
import rospy
import rostest
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
from simple_script_server import *
from dialog_client import dialog_client


class HardwareTest(unittest.TestCase):
    def __init__(self, *args):

        super(HardwareTest, self).__init__(*args)
        rospy.init_node('sound_test')
        self.message_received = False
        self.sss = simple_script_server()
        self.record = []

    def test_play(self):
        dialog_client(0, 'Listen up for the sound' )
        rospy.set_param("script_server/sound/language","de")
        handle = self.sss.play("grasp_tutorial_01")
        self.assertEqual(handle.get_state(), 3)
        self.assertTrue(dialog_client(1, 'Did you hear some audio file?'))
        
    def test_say(self):
        dialog_client(0, 'Listen up for the sound' )
        handle = self.sss.say(["Hello"])
        self.assertEqual(handle.get_state(), 3)
        if handle.get_error_code() != 0:
            error_msg = 'Could say something'
            self.fail(error_msg + "; errorCode: " + str(handle.get_error_code()))
        self.assertTrue(dialog_client(1, 'Did you hear <<Hello>>?'))
                
    def test_record(self):
        
        dialog_client(0, 'Prepare to Record')
        sub = rospy.Subscriber("/audio_in", AudioData, self.callback)
        pub = rospy.Publisher("/audio_out", AudioData)
        rospy.sleep(4.0)
        dialog_client(0, 'Done recording, listen up')        
        r = rospy.Rate(len(self.record)/4.0)
        
        for Data in self.record:
            pub.publish(Data)
            r.sleep()
        
        self.assertTrue(dialog_client(1, 'Did you hear your record?' ))
        
        
    def callback(self, Data):
        self.record.append(Data)        
        

    

if __name__ == '__main__':
    # Copied from hztest: A dirty hack to work around an apparent race condition at startup
    # that causes some hztests to fail. Most evident in the tests of
    # rosstage.
    time.sleep(0.75)

    try:
        rostest.run('rostest', 'sound_test', HardwareTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"
