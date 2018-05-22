#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
from cob_default_robot_behavior import default_behavior
from std_srvs.srv import Trigger, TriggerResponse

def trigger_srvs():
    rospy.init_node('trigger_srvs')
    s = rospy.Service('/behavior/setLightCyan',Trigger,default_behavior.setLightCyan_cb)
    s = rospy.Service('/behavior/setLightRed',Trigger,default_behavior.setLightRed_cb)
    s = rospy.Service('/behavior/setLightGreen',Trigger,default_behavior.setLightGreen_cb)
    s = rospy.Service('/behavior/setLightCyanSweep',Trigger,default_behavior.setLightCyanSweep_cb)
    s = rospy.Service('/behavior/setLightCyanBreath',Trigger,default_behavior.setLightCyanBreath_cb)

    s = rospy.Service('/behavior/setMimicAfraid',Trigger,default_behavior.setMimicAfraid_cb)
    s = rospy.Service('/behavior/setMimicAngry',Trigger,default_behavior.setMimicAngry_cb)
    s = rospy.Service('/behavior/setMimicAsking',Trigger,default_behavior.setMimicAsking_cb)
    s = rospy.Service('/behavior/setMimicBlinkingLeft',Trigger,default_behavior.setMimicBlinkingLeft_cb)
    s = rospy.Service('/behavior/setMimicBlinking',Trigger,default_behavior.setMimicBlinking_cb)
    s = rospy.Service('/behavior/setMimicBlinkingRight',Trigger,default_behavior.setMimicBlinkingRight_cb)
    s = rospy.Service('/behavior/setMimicBored',Trigger,default_behavior.setMimicBored_cb)
    s = rospy.Service('/behavior/setMimicBusy',Trigger,default_behavior.setMimicBusy_cb)
    s = rospy.Service('/behavior/setMimicConfused',Trigger,default_behavior.setMimicConfused_cb)
    s = rospy.Service('/behavior/setMimicDefault',Trigger,default_behavior.setMimicDefault_cb)
    s = rospy.Service('/behavior/setMimicFallingAsleep',Trigger,default_behavior.setMimicFallingAsleep_cb)
    s = rospy.Service('/behavior/setMimicHappy',Trigger,default_behavior.setMimicHappy_cb)
    s = rospy.Service('/behavior/setMimicLaughing',Trigger,default_behavior.setMimicLaughing_cb)
    s = rospy.Service('/behavior/setMimicNo',Trigger,default_behavior.setMimicNo_cb)
    s = rospy.Service('/behavior/setMimicSad',Trigger,default_behavior.setMimicSad_cb)
    s = rospy.Service('/behavior/setMimicSearching',Trigger,default_behavior.setMimicSearching_cb)
    s = rospy.Service('/behavior/setMimicSleeping',Trigger,default_behavior.setMimicSleeping_cb)
    s = rospy.Service('/behavior/setMimicSurprised',Trigger,default_behavior.setMimicSurprised_cb)
    s = rospy.Service('/behavior/setMimicTired',Trigger,default_behavior.setMimicTired_cb)
    s = rospy.Service('/behavior/setMimicWakingUp',Trigger,default_behavior.setMimicWakingUp_cb)
    s = rospy.Service('/behavior/setMimicYes',Trigger,default_behavior.setMimicYes_cb)

    s = rospy.Service('/behavior/soundHello',Trigger,default_behavior.soundHello_cb)

    rospy.spin()

if __name__ == "__main__":
    trigger_srvs()
