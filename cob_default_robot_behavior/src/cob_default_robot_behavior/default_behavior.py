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
from simple_script_server import *
sss = simple_script_server()
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse
from cob_light.srv import *
from cob_light.msg import *


def setLightCyan_cb(req):
    sss.set_light("light_base","cyan")
    sss.set_light("light_torso","cyan")
    return TriggerResponse(True, "")
    
def setLightRed_cb(req):
    sss.set_light("light_base","red")
    sss.set_light("light_torso","red")
    return TriggerResponse(True, "")
    
def setLightGreen_cb(req):
    sss.set_light("light_base","green")
    sss.set_light("light_torso","green")
    return TriggerResponse(True, "")
    
def setLightCyanSweep_cb(req):
    #rospy.wait_for_service('/light_torso/set_light')
    sss.set_light("light_base","cyan")

    try:
        set_light_torso = rospy.ServiceProxy("/light_torso/set_light",SetLightMode)
        light_mode = LightMode()
        cyan_color = ColorRGBA()
        cyan_color.r = 0.0
        cyan_color.g = 1.0
        cyan_color.b = 0.5
        cyan_color.a = 0.4
        light_mode.colors.append(cyan_color)
        light_mode.mode = 8
        light_mode.frequency = 30
        resp = set_light_torso(light_mode)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return TriggerResponse(False, "Calling light service failed.")
    return TriggerResponse(True, "")

def setLightCyanBreath_cb(req):
    #rospy.wait_for_service('/light_torso/set_light')
    sss.set_light("light_base","cyan")

    try:
        set_light_torso = rospy.ServiceProxy("/light_torso/set_light",SetLightMode)
        light_mode = LightMode()
        cyan_color = ColorRGBA()
        cyan_color.r = 0.0
        cyan_color.g = 1.0
        cyan_color.b = 0.5
        cyan_color.a = 0.4
        light_mode.colors.append(cyan_color)
        light_mode.mode = 3
        light_mode.frequency = 0.25
        resp = set_light_torso(light_mode)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return TriggerResponse(False, "Calling light service failed.")
    return TriggerResponse(True, "")


def setMimicAfraid_cb(req):
    sss.set_mimic("mimic",["afraid",0,1])
    return TriggerResponse(True, "")

def setMimicAngry_cb(req):
    sss.set_mimic("mimic",["angry",0,1])
    return TriggerResponse(True, "")

def setMimicAsking_cb(req):
    sss.set_mimic("mimic",["asking",0,1])
    return TriggerResponse(True, "")

def setMimicBlinkingLeft_cb(req):
    sss.set_mimic("mimic",["blinking_left",0,1])
    return TriggerResponse(True, "")

def setMimicBlinking_cb(req):
    sss.set_mimic("mimic",["blinking",0,1])
    return TriggerResponse(True, "")

def setMimicBlinkingRight_cb(req):
    sss.set_mimic("mimic",["blinking_right",0,1])
    return TriggerResponse(True, "")

def setMimicBored_cb(req):
    sss.set_mimic("mimic",["bored",0,1])
    return TriggerResponse(True, "")

def setMimicBusy_cb(req):
    sss.set_mimic("mimic",["busy",0,1])
    return TriggerResponse(True, "")

def setMimicConfused_cb(req):
    sss.set_mimic("mimic",["confused",0,1])
    return TriggerResponse(True, "")

def setMimicDefault_cb(req):
    sss.set_mimic("mimic",["default",0,1])
    return TriggerResponse(True, "")

def setMimicFallingAsleep_cb(req):
    sss.set_mimic("mimic",["falling_asleep",0,1])
    return TriggerResponse(True, "")

def setMimicHappy_cb(req):
    sss.set_mimic("mimic",["happy",0,1])
    return TriggerResponse(True, "")

def setMimicLaughing_cb(req):
    sss.set_mimic("mimic",["laughing",0,1])
    return TriggerResponse(True, "")

def setMimicNo_cb(req):
    sss.set_mimic("mimic",["no",0,1])
    return TriggerResponse(True, "")

def setMimicSad_cb(req):
    sss.set_mimic("mimic",["sad",0,1])
    return TriggerResponse(True, "")

def setMimicSearching_cb(req):
    sss.set_mimic("mimic",["searching",0,1])
    return TriggerResponse(True, "")

def setMimicSleeping_cb(req):
    sss.set_mimic("mimic",["sleeping",0,1])
    return TriggerResponse(True, "")

def setMimicSurprised_cb(req):
    sss.set_mimic("mimic",["surprised",0,1])
    return TriggerResponse(True, "")

def setMimicTired_cb(req):
    sss.set_mimic("mimic",["tired",0,1])
    return TriggerResponse(True, "")

def setMimicWakingUp_cb(req):
    sss.set_mimic("mimic",["waking_up",0,1])
    return TriggerResponse(True, "")

def setMimicYes_cb(req):
    sss.set_mimic("mimic",["yes",0,1])
    return TriggerResponse(True, "")

def soundHello_cb(req):
    sss.say("sound", ["Hello, my name is Care O bot, a mobile service robot from Mojin Robotics."])
    return TriggerResponse(True, "")
