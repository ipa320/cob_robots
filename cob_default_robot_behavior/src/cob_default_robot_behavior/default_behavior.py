#!/usr/bin/python
#################################################################
##\file
#
# \note
#    Copyright (c) 2015 \n
#    Fraunhofer Institute for Manufacturing Engineering
#    and Automation (IPA) \n\n
#
#################################################################
#
# \note
#    Project name: care-o-bot
# \note
#    ROS stack name: cob_robots
# \note
#    ROS package name: cob_default_robot_behavior
#
# \author
#    Author: Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fhg.de
# \author
#    Supervised by: Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fhg.de
#
# \date Date of creation: Aug 2015
#
# \brief
#    Implements script server functionalities.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#       - Redistributions of source code must retain the above copyright
#           notice, this list of conditions and the following disclaimer. \n
#       - Redistributions in binary form must reproduce the above copyright
#           notice, this list of conditions and the following disclaimer in the
#           documentation and/or other materials provided with the distribution. \n
#       - Neither the name of the Fraunhofer Institute for Manufacturing
#           Engineering and Automation (IPA) nor the names of its
#           contributors may be used to endorse or promote products derived from
#           this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################


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


def setMimicLaughing_cb(req):
    sss.set_mimic("mimic",["laughing",0,1])
    return TriggerResponse(True, "")
    
def setMimicAsking_cb(req):
    sss.set_mimic("mimic",["asking",0,1])
    return TriggerResponse(True, "")
    
def setMimicYes_cb(req):
    sss.set_mimic("mimic",["yes",0,1])
    return TriggerResponse(True, "")
    
def setMimicBlinkingRight_cb(req):
    sss.set_mimic("mimic",["blinking_right",0,1])
    return TriggerResponse(True, "")

def setMimicConfused_cb(req):
    sss.set_mimic("mimic",["confused",0,1])
    return TriggerResponse(True, "")

def setMimicAngry_cb(req):
    sss.set_mimic("mimic",["angry",0,1])
    return TriggerResponse(True, "")

def setMimicFallingAsleep_cb(req):
    sss.set_mimic("mimic",["falling_asleep",0,1])
    return TriggerResponse(True, "")

def soundHello_cb(req):
    sss.say("sound", ["Hello, my name is Care O bot, a mobile service robot from Fraunhofer I.P.A."])
    return TriggerResponse(True, "")
