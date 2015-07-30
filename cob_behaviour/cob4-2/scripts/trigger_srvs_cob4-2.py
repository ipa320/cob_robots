#!/usr/bin/python
#################################################################
##\file
#
# \note
#	 Copyright (c) 2015 \n
#	 Fraunhofer Institute for Manufacturing Engineering
#	 and Automation (IPA) \n\n
#
#################################################################
#
# \note
#	 Project name: care-o-bot
# \note
#	 ROS stack name: cob_robots
# \note
#	 ROS package name: cob_behaviour
#
# \author
#	 Author: Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fhg.de
# \author
#	 Supervised by: Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fhg.de
#
# \date Date of creation: Aug 2015
#
# \brief
#	 Implements script server functionalities.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#		 - Redistributions of source code must retain the above copyright
#			 notice, this list of conditions and the following disclaimer. \n
#		 - Redistributions in binary form must reproduce the above copyright
#			 notice, this list of conditions and the following disclaimer in the
#			 documentation and/or other materials provided with the distribution. \n
#		 - Neither the name of the Fraunhofer Institute for Manufacturing
#			 Engineering and Automation (IPA) nor the names of its
#			 contributors may be used to endorse or promote products derived from
#			 this software without specific prior written permission. \n
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

from std_srvs.srv import Trigger
import rospy
from simple_script_server import *
sss = simple_script_server()
from std_msgs.msg import ColorRGBA
from cob_light.srv import *
from cob_light.msg import *
from cob_mimic.msg import *
from cob_mimic.srv import *

def bow_cb(req):
    handle_arm_left = sss.move("arm_left","side", False)
    handle_arm_right = sss.move("arm_right","side", False)
    handle_arm_left.wait()
    handle_arm_right.wait()
    if handle_arm_left.get_error_code() == 0 and handle_arm_right.get_error_code() == 0:
        sss.move("torso","front")
        sss.sleep(3)
        sss.move("torso","home")
    return

def setLightCyan_cb(req):
    sss.set_light("light_base","cyan")
    sss.set_light("light_torso","cyan")
    return
    
def setLightRed_cb(req):
    sss.set_light("light_base","red")
    sss.set_light("light_torso","red")
    return
    
def setLightGreen_cb(req):
    sss.set_light("light_base","green")
    sss.set_light("light_torso","green")
    return
    
def setLightCyanSweep_cb(req):
    #rospy.wait_for_service('/light_torso/set_light')
    sss.set_light("light_base","cyan")

    try:
      set_light_torso = rospy.ServiceProxy("/light_torso/mode",SetLightMode)
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
      return
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def setLightCyanBreath_cb(req):
    #rospy.wait_for_service('/light_torso/set_light')
    sss.set_light("light_base","cyan")

    try:
      set_light_torso = rospy.ServiceProxy("/light_torso/mode",SetLightMode)
      light_mode = LightMode()
      cyan_color = ColorRGBA()
      cyan_color.r = 0.0
      cyan_color.g = 1.0
      cyan_color.b = 0.5
      cyan_color.a = 0.4
      light_mode.color = cyan_color
      light_mode.mode = 3
      light_mode.frequency = 0.25
      resp = set_light_torso(light_mode)
      print resp
      return
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def setMimicLaughing_cb(req):
    sss.set_mimic("mimic",["laughing",0,1])
    return
    
def setMimicAsking_cb(req):
    sss.set_mimic("mimic",["asking",0,1])
    return
    
def setMimicYes_cb(req):
    sss.set_mimic("mimic",["yes",0,1])
    return
    
def setMimicBlinkingRight_cb(req):
    sss.set_mimic("mimic",["blinking_right",0,1])
    return

def setMimicConfused_cb(req):
    sss.set_mimic("mimic",["confused",0,1])
    return

def setMimicAngry_cb(req):
    sss.set_mimic("mimic",["angry",0,1])
    return

def setMimicFallingAsleep_cb(req):
    sss.set_mimic("mimic",["falling_asleep",0,1])
    return

def playSound_cb(req):
    os.system("aplay -q /u/behaviour/sounds/R2D2.wav")
    return

def showCamera_left_cb(req):
    sss.move("arm_left",[[-1.7642137145009082, -1.2919974320813223, 1.3, 1.8777473823431394, -0.0, -0.24, -0.0]])
    sss.move("gripper_left","open")          
    return

def showCamera_right_cb(req):
    sss.move("arm_right",[[1.7642137145009082, 1.2919974320813223, -1.3, -1.8777473823431394, 0.0, 0.24, 0.0]])
    sss.move("gripper_right","open")
    return

def soundNoConnection_cb(req):
    sss.set_mimic("mimic",["confused",0,1])
    os.system("aplay -q /u/behaviour/sounds/confused.wav")
    return

def soundNegative_cb(req):
    os.system("aplay -q /u/behaviour/sounds/negative.wav")
    return

def soundStarting_cb(req):
    os.system("aplay -q /u/behaviour/sounds/starting.wav")
    return

def soundHello_cb(req):
    sss.say(["Hello, my name is Care O bot, a mobile service robot from Fraunhofer I.P.A."])
    sss.say(["I am at your command."])
    return

def trigger_srvs():
    rospy.init_node('trigger_srvs')
    s = rospy.Service('/behaviour/bow', Trigger, bow_cb)
    s = rospy.Service('/behaviour/setLightCyan', Trigger, setLightCyan_cb)
    s = rospy.Service('/behaviour/setLightRed', Trigger, setLightRed_cb)
    s = rospy.Service('/behaviour/setLightGreen', Trigger, setLightGreen_cb)
    s = rospy.Service('/behaviour/setLightCyanSweep', Trigger, setLightCyanSweep_cb)
    s = rospy.Service('/behaviour/setLightCyanBreath', Trigger, setLightCyanBreath_cb)
    s = rospy.Service('/behaviour/setMimicLaughing', Trigger, setMimicLaughing_cb)
    s = rospy.Service('/behaviour/setMimicAsking', Trigger, setMimicAsking_cb)
    s = rospy.Service('/behaviour/setMimicYes', Trigger, setMimicYes_cb)
    s = rospy.Service('/behaviour/setMimicBlinkingRight', Trigger, setMimicBlinkingRight_cb)
    s = rospy.Service('/behaviour/setMimicConfused', Trigger, setMimicConfused_cb)
    s = rospy.Service('/behaviour/setMimicAngry', Trigger, setMimicAngry_cb)
    s = rospy.Service('/behaviour/setMimicFallingAsleep', Trigger, setMimicFallingAsleep_cb)
    s = rospy.Service('/behaviour/playSound', Trigger, playSound_cb)
    s = rospy.Service('/behaviour/soundNoConnection',Trigger,soundNoConnection_cb)
    s = rospy.Service('/behaviour/soundNegative',Trigger,soundNegative_cb)
    s = rospy.Service('/behaviour/soundStarting',Trigger,soundStarting_cb)
    s = rospy.Service('/behaviour/soundHello',Trigger,soundHello_cb)

    s = rospy.Service('/gripper_left/driver/showCamera_left',Trigger, showCamera_left_cb)
    s = rospy.Service('/gripper_right/driver/showCamera_right',Trigger, showCamera_right_cb)
    print "Ready"
    rospy.spin()

if __name__ == "__main__":
    trigger_srvs()
