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

from cob_light.srv import *
from cob_light.msg import *

import rospy
from simple_script_server import *
sss = simple_script_server()
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse

def torso_front_cb(req):
    handle_arm_left = sss.move("arm_left","side", False)
    handle_arm_right = sss.move("arm_right","side", False)
    handle_arm_left.wait()
    handle_arm_right.wait()
    if handle_arm_left.get_error_code() == 0 and handle_arm_right.get_error_code() == 0:
        sss.move_base_rel("base",[0,0,1.57],False)
        sss.move("torso","front",True)
    else:
        return TriggerResponse(False, "Could not move arms.")

    return TriggerResponse(True, "")
    
def front_to_home_cb(req):
    handle_arm_left = sss.move("arm_left","side", False)
    handle_arm_right = sss.move("arm_right","side", False)
    handle_arm_left.wait()
    handle_arm_right.wait()
    if handle_arm_left.get_error_code() == 0 and handle_arm_right.get_error_code() == 0:
        sss.move_base_rel("base",[0,0,-1.57],False)
        sss.move("torso","home",False)
    else:
        return TriggerResponse(False, "Could not move arms.")

    return TriggerResponse(True, "")

def pick_cb(req):
    #sss.set_mimic("mimic",["asking",0,1])
    sss.set_mimic("mimic",["happy",0,1])
    handle_arm_left = sss.move("arm_left","side", False)
    handle_arm_right = sss.move("arm_right","side", False)
    #sss.set_mimic("mimic",["happy",0,1])
    handle_arm_left.wait()
    handle_arm_right.wait()
    if handle_arm_left.get_error_code() == 0 and handle_arm_right.get_error_code() == 0:
        sss.move("torso","front",False)
        sss.move_base_rel("base",[0,0,1.57],False)
        rospy.loginfo("------------------1 arm right movement")
        handle_arm_right = sss.move("arm_right",[[1.2, 0.85, -1.0499900779997886, -1.660000104864327, -1.0499900779997886, -0.6999817498048457, 1.0699740979351238]], False)
        sss.sleep(2)
        sss.move_base_rel("base",[0,0,0.5],False)
        rospy.loginfo("------------------2 arm right movement")
        handle_arm_right = sss.move("arm_right",[[2.929779495567761, 0.9417796643761402, -2.5, -1.5, -0.28560567879635207, -0.41160099749782275, 0.2752035164544659]], False)
        sss.sleep(2)
        handle_arm_left = sss.move("arm_left",[[-0.9599, -1.5708, 0.12, -1.0, -1.38, -0.75, 1.36]], False)
        sss.sleep(2)
        rospy.loginfo("torso movement")
        sss.move("torso",[[-3.14,2.7]],False)
        handle_arm_right = sss.move("arm_right",[[2.9300063883705207, 0.41079814604190534, -2.4999747139716377, -1.4999883190414864, -0.2899864552188579, -0.40999529458598793, 0.2800031718974503]], True)
        handle_arm_right = sss.move("arm_right",[[2.929988935078001, 0.9391791237906687, -2.940007124984448, -0.8784067592362261, -0.2899864552188579, -0.23357741379440114, 0.2800031718974503]], True)
        sss.sleep(3)
        #move right arm
        #open the gripper
        
        ##RETURN


        handle_arm_right = sss.move("arm_right",[[2.9300063883705207, 0.41079814604190534, -2.4999747139716377, -1.4999883190414864, -0.2899864552188579, -0.40999529458598793, 0.2800031718974503]],True)
        #sss.sleep(1)
        sss.move("torso","front",False)
        handle_arm_right = sss.move("arm_right",[[2.929779495567761, 0.9417796643761402, -2.5, -1.5, -0.28560567879635207, -0.41160099749782275, 0.2752035164544659]], False)
        sss.move_base_rel("base",[0,0,-0.3],False)
        sss.sleep(2)
        sss.move("torso","home",False)
        handle_arm_right = sss.move("arm_right",[[1.2, 0.85, -1.0499900779997886, -1.660000104864327, -1.0499900779997886, -0.6999817498048457, 1.0699740979351238]], False)
        handle_base = sss.move_base_rel("base",[0,0,-1.2],False)
        sss.sleep(2)
        handle_arm_left = sss.move("arm_left","side", False)
        handle_arm_right = sss.move("arm_right","side", False)
        sss.set_mimic("mimic",["blinking_right",0,1])
        handle_base.wait()
    else:
        return TriggerResponse(False, "Could not move arms.")

    return TriggerResponse(True, "")

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
        light_mode.color = cyan_color
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

def soundR2D2_cb(req):
    sss.play("R2D2")
    return TriggerResponse(True, "")

def showCamera_left_cb(req):
    sss.move("arm_left",[[-1.7642137145009082, -1.2919974320813223, 1.3, 1.8777473823431394, -0.0, -0.24, -0.0]])
    sss.move("gripper_left","open")
    return TriggerResponse(True, "")

def showCamera_right_cb(req):
    sss.move("arm_right",[[1.7642137145009082, 1.2919974320813223, -1.3, -1.8777473823431394, 0.0, 0.24, 0.0]])
    sss.move("gripper_right","open")
    return TriggerResponse(True, "")

def soundNoConnection_cb(req):
    sss.set_mimic("mimic",["confused",0,1])
    sss.play("confused")
    return TriggerResponse(True, "")

def soundNegative_cb(req):
    sss.play("negative")
    return TriggerResponse(True, "")

def soundStarting_cb(req):
    sss.play("starting")
    return TriggerResponse(True, "")

def soundHello_cb(req):
    sss.say("sound", ["Hello, my name is Care O bot, a mobile service robot from Fraunhofer I.P.A."])
    return TriggerResponse(True, "")

def trigger_srvs():
    rospy.init_node('trigger_srvs')
    s = rospy.Service('/behavior/torso_front', Trigger, torso_front_cb)
    s = rospy.Service('/behavior/torso_front_home', Trigger, front_to_home_cb)
    s = rospy.Service('/behavior/pick', Trigger, pick_cb)
    s = rospy.Service('/behavior/setLightCyan', Trigger, setLightCyan_cb)
    s = rospy.Service('/behavior/setLightRed', Trigger, setLightRed_cb)
    s = rospy.Service('/behavior/setLightGreen', Trigger, setLightGreen_cb)
    s = rospy.Service('/behavior/setLightCyanSweep', Trigger, setLightCyanSweep_cb)
    s = rospy.Service('/behavior/setLightCyanBreath', Trigger, setLightCyanBreath_cb)
    s = rospy.Service('/behavior/setMimicLaughing', Trigger, setMimicLaughing_cb)
    s = rospy.Service('/behavior/setMimicAsking', Trigger, setMimicAsking_cb)
    s = rospy.Service('/behavior/setMimicYes', Trigger, setMimicYes_cb)
    s = rospy.Service('/behavior/setMimicBlinkingRight', Trigger, setMimicBlinkingRight_cb)
    s = rospy.Service('/behavior/setMimicConfused', Trigger, setMimicConfused_cb)
    s = rospy.Service('/behavior/setMimicAngry', Trigger, setMimicAngry_cb)
    s = rospy.Service('/behavior/setMimicFallingAsleep', Trigger, setMimicFallingAsleep_cb)
    s = rospy.Service('/behavior/soundR2D2', Trigger, soundR2D2_cb)
    s = rospy.Service('/behavior/soundNoConnection',Trigger,soundNoConnection_cb)
    s = rospy.Service('/behavior/soundNegative',Trigger,soundNegative_cb)
    s = rospy.Service('/behavior/soundStarting',Trigger,soundStarting_cb)
    s = rospy.Service('/behavior/soundHello',Trigger,soundHello_cb)
    s = rospy.Service('/gripper_left/driver/showCamera_left',Trigger, showCamera_left_cb)
    s = rospy.Service('/gripper_right/driver/showCamera_right',Trigger, showCamera_right_cb)

    rospy.spin()

if __name__ == "__main__":
    trigger_srvs()
