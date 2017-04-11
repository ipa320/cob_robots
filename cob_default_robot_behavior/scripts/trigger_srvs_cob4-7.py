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
from cob_default_robot_behavior import default_behavior
from std_srvs.srv import Trigger, TriggerResponse

def trigger_srvs():
    rospy.init_node('trigger_srvs')
    s = rospy.Service('/behavior/setLightCyan',Trigger,default_behavior.setLightCyan_cb)
    s = rospy.Service('/behavior/setLightRed',Trigger,default_behavior.setLightRed_cb)
    s = rospy.Service('/behavior/setLightGreen',Trigger,default_behavior.setLightGreen_cb)
    s = rospy.Service('/behavior/setLightCyanSweep',Trigger,default_behavior.setLightCyanSweep_cb)
    s = rospy.Service('/behavior/setLightCyanBreath',Trigger,default_behavior.setLightCyanBreath_cb)
    s = rospy.Service('/behavior/setMimicLaughing',Trigger,default_behavior.setMimicLaughing_cb)
    s = rospy.Service('/behavior/setMimicAsking',Trigger,default_behavior.setMimicAsking_cb)
    s = rospy.Service('/behavior/setMimicYes',Trigger,default_behavior.setMimicYes_cb)
    s = rospy.Service('/behavior/setMimicBlinkingRight',Trigger,default_behavior.setMimicBlinkingRight_cb)
    s = rospy.Service('/behavior/setMimicConfused',Trigger,default_behavior.setMimicConfused_cb)
    s = rospy.Service('/behavior/setMimicAngry',Trigger,default_behavior.setMimicAngry_cb)
    s = rospy.Service('/behavior/setMimicFallingAsleep',Trigger,default_behavior.setMimicFallingAsleep_cb)
    s = rospy.Service('/behavior/soundHello',Trigger,default_behavior.soundHello_cb)

    rospy.spin()

if __name__ == "__main__":
    trigger_srvs()
