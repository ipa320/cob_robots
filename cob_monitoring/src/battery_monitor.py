#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2012 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_robots
# \note
#   ROS package name: cob_monitoring
#
# \author
#   Author: Florian Weisshardt
# \author
#   Supervised by: 
#
# \date Date of creation: Dec 2012
#
# \brief
#   Monitors the battery level and announces warnings and reminders to recharge.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
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

import roslib
roslib.load_manifest('cob_monitoring')
import rospy

from cob_msgs.msg import *

from simple_script_server import *
sss = simple_script_server()

class battery_monitor():
	def __init__(self):
		rospy.Subscriber("/power_state", PowerState, self.power_state_callback)
		self.rate = rospy.Rate(1/10.0) # check every 10 sec
		self.warn_announce_time  = rospy.Duration(300.0)
		self.error_announce_time = rospy.Duration(120.0)
		self.last_announced_time = rospy.Time.now()

	## Battery monitoring
	### TODO: make values parametrized through yaml file (depending on env ROBOT)
	def power_state_callback(self,msg):
		if msg.relative_capacity <= 10.0:
			rospy.logerr("Battery empty, recharge now! Battery state is at " + str(msg.relative_capacity) + "%.")
			#TODO: print "start flashing red fast --> action call to lightmode"
			if rospy.Time.now() - self.last_announced_time >= self.error_announce_time:
				sss.say(["My battery is empty, please recharge now."])
				self.last_announced_time = rospy.Time.now()
		elif msg.relative_capacity <= 30.0:			
			rospy.logwarn("Battery nearly empty, consider recharging. Battery state is at " + str(msg.relative_capacity) + "%.") 
			#TODO: "start flashing yellow slowly --> action call to lightmode"
			if rospy.Time.now() - self.last_announced_time >= self.warn_announce_time:
				sss.say(["My battery is nearly empty, please consider recharging."])
				self.last_announced_time = rospy.Time.now()
		else:
			rospy.logdebug("Battery level ok.")	
		
		# sleep
		self.rate.sleep()

if __name__ == "__main__":
	rospy.init_node("battery_monitor")
	battery_monitor()
	rospy.spin()
