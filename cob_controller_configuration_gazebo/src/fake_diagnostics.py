#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_controller_configuration_gazebo')

import sys
import rospy
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus

global last_received_

def callback(msg):
	global last_received_
	last_received_ = rospy.Time.now()

if __name__ == "__main__":
	rospy.init_node('fake_diagnostics')

	if not rospy.has_param("~diagnostics_name"):
		rospy.logerr("parameter diagnostics_name not found, shutting down " + rospy.get_name())
		sys.exit()
	diagnostics_name = rospy.get_param("~diagnostics_name")

	if not rospy.has_param("~topic_name"):
		rospy.logwarn("parameter topic_name not found. Not listening to any topic for " + rospy.get_name())
	topic_name = rospy.get_param("~topic_name",None)

	global last_received_
	last_received_ = rospy.Time.now()

	# subscribe to topics
	if topic_name != None:
		rospy.Subscriber(topic_name, rospy.AnyMsg, callback)

	pub_diagnostics = rospy.Publisher('/diagnostics', DiagnosticArray)

	rospy.loginfo("fake diagnostics for %s running listening to %s",diagnostics_name, topic_name)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		# if no topic_name is set, we assume that we received a 
		if topic_name == None:
			last_received_ = rospy.Time.now()

		# only publish ok if message received recently
		if rospy.Time.now() - last_received_ <= rospy.Duration(10.0):
			status = DiagnosticStatus()
			status.level = 0
			status.name = diagnostics_name
			status.message = diagnostics_name + " running"
			diagnostics = DiagnosticArray()
			diagnostics.status.append(status)
		else:
			status = DiagnosticStatus()
			status.level = 2
			status.name = diagnostics_name
			status.message = "no message received on " + topic_name
			diagnostics = DiagnosticArray()
			diagnostics.status.append(status)
		pub_diagnostics.publish(diagnostics)
		rate.sleep()
