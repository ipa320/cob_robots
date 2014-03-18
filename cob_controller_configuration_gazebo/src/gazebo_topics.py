#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_controller_configuration_gazebo')

import rospy


# care-o-bot includes
from std_msgs.msg import Empty

class gazebo_topics():

	def __init__(self):
		#fake_diagnostics
		self.joy_usage_pub = rospy.Publisher("/joy_usage", Empty)
		self.pc1_usage_pub = rospy.Publisher("/pc1_usage", Empty)
		self.pc2_usage_pub = rospy.Publisher("/pc2_usage", Empty)
		self.pc3_usage_pub = rospy.Publisher("/pc3_usage", Empty)
		self.wifi_status_pub = rospy.Publisher("/wifi_status", Empty)
		
		rospy.sleep(0.5)


if __name__ == "__main__":
	rospy.init_node('gazebo_topics')
	gt = gazebo_topics()
	rospy.loginfo("gazebo_topics running")
	
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		msg = Empty()
		gt.joy_usage_pub.publish(msg)
		gt.pc1_usage_pub.publish(msg)
		gt.pc2_usage_pub.publish(msg)
		gt.pc3_usage_pub.publish(msg)
		gt.wifi_status_pub.publish(msg)
		rate.sleep()

