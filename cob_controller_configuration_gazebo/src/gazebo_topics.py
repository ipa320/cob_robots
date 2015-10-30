#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty

class gazebo_topics():

	def __init__(self):
		#fake_diagnostics
		self.joy_usage_pub = rospy.Publisher("joy_usage", Empty, queue_size=1)
		self.pc1_usage_pub = rospy.Publisher("pc1_usage", Empty, queue_size=1)
		self.pc2_usage_pub = rospy.Publisher("pc2_usage", Empty, queue_size=1)
		self.pc3_usage_pub = rospy.Publisher("pc3_usage", Empty, queue_size=1)
		self.b1_usage_pub = rospy.Publisher("b1_usage", Empty, queue_size=1)
		self.t1_usage_pub = rospy.Publisher("t1_usage", Empty, queue_size=1)
		self.t2_usage_pub = rospy.Publisher("t2_usage", Empty, queue_size=1)
		self.t3_usage_pub = rospy.Publisher("t3_usage", Empty, queue_size=1)
		self.wifi_status_pub = rospy.Publisher("wifi_status", Empty, queue_size=1)

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
		gt.b1_usage_pub.publish(msg)
		gt.t1_usage_pub.publish(msg)
		gt.t2_usage_pub.publish(msg)
		gt.t3_usage_pub.publish(msg)
		gt.wifi_status_pub.publish(msg)
		try:
			rate.sleep()
		except rospy.ROSInterruptException as e:
			#print "ROSInterruptException"
			pass

