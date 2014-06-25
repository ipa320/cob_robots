#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_controller_configuration_gazebo')

import rospy
import math
from brics_actuator.msg import JointVelocities, JointValue
 
def velPub():
  rospy.init_node("velPub",anonymous=True)
  
  pub = rospy.Publisher("/arm_controller/command_vel", JointVelocities, queue_size=10)
  vel =  JointVelocities()
  for x in range (0,7):
    vel.velocities.append(JointValue())
  
  r = rospy.Rate(100)
  ini_time = rospy.get_time()
  
  while not rospy.is_shutdown():
    #current_time = rospy.get_time() - ini_time
    #vel.velocities[1].value=math.sin(current_time/3)/20
    
    print vel
    pub.publish(vel)
    
    r.sleep()

if __name__ == '__main__':
  try:
      velPub()
  except rospy.ROSInterruptException: pass
