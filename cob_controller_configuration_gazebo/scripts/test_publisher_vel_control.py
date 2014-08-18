#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_controller_configuration_gazebo')

import rospy
import math
from brics_actuator.msg import JointVelocities, JointValue
 
def velPub():
  rospy.init_node("velPub", anonymous=True)
  
  component_name = "arm"
  component_dof = 7
  
  pub = rospy.Publisher("/"+component_name+"_controller/command_vel", JointVelocities, queue_size=1)
  vel =  JointVelocities()
  for x in range (0,component_dof):
    vel.velocities.append(JointValue())
  
  r = rospy.Rate(100)
  t0 = rospy.get_time()
  A = 0.1 #amplitude
  w = 1.0 #frequency
  phi = 0.0 #phase shift
  
  
  while not rospy.is_shutdown():
    t = rospy.get_time()
    vel.velocities[1].value=A*math.sin(w*(t-t0)+phi)
    
    #print vel
    pub.publish(vel)
    
    r.sleep()

if __name__ == '__main__':
  try:
      velPub()
  except rospy.ROSInterruptException: pass
