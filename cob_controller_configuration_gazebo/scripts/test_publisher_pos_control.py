#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_controller_configuration_gazebo')

import rospy
import math
from brics_actuator.msg import JointPositions, JointValue
 
def posPub():
  rospy.init_node("posPub", anonymous=True)
  
  component_name = "arm"
  component_dof = 7
  
  pub = rospy.Publisher("/"+component_name+"_controller/command_pos", JointPositions, queue_size=1)
  pos =  JointPositions()
  for x in range (0,component_dof):
    pos.positions.append(JointValue())
  
  r = rospy.Rate(100)
  ini_time = rospy.get_time()
  
  while not rospy.is_shutdown():
    #current_time = rospy.get_time() - ini_time
    #pos.positions[1].value=math.sin(current_time/3)/20
    
    #print pos
    pub.publish(pos)
    
    r.sleep()

if __name__ == '__main__':
  try:
      posPub()
  except rospy.ROSInterruptException: pass
