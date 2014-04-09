#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_controller_configuration_gazebo')

import rospy
from std_msgs.msg import Float64
from brics_actuator.msg import JointVelocities

class cob_control_topic_mapper():

  def __init__(self):
    self.joint_names = rospy.get_param("joint_names")
    
    self.vel_controller_pubs = []
    for i in range(len(self.joint_names)):
        pub = rospy.Publisher('/'+self.joint_names[i]+'_velocity_controller/command', Float64)
        self.vel_controller_pubs.append(pub)

    self.cmd_vel_sub = rospy.Subscriber("command_vel", JointVelocities, self.cmd_vel_cb)

    rospy.sleep(0.5)


  def cmd_vel_cb(self, data):
    #print data
    if(len(self.joint_names) != len(data.velocities)):
        rospy.logerr("DOF do not match")
        return
    
    for i in range(len(self.joint_names)):
        self.vel_controller_pubs[i].publish(Float64(data.velocities[i].value))



if __name__ == "__main__":
  rospy.init_node('cob_control_topic_mapper')
  cctm = cob_control_topic_mapper()
  rospy.loginfo("cob_control_topic_mapper running")
  rospy.spin()
