#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_controller_configuration_gazebo')

import rospy
from std_msgs.msg import Float64
from brics_actuator.msg import JointVelocities, JointPositions
from controller_manager_msgs.srv import *

class cob_controller_adapter_gazebo():

  def __init__(self):
    self.joint_names = rospy.get_param("joint_names", [])
    
    self.vel_controller_pubs = []
    self.vel_controller_names = []
    self.pos_controller_pubs = []
    self.pos_controller_names = []
    
    for i in range(len(self.joint_names)):
        pub = rospy.Publisher('/'+self.joint_names[i]+'_velocity_controller/command', Float64, queue_size=1)
        self.vel_controller_pubs.append(pub)
        self.vel_controller_names.append(self.joint_names[i]+'_velocity_controller')
    for i in range(len(self.joint_names)):
        pub = rospy.Publisher('/'+self.joint_names[i]+'_position_controller/command', Float64, queue_size=1)
        self.pos_controller_pubs.append(pub)
        self.pos_controller_names.append(self.joint_names[i]+'_position_controller')
    
    rospy.logwarn("Waiting for load_controller service...")
    rospy.wait_for_service('/controller_manager/load_controller')
    rospy.loginfo("...load_controller service available!")
    
    self.load_client = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)

    rospy.logwarn("Waiting for switch_controller service...")
    rospy.wait_for_service('/controller_manager/switch_controller')
    rospy.loginfo("...switch_controller service available!")
    
    self.switch_client = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    
    for controller in self.vel_controller_names:
        res = self.load_client(controller)
    for controller in self.pos_controller_names:
        res = self.load_client(controller)
    
    self.switch_controller(self.pos_controller_names, [])
    self.current_control_mode = "position"
    
    self.update_rate = rospy.get_param("update_rate", 33.0)
    self.max_vel_command_silence = rospy.get_param("max_vel_command_silence", 0.5)
    self.last_vel_command = rospy.get_time()
    
    self.cmd_vel_sub = rospy.Subscriber("command_vel", JointVelocities, self.cmd_vel_cb)
    self.cmd_pos_sub = rospy.Subscriber("command_pos", JointPositions, self.cmd_pos_cb)

    rospy.sleep(0.5)


  def run(self):
    r = rospy.Rate(self.update_rate)
    while not rospy.is_shutdown():
        if (rospy.get_time() - self.last_vel_command >= self.max_vel_command_silence) and (self.current_control_mode != "position"):
            rospy.loginfo("Have not heard a vel command for %f seconds. Switch to position_controllers", (rospy.get_time()-self.last_vel_command))
            self.switch_controller(self.pos_controller_names, self.vel_controller_names)
            self.current_control_mode = "position"
        r.sleep()
  
  
  def switch_controller(self, start_controllers, stop_controllers):
    rospy.loginfo("Switching controllers")
    
    req = SwitchControllerRequest()
    req.strictness = 2
    for i in range(len(start_controllers)):
        req.start_controllers.append(start_controllers[i])
    for i in range(len(stop_controllers)):
        req.stop_controllers.append(stop_controllers[i])
    
    try:
        res = self.switch_client(req)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s", e)
    
    print res
    return res.ok
    
    

  def cmd_vel_cb(self, data):
    if (self.current_control_mode != "velocity"):
        rospy.logwarn("Have to switch to velocity_controllers")
        self.switch_controller(self.vel_controller_names, self.pos_controller_names)
        self.current_control_mode = "velocity"
    self.last_vel_command = rospy.get_time()
    #print data
    if(len(self.joint_names) != len(data.velocities)):
        rospy.logerr("DOF do not match")
        return
    for i in range(len(self.joint_names)):
        self.vel_controller_pubs[i].publish(Float64(data.velocities[i].value))
    
    
  def cmd_pos_cb(self, data):
    if (self.current_control_mode != "position"):
        rospy.logwarn("Have to switch to position_controllers")
        self.switch_controller(self.pos_controller_names, self.vel_controller_names)
        self.current_control_mode = "position"
    #print data
    if(len(self.joint_names) != len(data.positions)):
        rospy.logerr("DOF do not match")
        return
    for i in range(len(self.joint_names)):
        self.pos_controller_pubs[i].publish(Float64(data.positions[i].value))




if __name__ == "__main__":
  rospy.init_node('cob_controller_adapter_gazebo_node')
  cctm = cob_controller_adapter_gazebo()
  rospy.loginfo("cob_controller_adapter_gazebo running")
  cctm.run()
