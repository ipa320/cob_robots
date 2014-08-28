#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_controller_configuration_gazebo')

import rospy
from std_msgs.msg import Float64
from brics_actuator.msg import JointVelocities, JointPositions
from controller_manager_msgs.srv import *
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult

class cob_controller_adapter_gazebo():

  def __init__(self):
    self.joint_names = rospy.get_param("joint_names", [])
        
    #self.pos_controller_pubs = []
    #self.pos_controller_names = []
    
    #self.trajectory_controller_names = []
    #self.trajectory_controller_names.append(rospy.get_namespace()[1:-1])    
    
    self.vel_controller_pubs = []
    self.vel_controller_names = []
    
    self.current_controller_names = []
    

    #for i in range(len(self.joint_names)):
        #pub = rospy.Publisher('/'+self.joint_names[i]+'_position_controller/command', Float64, queue_size=1)
        #self.pos_controller_pubs.append(pub)
        #self.pos_controller_names.append(self.joint_names[i]+'_position_controller')
        
    for i in range(len(self.joint_names)):
        pub = rospy.Publisher('/'+self.joint_names[i]+'_velocity_controller/command', Float64, queue_size=1)
        self.vel_controller_pubs.append(pub)
        self.vel_controller_names.append(self.joint_names[i]+'_velocity_controller')
        
    
    rospy.logwarn("Waiting for load_controller service...")
    rospy.wait_for_service('/controller_manager/load_controller')
    rospy.loginfo("...load_controller service available!")
    
    self.load_client = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)

    rospy.logwarn("Waiting for switch_controller service...")
    rospy.wait_for_service('/controller_manager/switch_controller')
    rospy.loginfo("...switch_controller service available!")
    
    self.switch_client = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    
    #for controller in self.pos_controller_names:
        #res = self.load_client(controller)
    #for controller in self.trajectory_controller_names:
        #res = self.load_client(controller)
    for controller in self.vel_controller_names:
        res = self.load_client(controller)    
    

    ###Initial control mode depends on which controller are started loaded here
    #self.switch_controller(self.pos_controller_names, self.current_controller_names)
    #self.current_control_mode = "position"
    #self.switch_controller(self.trajectory_controller_names, self.current_controller_names)
    #self.current_control_mode = "trajectory"
    self.switch_controller(self.vel_controller_names, self.current_controller_names)
    self.current_control_mode = "velocity"

    
    self.update_rate = rospy.get_param("update_rate", 68.0)
    self.max_trajectory_command_silence = rospy.get_param("max_trajectory_command_silence", 0.5)
    self.max_vel_command_silence = rospy.get_param("max_vel_command_silence", 0.5)
    self.last_trajectory_command = rospy.get_time()
    self.last_vel_command = rospy.get_time()
    self.finished_trajectory=True
    
    #self.cmd_pos_sub = rospy.Subscriber("command_pos", JointPositions, self.cmd_pos_cb)
    self.cmd_vel_sub = rospy.Subscriber("command_vel", JointVelocities, self.cmd_vel_cb)
    #self.cmd_result_sub = rospy.Subscriber("follow_joint_trajectory/result", FollowJointTrajectoryActionResult, self.cmd_result_cb)
    #self.cmd_goal_sub = rospy.Subscriber("follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, self.cmd_goal_cb)
    
    rospy.sleep(0.5)


  def run(self):
    r = rospy.Rate(self.update_rate)
    while not rospy.is_shutdown():
        #if (((rospy.get_time() - self.last_vel_command) >= self.max_vel_command_silence) and (self.current_control_mode != "trajectory")):            
            #rospy.loginfo("Have not heard a vel command for %f seconds, switching to trajectory_controllers", (rospy.get_time()-self.last_vel_command))
            #self.switch_controller(self.trajectory_controller_names, self.current_controller_names)
            #self.current_control_mode = "trajectory"
            #print("Current control mode: "+self.current_control_mode)

        r.sleep()
    
  def switch_controller(self, start_controllers, stop_controllers):
    rospy.loginfo("Switching controllers")
    
    req = SwitchControllerRequest()
    req.strictness = 1
    for i in range(len(stop_controllers)):
        req.stop_controllers.append(stop_controllers[i])
    for i in range(len(start_controllers)):
        req.start_controllers.append(start_controllers[i])
    
    try:
        res = self.switch_client(req)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s", e)
        
    print ("Switched from: "+str(stop_controllers)+" to:"+str(start_controllers))
    self.current_controller_names = start_controllers
    print res    
    return res.ok   
    

  def cmd_vel_cb(self, data):
    if(len(self.joint_names) != len(data.velocities)):
      rospy.logerr("DOF do not match")
      return
    if not self.finished_trajectory:
      rospy.logwarn("Can't switch to velocity controller, haven't finished trajectory!")
      return
    self.last_vel_command = rospy.get_time()
    if self.current_control_mode != "velocity":
      rospy.logwarn("Have to switch to velocity_controllers")
      self.switch_controller(self.vel_controller_names, self.current_controller_names)
      self.current_control_mode = "velocity"
      print("Current control mode: "+self.current_control_mode)
      #print data    
    for i in range(len(self.joint_names)):
      self.vel_controller_pubs[i].publish(Float64(data.velocities[i].value))
    
    
  #def cmd_pos_cb(self, data):
    #if (self.current_control_mode != "trajectory"):
      #if (self.current_control_mode != "position"):
          #rospy.logwarn("Have to switch to position_controllers")
          #self.switch_controller(self.pos_controller_names, self.current_controller_names)
          #self.current_control_mode = "position"
      ##print data
      #if(len(self.joint_names) != len(data.positions)):
          #rospy.logerr("DOF do not match")
          #return
      #for i in range(len(self.joint_names)):
          #self.pos_controller_pubs[i].publish(Float64(data.positions[i].value))

  #def cmd_result_cb(self, data):    
    #self.last_trajectory_command = rospy.get_time()    
    #self.finished_trajectory=True
        
  #def cmd_goal_cb(self, data):    
    #self.last_trajectory_command = rospy.get_time()
    #if (((rospy.get_time() - self.last_vel_command) >= self.max_vel_command_silence) and self.current_control_mode != "trajectory"):
      #rospy.logwarn("Can't switch to trajectory controller because I'm receiving velocities!")
      #print(self.current_control_mode)
      #return
    #self.finished_trajectory=False
    #if (self.current_control_mode != "trajectory"):
        #rospy.logwarn("Have to switch to joint_trajectory_controllers")
        #self.switch_controller(self.trajectory_controller_names, self.current_controller_names)
        #self.current_control_mode = "trajectory"
        #print("Current control mode:"+self.current_control_mode)
    ##print data



if __name__ == "__main__":
  rospy.init_node('cob_controller_adapter_gazebo_node')
  cctm = cob_controller_adapter_gazebo()
  rospy.loginfo("cob_controller_adapter_gazebo running")
  cctm.run()
