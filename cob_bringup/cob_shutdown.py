#!/usr/bin/env python

import roslib
import pwd
roslib.load_manifest('cob_bringup')

import rospy

import commands
import os
import subprocess
import time


class shutdown_robot():
    
    def __init__(self):
        
        self.computers = rospy.get_param("computers")

        if (not rospy.has_param("username")):
                self.username = pwd.getpwuid(os.getuid())[0]
	else:
	        self.username = rospy.get_param("username")
        
    def shutdown(self):
        
        for pc in self.computers:
            
            command = subprocess.Popen(['ssh', "-t",  self.username+"@"+pc, 'sudo', 'shutdown', '-r', 'now'])
            time.sleep(5)
            msg = "Shutting down " + pc
	    print msg
            rospy.loginfo(msg)


if __name__ == '__main__':
    
    rospy.init_node("shutdown")
    
    shut_down = shutdown_robot()
    
    shut_down.shutdown()
