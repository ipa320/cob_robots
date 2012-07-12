#!/usr/bin/python
import roslib
roslib.load_manifest('cob_hardware_test')
import rospy
import sys

from simple_script_server import *
sss = simple_script_server()

c1 = [["torso","nod"],
     ["head","front"],
     ["sdh","cylclosed"]]
c2 = [["torso","shake"],
     ["head","back"],
     ["sdh","cylopen"]]


def move_single_component(config):
	handle = sss.move(config[0],config[1])
	if handle.get_state() != 3:
		print "something went wrong"
		sss.set_light("red")
		sys.exit()

def move_all_component(config_list):
	handles = []
	
	# move all components non-blocking
	for config in config_list:
		handles.append(sss.move(config[0],config[1],False))
		
	# wait for all components
	for handle in handles:
		handle.wait()
	
	# check result
	for handle in handles:
		if handle.get_state() != 3:
			print "something went wrong"
			sss.set_light("red")
			sys.exit()

if __name__ == "__main__":
	rospy.init_node("life_test")

	# prepare
	sss.move("torso","home")
	sss.move("head","front")
	sss.move("sdh","home")
	sss.move("arm","folded")
	
	# do life test
	while not rospy.is_shutdown():
		print "================================"
		print "=== moving single components ==="
		print "================================"
		sss.set_light("blue")
		for config in c1:
			move_single_component(config)
		for config in c2:
			move_single_component(config)
			
		print "*****************************"
		print "*** moving all components ***"
		print "*****************************"
		sss.set_light("yellow")
		move_all_component(c1)
			

