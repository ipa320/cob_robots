#!/usr/bin/python
import roslib
roslib.load_manifest('cob_hardware_test')
import rospy
import sys

from simple_script_server import *
sss = simple_script_server()

c1 = [["tray","up"],
     ["head","back"],
     ["sdh","cylopen"]]
c2 = [["tray","down"],
     ["head","front"],
     ["sdh","cylclosed"]]

def init(config_list):
	for config in config_list:
		sss.init(config[0])

def move_single_component(config):
	# recover
	sss.recover(config[0])
	
	# move
	handle = sss.move(config[0],config[1])
	
	# check result
	if handle.get_state() != 3:
		print "something went wrong"
		sss.set_light("red")
		#sys.exit()

def move_all_component(config_list):
	handles = []

	# recover
	for config in config_list:
		sss.recover(config[0])
	
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
			#sys.exit()

if __name__ == "__main__":
	rospy.init_node("life_test")

	init(c1)

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
		move_all_component(c2)
			

