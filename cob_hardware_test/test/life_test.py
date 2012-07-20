#!/usr/bin/python
import roslib
roslib.load_manifest('cob_hardware_test')
import rospy
import sys

from simple_script_server import *
sss = simple_script_server()

c1 = [["torso","shake"],
     ["tray","up"],
     ["arm","pregrasp"],
     ["sdh","cylopen"],
     ["head","back"]]

# c2 should be "navigationable" configuration
c2 = [["torso","nod"],
     ["tray","down"],
     ["arm","folded"],
     ["sdh","cylclosed"],
     ["head","front"]]

def init(config_list):
	for config in config_list:
		handle = sss.init(config[0])
		if handle.get_error_code() < 0:
			sss.set_light("red")
			raise NameError('could not initialize ' + config[0])

def move_single_component(config):
	# recover
	sss.recover(config[0])
	
	# move
	handle = sss.move(config[0],config[1])
	
	# check result
	if handle.get_state() != 3:
		sss.set_light("red")
		raise NameError('something went wrong')
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
			sss.set_light("red")
			raise NameError('something went wrong')
			#sys.exit()

if __name__ == "__main__":
	rospy.init_node("life_test")

	init(c1)
	sss.init("base")

	# do life test
	counter = 0
	while not rospy.is_shutdown():
		counter += 1
		try:
			sss.say(["This is round " + str(counter)])
			print "================================"
			print "=== moving single components ==="
			print "================================"
			sss.set_light("yellow")
			sss.recover("base")
			sss.move("base",[1,0,0])
			sss.set_light("green")

			for config in c1:
				move_single_component(config)
			for config in c2:
				move_single_component(config)

			print "*****************************"
			print "*** moving all components ***"
			print "*****************************"
			sss.set_light("yellow")
			sss.recover("base")
			sss.move("base",[0,0,0])
			sss.set_light("green")

			move_all_component(c1)
			move_all_component(c2)
		except NameError:
			print "Error after " + str(counter) + " successfull round(s): press <return> to continue"
			sss.wait_for_input()
			counter = 0
