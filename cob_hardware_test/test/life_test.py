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
        if handle.get_error_code() > 0:
            sss.set_light("red")
            raise NameError('could not initialize ' + config[0])

def recover(config_list):
    for config in config_list:
        handle = sss.recover(config[0])
        if handle.get_error_code() > 0:
            sss.set_light("red")
            raise NameError('could not recover ' + config[0])

def move_single_component(config):
    # move
    handle = sss.move(config[0],config[1])

    # check result
    if handle.get_state() != 3:
        sss.set_light("red")
        raise NameError('something went wrong with ' + str(config[0]) + '. Action state = ' + str(handle.get_state()))
        #sys.exit()

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
            sss.set_light("red")
            raise NameError('something went wrong in move all. Action state = ' + str(handle.get_state()))
            #sys.exit()

if __name__ == "__main__":
    rospy.init_node("life_test")

    init(c1)

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
            for config in c1:
                move_single_component(config)
            for config in c2:
                move_single_component(config)
            sss.set_light("green")

            print "*****************************"
            print "*** moving all components ***"
            print "*****************************"
            sss.set_light("yellow")
            move_all_component(c1)
            move_all_component(c2)
            sss.set_light("green")
        except NameError, e:
            print "Error: %s"%e
            print "Successfull round(s): <<" + str(counter) + ">>"
            print "press <return> to continue"
            sss.wait_for_input()
            counter = 0
            recover(c1)
