#!/usr/bin/env python

import roslib
roslib.load_manifest('cob_hardware_test')
import sys
import rospy
import wx
from cob_hardware_test.srv import *

def handle_dialog(req):
    if req.type == 1:
            print "Asking: %s" % (req.message)
            ex = wx.App()
            dial = wx.MessageDialog(None, req.message, 'Question',
                            wx.YES_NO | wx.ICON_QUESTION)
            ret = dial.ShowModal()
            if ret == wx.ID_YES:
                    answer = True
            else:
                    answer = False

        #TODO exit properly
        #self.Destroy()
        #ex.Destroy()        
        #dial.Destroy()
            return DialogResponse(answer)

    if req.type == 0:
            print "Confirm: %s" % (req.message)
            ex = wx.App()
            dial = wx.MessageDialog(None, req.message, 'Confirm',
                            wx.OK | wx.ICON_WARNING)
            ret = dial.ShowModal()
            if ret == wx.ID_OK:
                    answer = True
            else:
                    answer = False

        #TODO exit properly
        #self.Destroy()
        #ex.Destroy()        
        #dial.Destroy()
            return DialogResponse(answer)
    


def dialog_server():
        rospy.init_node('dialog_server')
        s = rospy.Service('dialog', Dialog, handle_dialog)
        print "Ready to ask(1) or confirm(0)!"
        rospy.spin()



if __name__ == "__main__":
      dialog_server()
