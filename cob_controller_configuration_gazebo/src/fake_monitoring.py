#!/usr/bin/env python

import traceback
import threading
from threading import Timer
import sys, os, time
from time import sleep
import subprocess
import string
import socket

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class FakeMonitor():
    def __init__(self):
        self._fake_diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=50)
        self._last_publish_time = 0


    def publish_stats(self, diag_hostnames):
        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()
        # Add all fake
        hostname_list = diag_hostnames.split(", ")
        print "hostnames: "+str(len(hostname_list))
        for hostname in hostname_list:
            status = DiagnosticStatus()
            status.name = hostname
            status.level = 0
            msg.status.append(status)
            print hostname

        self._fake_diag_pub.publish(msg)
        print "fake diagnostics published!"


if __name__ == '__main__':

    import optparse
    parser = optparse.OptionParser(usage="usage: fake_monitoring.py [--diag-hostnames=hostname1, hostname2, ...]")
    parser.add_option("--diag-hostnames", dest="diag_hostnames",
                      help="Fake monitor diagnostics")
    options, args = parser.parse_args(rospy.myargv())

    try:
        rospy.init_node('fake_monitor')
    except rospy.exceptions.ROSInitException:
        print >> sys.stderr, 'Fake monitor is unable to initialize node. Master may not be running.'
        sys.exit(0)

    fake_node = FakeMonitor()

    rate = rospy.Rate(1.0)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
            fake_node.publish_stats(options.diag_hostnames)
    except KeyboardInterrupt:
        pass
    except Exception, e:
        traceback.print_exc()
        rospy.logerr(traceback.format_exc())

    sys.exit(0)
