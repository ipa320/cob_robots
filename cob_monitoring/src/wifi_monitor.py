#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\author Kevin Watts
##\brief Republishes the data from ddwrt/accesspoint onto diagnostics
# This file has been copied from https://github.com/PR2/pr2_computer_monitor in order to support this feature for indigo indepenendly from PR2 dependencies

from __future__ import with_statement

PKG = 'cob_monitoring'
import roslib
roslib.load_manifest(PKG)

import rospy

import threading
import sys

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from cob_msgs.msg import AccessPoint

DIAG_NAME = 'Wifi Status (ddwrt)'
WARN_TIME = 30
ERROR_TIME = 60


def wifi_to_diag(msg):
    stat = DiagnosticStatus()

    stat.name = DIAG_NAME
    stat.level = DiagnosticStatus.OK
    stat.message = 'OK'

    stat.values.append(KeyValue(key='ESSID',       value=msg.essid))
    stat.values.append(KeyValue(key='Mac Address', value=msg.macaddr))
    stat.values.append(KeyValue(key='Signal',      value=str(msg.signal)))
    stat.values.append(KeyValue(key='Noise',       value=str(msg.noise)))
    stat.values.append(KeyValue(key='Sig/Noise',   value=str(msg.snr)))
    stat.values.append(KeyValue(key='Channel',     value=str(msg.channel)))
    stat.values.append(KeyValue(key='Rate',        value=msg.rate))
    stat.values.append(KeyValue(key='TX Power',    value=msg.tx_power))
    stat.values.append(KeyValue(key='Quality',     value=str(msg.quality)))

    return stat

def mark_diag_stale(diag_stat = None, error = False):
    if not diag_stat:
        diag_stat = DiagnosticStatus()
        diag_stat.message = 'No Updates'
        diag_stat.name    = DIAG_NAME
    else:
        diag_stat.message = 'Updates Stale'

    diag_stat.level = DiagnosticStatus.WARN
    if error:
        diag_stat.level = DiagnosticStatus.ERROR

    return diag_stat

class WifiMonitor(object):
    def __init__(self):
        self._mutex = threading.Lock()
        
        self._last_msg = None
        self._last_update_time = None
        self._start_time = rospy.get_time()

        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=50)

        self._ddwrt_sub = rospy.Subscriber('ddwrt/accesspoint', AccessPoint, self._cb)

    def _cb(self, msg):
        with self._mutex:
            self._last_msg = msg
            self._last_update_time = rospy.get_time()

    def publish_stats(self):
        with self._mutex:
            if self._last_msg:
                ddwrt_stat = wifi_to_diag(self._last_msg)

                update_diff = rospy.get_time() - self._last_update_time
                if update_diff > WARN_TIME:
                    ddwrt_stat = mark_diag_stale(ddwrt_stat)
                if (rospy.get_time() - self._last_update_time) > ERROR_TIME:
                    ddwrt_stat = mark_diag_stale(ddwrt_stat, True)

                ddwrt_stat.values.append(KeyValue(key='Time Since Update', value=str(update_diff)))
            else:
                error_state = (rospy.get_time() - self._start_time) > ERROR_TIME
                ddwrt_stat = mark_diag_stale(None, error_state)
                ddwrt_stat.values.append(KeyValue(key='Time Since Update', value="N/A"))

        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()
        msg.status.append(ddwrt_stat)
        
        self._diag_pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ddwrt_diag')
    except rospy.exceptions.ROSInitException:
        print 'Wifi monitor is unable to initialize node. Master may not be running.'
        sys.exit(2)
        
    wifi_monitor = WifiMonitor()
    rate = rospy.Rate(1.0)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
            wifi_monitor.publish_stats()
    except KeyboardInterrupt:
        pass
    except Exception, e:
        import traceback
        traceback.print_exc()

    sys.exit(0)
    

            
