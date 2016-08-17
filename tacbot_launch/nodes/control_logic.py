#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Michał Drwięga
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#####################################################################

import rospy
import sys
import struct as st

from time import time
from std_msgs.msg import Int8, UInt8

# Acceptable statuses
statuses = {'OFF': 0, 'STARTUP': 10, 'FAILURE': 20, 'FAILURE_CLR': 22,
                'SHUTDOWN': 30, 'HOLD_POSE': 40, 'PLATF_MANUAL_MOVE': 60,
                'PLATF_REMOTE_MOVE': 70, 'PLATF_AUTONOMIC_MOVE': 80}

# Global variables
status_publisher = None
platform_status = 0 # Status from mobile platform controller

# Callback for platform status
def callback_status(data):
  global platform_status
  platform_status = data.data
  print 'Received status: ', data

'''
  Decode status name from it value (key from dict by value)
'''
def find_status_name(val):
  stat = 'Unrecognized status'
  try:
    stat = (key for key, value in statuses.items() if value == val).next()
  finally:
    return stat

'''
  ROS node
'''
def node():
  global publisher, platform_status

  rospy.init_node("control_logic")

  status_out_topic = rospy.get_param('~status_out', 'platform/RCCS_status')
  platform_status_topic = rospy.get_param('~platform_status', 'platform/MobilePlatform_status')

  rospy.Subscriber(platform_status_topic, UInt8, callback_status)
  status_publisher = rospy.Publisher(status_out_topic, Int8, queue_size=1)

  # Set initially state of system as OFF
  status = statuses['OFF']
  cnt = 0
  cnt_print = 0

  # Node loop
  while not rospy.is_shutdown():
    # Init state OFF
    if status == statuses['OFF']:
      cnt += 1
      if cnt == 5:
        cnt = 0
        status = statuses['STARTUP']

    # After some time the startup state activate
    elif status == statuses['STARTUP']:
      if platform_status == statuses['HOLD_POSE']:
        status = statuses['HOLD_POSE']

    elif status == statuses['FAILURE']:
      if platform_status == statuses['FAILURE_CLR']:
        status = statuses['FAILURE_CLR']

    elif status == statuses['FAILURE_CLR']:
      status = statuses['HOLD_POSE']

    elif status == statuses['HOLD_POSE'] or \
        status == statuses['PLATF_MANUAL_MOVE'] or \
        status == statuses['PLATF_REMOTE_MOVE'] or \
        status == statuses['PLATF_AUTONOMIC_MOVE']:

      if platform_status == statuses['FAILURE'] or \
          platform_status == statuses['HOLD_POSE'] or \
          platform_status == statuses['PLATF_MANUAL_MOVE'] or \
          platform_status == statuses['PLATF_REMOTE_MOVE'] or \
          platform_status == statuses['PLATF_AUTONOMIC_MOVE']:
        status = platform_status
      elif platform_status == statuses['OFF']:
        status = statuses['STARTUP']

      status_publisher.publish(status)

    cnt_print += 1
    if cnt_print >= 20:
      cnt_print = 0
      print 'RCCS status = ' , find_status_name(status), \
          ' Platform status = ', find_status_name(platform_status)

    rospy.sleep(0.02)

if __name__ == '__main__':
  try:
    node()
  except rospy.ROSInterruptException:
    pass