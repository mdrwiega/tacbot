#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Michal Drwiega
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

'''
  This node cuts side data from LaserScan message and republish it to new topic
  Parameters of node are following:
    scan_topic -- this topic is subscribed for LaserScan messages
    scan_topic_new -- modified scan (LaserScan) is published to this topic
    left_cnt  -- number of samples removed from left side of scan
    right_cnt -- number of samples removed from right side of scan
'''

import rospy
from sensor_msgs.msg    import LaserScan

# Global variables
laserMsg = LaserScan()
publisher = None
left_cnt = 0
right_cnt = 0


def callback_laser(data):
  global laserMsg, publisher, left_cnt, right_cnt
  laserMsg = data
  if publisher is not None:
    publisher.publish(convert_laserscan(data, left_cnt, right_cnt))


def convert_laserscan(scan, left_cnt, right_cnt):
  scan.angle_min = scan.angle_min + left_cnt * scan.angle_increment
  scan.angle_max = scan.angle_max - right_cnt * scan.angle_increment

  if left_cnt > 0 and right_cnt > 0:
    scan.ranges = scan.ranges[left_cnt:-right_cnt]

  return scan


def node():
  global publisher, left_cnt, right_cnt

  rospy.init_node('laserscan_converter')

  topic = rospy.get_param('~scan_topic', 'scan')
  new_topic = rospy.get_param('~scan_topic_new', 'scan2')
  left_cnt = rospy.get_param('~left_cnt', 0)
  right_cnt = rospy.get_param('~right_cnt', 0)

  rospy.Subscriber(topic, LaserScan, callback_laser)
  publisher = rospy.Publisher(new_topic, LaserScan, queue_size=1)

  while not rospy.is_shutdown():
    rospy.sleep(1)


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
