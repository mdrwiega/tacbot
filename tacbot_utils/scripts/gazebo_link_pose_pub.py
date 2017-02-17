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
    Script allows to subscribe Gazebo link states and publish specified
    link pose as a odometry message.
    Parameters:
        link_name
        odom_frame
        base_frame
        odom_topic
        rate_pub
'''

import rospy
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry

linkStates = LinkStates()

def callbackLinkStates(data):
  global linkStates
  linkStates = data

def converter():
    odom = Odometry()
    odom.header.frame_id = rospy.get_param('odom_frame', 'odom_ref')
    odom.child_frame_id = rospy.get_param('base_frame', 'base_link_ref')
    linkName = rospy.get_param('link_name', 'r1::r1_base_footprint')
    odomTopic = rospy.get_param('odom_topic', 'odom_ref')
    rateHz = rospy. get_param('rate_pub', 50)

    rospy.Subscriber("gazebo/link_states", LinkStates, callbackLinkStates)
    pub = rospy.Publisher(odomTopic, Odometry, queue_size=10)
    rospy.init_node('gazebo_link_pose_pub', anonymous=True)
    rospy.loginfo('Gazebo link pose as odom publisher started')
    rospy.loginfo('Selected link for publisher: %s', linkName)

    start = True
    index = -1
    r = rospy.Rate(rateHz)

    while not rospy.is_shutdown():
        if start:
            index = linkStates.name.index(linkName)
            if index != -1:
                start = False
                rospy.loginfo('Founded index of %s. Start publishing.', linkName)
        else:
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose = linkStates.pose[index]
            odom.twist.twist = linkStates.twist[index]
            pub.publish(odom)

        r.sleep()

if __name__ == '__main__':
    try:
        converter()
    except rospy.ROSInterruptException:
        pass