#!/usr/bin/env python  

'''
  Script cuts side LaserScan data
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


'''
  Remove data from start and end of laserscan message
'''
def convert_laserscan(scan, left_cnt, right_cnt):
  scan.angle_min = scan.angle_min + left_cnt * scan.angle_increment
  scan.angle_max = scan.angle_max - right_cnt * scan.angle_increment

  # Remove some of first and last measurements
  if left_cnt > 0 and right_cnt > 0:
    scan.ranges = scan.ranges[left_cnt:-right_cnt]

  return scan

'''
  ROS node
'''
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
