#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge, CvBridgeError


#######################
## Control Constants ##
#######################

kp = -0.1 # -ve value since +z turn corresponds to -x in camera feed


##################
## CV Functions ##
##################


def rgb_sum(rgb):
    return 1 if rgb[0] < 50 else 0


def total(row):
    return reduce(lambda acc, val: acc + rgb_sum(val), row, 0)


def moment(row):
    sum = 0
    for i in range(row.shape[0]):
        sum += i * rgb_sum(row[i])
    return sum


def error(frame):

    row = frame[-1]
    mo = moment(row)
    tot = total(row)

    x = mo / tot if tot != 0 else 0
    centre = row.shape[0] / 2

    return x - centre


###################
## ROS Functions ##
###################


def main():

    global pub, bridge, vel_msg

    bridge = CvBridge()

    vel_msg = Twist()
    vel_msg.linear.x = 1.0 # init speed

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/robot/camera1/image_raw", Image, steer)

    rospy.init_node('driver', anonymous=True)

    rospy.spin()


def steer(frame):

    cv_image = bridge.imgmsg_to_cv2(frame, desired_encoding="passthrough")

    vel_msg.angular.z = kp * error(cv_image)

    pub.publish(vel_msg)


if __name__ == '__main__':
    main()

