#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge, CvBridgeError


def main():

    global pub, bridge, vel_msg

    bridge = CvBridge()

    vel_msg = Twist()

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/robot/camera1/image_raw", Image, steer)

    rospy.init_node('driver', anonymous=True)

    rospy.spin()


def steer(frame):

    cv_image = bridge.imgmsg_to_cv2(frame, desired_encoding="passthrough")

    vel_msg.linear.x = 1.0

    pub.publish(vel_msg)


if __name__ == '__main__':
    main()

# import imageio
# import cv2
# import numpy as np
# from functools import reduce

# def rgb_sum(rgb):
#     return 1 if rgb[0] < 50 else 0

# def total(row):
#     return reduce(lambda acc, val: acc + rgb_sum(val), row, 0)

# def moment(row):
#     sum = 0
#     for i in range(row.shape[0]):
#         sum += i * rgb_sum(row[i])
#     return sum

# video_file ='raw_video_feed.mp4'

# vid = cv2.VideoCapture(video_file)
# height = vid.get(cv2.CAP_PROP_FRAME_HEIGHT)
# width = vid.get(cv2.CAP_PROP_FRAME_WIDTH)

# reader = imageio.get_reader(video_file)
# writer = imageio.get_writer('result.mp4', fps=20)

# av = 0

# # Stream through video
# for i, im in enumerate(reader):

#     # 1. Compute average pixel
#     row = im[-1]
#     mo = moment(row)
#     tot = total(row)
#     av = mo / tot if tot != 0 else av

#     # 2. Draw corresponding dot on im
#     cv2.circle(im, (int(av), int(height) - 30), 20, 0xFF0000, -1)

#     # 3. Append im back to writer
#     writer.append_data(im)
