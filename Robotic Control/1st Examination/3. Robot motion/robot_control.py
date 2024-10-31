#! /usr/bin/env python
from time import sleep
import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt

import math
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
import pdb


bridge = CvBridge()

lower_yellow = np.array([10,255,255])
upper_yellow = np.array([20,255,255])

lower_white = np.array([255,255,255])
upper_white = np.array([255,255,255])


steer = {
    'left' : .0, 
    'right' : .0, 
    'diff' : .0, 
}
speed = [0, 0]

def show_image(img):
    mask_yellow = cv2.inRange(img,lower_yellow,upper_yellow)
    mask_white = cv2.inRange(img,lower_white,upper_white)

    mask_yellow[:3*mask_yellow.shape[0]//4, :] = 0
    mask_white[:3*mask_white.shape[0]//4, :] = 0

    steer['left'] = float(np.count_nonzero(mask_yellow == 255)) / (mask_yellow.size // 4)
    steer['right'] = float(np.count_nonzero(mask_white == 255)) / (mask_white.size // 4)
    steer['diff'] = (steer['right'] - steer['left'])

    speed[0] = 10 * steer['diff'] * math.e ** abs(20 * steer['diff'])
    speed[1] = 0.2 * (math.e ** (-abs(velocity.angular.z))) ** 2

    print('diff: %f' % steer['diff'])
    print('angular: %f' % speed[0])
    print('linear: %f' % speed[1])

    cv2.namedWindow("subscriber preview")
    cv2.imshow('subscriber preview',mask_yellow+mask_white)
    cv2.waitKey(1)


# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)
    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8") 
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
    # Show the converted image
    show_image(cv_image)


rospy.init_node('robot_controller', anonymous=True)
sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)

rate = rospy.Rate(30)  # 30Hz
rospy.loginfo("Publisher nodes started publishing messages")

# Our custom message
velocity = Twist()
while not rospy.is_shutdown():
    velocity.angular.x = .0
    velocity.angular.y = .0
    velocity.angular.z =  speed[0]

    velocity.linear.x = speed[1]
    velocity.linear.y = .0
    velocity.linear.z = .0
    # Broadcast the message
    pub_vel.publish(velocity)
    rate.sleep()
