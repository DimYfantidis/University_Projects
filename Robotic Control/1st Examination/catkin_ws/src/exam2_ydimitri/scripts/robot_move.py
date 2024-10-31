#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


# rosrun exam2_ydimitri robot_move.py
def callback(msg):
    rospy.loginfo('%s', msg)


def listener():
    rospy.init_node('velocity_listener', anonymous=False)
    rospy.Subscriber("cmd_vel", Twist, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()