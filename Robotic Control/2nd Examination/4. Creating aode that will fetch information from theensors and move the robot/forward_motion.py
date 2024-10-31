#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


# The subscriber determines how much the robot must steer based on the lidar measurements
def lidar_callback(scan, callback_args):
    distance_threshold, steer_dict = callback_args
    rho = np.array(scan.ranges)

    # Isolate the laser data in the front of the robot (-30 degrees to 30 degrees)
    right = rho[330:]
    left = rho[:30]

    # Isolate the points whose distances are closer than the threshold value
    left = left[left < distance_threshold]
    right = right[right < distance_threshold]

    # print("> Left: %s" % left)
    # print("> Right: %s\n" % right)

    steer_dict['right'] = left.shape[0]
    steer_dict['left'] = right.shape[0]
    steer_dict['total'] = -steer_dict['right'] if steer_dict['right'] > steer_dict['left'] else steer_dict['left']
    # print("steer RIGHT: %d | steer LEFT: %d | TOTAL: %d" % (steer_dict['right'], steer_dict['left'], steer_dict['total']))


if __name__ == '__main__':
    rospy.init_node(rospy.get_param("/node_name"), anonymous=True)

    # Result accumulator variable from the subscriber
    steer = { 'right' : 0, 'left' : 0, 'total' : 0 }

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber(
        name='/scan', 
        data_class=LaserScan, 
        callback=lidar_callback,
        # locate obstacles that are less than 0.6 units away from the lidar
        callback_args=(rospy.get_param("/proximity_threshold"), steer)
    )

    try:
        rate = rospy.Rate(rospy.get_param("/refresh_rate"))
        SCALE = rospy.get_param("/steer_scaling_const")
        LINEAR = rospy.get_param("/linear_speed_const")
        while not rospy.is_shutdown():
            cmd = Twist()
            factor = steer['total'] / SCALE
            cmd.angular.z = (math.e ** abs(factor)) * factor
            cmd.linear.x = (math.e ** (-abs(cmd.angular.z))) * LINEAR
            pub.publish(cmd)
            rate.sleep()
    except KeyboardInterrupt:
        pub.publish(Twist())
    except rospy.ROSInterruptException:
        pub.publish(Twist())
    finally:
        sub.unregister()
        pub.unregister()