#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

import rospy

import math
import tf
import geometry_msgs.msg
import turtlesim.srv

WORLD_MID_POINT = (5.544, 5.544)


if __name__ == '__main__':
    rospy.init_node('turtle_pointer')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(WORLD_MID_POINT[0], WORLD_MID_POINT[1], 0, 'turtle2')
    spawner(2, 2, 0, 'turtle3')
    
    turtle2_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=20)
    turtle3_vel = rospy.Publisher('turtle3/cmd_vel', geometry_msgs.msg.Twist, queue_size=20)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        angular = 4 * math.atan2(trans[1], trans[0])
        msg = geometry_msgs.msg.Twist()
        # Turtle2 simply observes the user-controlled turtle
        msg.angular.z = angular
        turtle2_vel.publish(msg)

        try:
            (trans, rot) = listener.lookupTransform('/turtle3', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)

        try:
            (trans, rot) = listener.lookupTransform('/world', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # Turtle1's world coordinates
        x1, y1, _ = tuple(trans)
        msg = geometry_msgs.msg.Twist()
        # The third turtle stalks the user-controlled turtle (turtle1), unless turtle1 is on the upper-right quadrant
        if x1 < WORLD_MID_POINT[0] or y1 < WORLD_MID_POINT[1]:
            msg.linear.x = linear 
        msg.angular.z = angular
        turtle3_vel.publish(msg)

        rate.sleep()
