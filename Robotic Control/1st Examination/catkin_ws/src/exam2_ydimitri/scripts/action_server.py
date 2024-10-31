#! /usr/bin/env python
import rospy

import time # will be used for the timer functionality of action server
import actionlib # for the SimpleActionServer class (see line 24)
from math import sqrt
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from exam2_ydimitri.msg import TurtleAction, TurtleGoal, TurtleResult, TurtleFeedback


MAX_ALLOWED_TIME = rospy.get_param('/max_time')   # seconds
DISTANCE_THRESHOLD = rospy.get_param('/distance_threshold')  

robot_position = {
    'x' : None, 
    'y' : None
}

def distance(x1, y1, x2, y2):
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def goal_function(goal):
    start_time = time.time()

    result = TurtleResult()
    result.elapsed_time = rospy.Duration.from_sec(.0)

    position_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gps_callback)

    while robot_position['x'] is None and robot_position['y'] is None:
        pass

    feedback = TurtleFeedback()
    while result.elapsed_time.to_sec() < MAX_ALLOWED_TIME:
        result.elapsed_time = rospy.Duration.from_sec(time.time() - start_time)
        feedback.elapsed_time = result.elapsed_time 
        dist = distance(
            robot_position['x'], robot_position['y'],
            goal.goal_x, goal.goal_y
        )
        if dist < DISTANCE_THRESHOLD:
            position_sub.unregister()
            server.set_succeeded(result, "Robot has successfully reached goal coordinates (%.2f, %.2f)" % (goal.goal_x, goal.goal_y))
            return
        server.publish_feedback(feedback)

    position_sub.unregister()
    server.set_aborted(result, "Robot could not reach goal position (%.2f, %.2f) in time" % (goal.goal_x, goal.goal_y))


# Callback function to process incoming messages on /gazebo/model_states
def gps_callback(data):
    if "turtlebot3_waffle_pi" in data.name:
        index = data.name.index("turtlebot3_waffle_pi")  # Replace "robot_model_name" with your robot's model name in Gazebo
        robot_pose = data.pose[index]
        robot_position['x'] = robot_pose.position.x
        robot_position['y'] = robot_pose.position.y


if __name__ == '__main__':
    rospy.init_node('gps_action_server', anonymous=True)
    print('Maximum time allowed to reach goal is %f seconds' % MAX_ALLOWED_TIME)
    server = actionlib.SimpleActionServer('gps', TurtleAction, goal_function, False)
    server.start()
    rospy.spin()




rospy.spin()

