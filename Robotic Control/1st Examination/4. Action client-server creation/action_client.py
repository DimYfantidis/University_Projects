#! /usr/bin/env python
import rospy

import actionlib
from exam2_ydimitri.msg import TurtleAction, TurtleGoal, TurtleResult, TurtleFeedback


def feedback_cb(feedback):
    print('[Feedback] Elapsed time: %.4f'% (feedback.elapsed_time.to_sec()))
    rospy.sleep(0.1)


# initalize the node
rospy.init_node('gps_action_client', anonymous=True)

# creation of a SimpleActionClient
client = actionlib.SimpleActionClient('gps', TurtleAction)

# wait for the action server to come up
client.wait_for_server()

# create the goal (x, y) coordinates
goal = TurtleGoal
goal.goal_x = rospy.get_param('/goal_x')
goal.goal_y = rospy.get_param('/goal_y')

# send the goal to the server (and the feedback function)
client.send_goal(goal, feedback_cb=feedback_cb)

# get the result and then print it
client.wait_for_result()
print('[Result] State: %d'%(client.get_state()))
print('[Result] Status: %s'%(client.get_goal_status_text()))
print('[Result] Time elapsed: %f'%(client.get_result().elapsed_time.to_sec()))