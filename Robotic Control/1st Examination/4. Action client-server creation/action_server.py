#! /usr/bin/env python
import rospy

import time # will be used for the timer functionality of action server
import actionlib # for the SimpleActionServer class (see line 24)
from math import sqrt
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from exam2_ydimitri.msg import TurtleAction, TurtleGoal, TurtleResult, TurtleFeedback


# Harcoded allowed time (wasn't mentioned in the Goal's specifications)
MAX_ALLOWED_TIME = rospy.get_param('/max_time')   # seconds
# Distance threshold when reaching the goal
DISTANCE_THRESHOLD = rospy.get_param('/distance_threshold')  

# Robot's position determined from gazebo_msgs/ModelStates
robot_position = {
    'x' : None, 
    'y' : None
}

# Euclidean distance of two points
def distance(x1, y1, x2, y2):
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def goal_function(goal):
    # Time Origin
    start_time = time.time()

    # Result action message
    result = TurtleResult()
    # Initialize elapsed time
    result.elapsed_time = rospy.Duration.from_sec(.0)

    # The nested subscriber starts when a goal arrives to fetch the robot's world position.
    position_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gps_callback)

    # Thread safety (maybe)
    while robot_position['x'] is None and robot_position['y'] is None:
        pass

    # Feedback message
    feedback = TurtleFeedback()
    while result.elapsed_time.to_sec() < MAX_ALLOWED_TIME:
        result.elapsed_time = rospy.Duration.from_sec(time.time() - start_time)
        feedback.elapsed_time = result.elapsed_time 
        dist = distance(
            robot_position['x'], robot_position['y'],
            goal.goal_x, goal.goal_y
        )
        if dist < DISTANCE_THRESHOLD:
            # Unsubscribe from /gazebo/model_states, release subscriber's resources and exit with success.
            position_sub.unregister()
            server.set_succeeded(result, "Robot has successfully reached goal coordinates (%.2f, %.2f)" % (goal.goal_x, goal.goal_y))
            return
        # Send feedback to the action client.
        server.publish_feedback(feedback)

    # Unsubscribe from /gazebo/model_states, release subscriber's resources and exit with failure.
    position_sub.unregister()
    server.set_aborted(result, "Robot could not reach goal position (%.2f, %.2f) in time" % (goal.goal_x, goal.goal_y))


# Callback function to process incoming messages on /gazebo/model_states
def gps_callback(data):
    # Determine turtlebot3_waffle_pi's existence in the Gazebo environment
    if "turtlebot3_waffle_pi" in data.name:
        # Fetch its index
        index = data.name.index("turtlebot3_waffle_pi") 
        robot_pose = data.pose[index]
        # Retrieve its (x, y) coorinates
        robot_position['x'] = robot_pose.position.x
        robot_position['y'] = robot_pose.position.y


if __name__ == '__main__':
    rospy.init_node('gps_action_server', anonymous=True)
    print('Maximum time allowed to reach goal is %f seconds' % MAX_ALLOWED_TIME)
    # Create new action server.
    server = actionlib.SimpleActionServer('gps', TurtleAction, goal_function, False)
    server.start()
    rospy.spin()




rospy.spin()

