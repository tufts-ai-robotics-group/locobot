#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import PickupActionGoal
from gazebo_msgs.msg import ModelStates


def model_states_callback(msg):
    # Find the index of the "ball" object in the model states
    try:
        object_index = msg.name.index("cricket_ball")
    except ValueError:
        rospy.logwarn("Object 'ball' not found in ModelStates.")
        return

    # Create the PickupActionGoal message
    pickup_goal = PickupActionGoal()

    # Fill in the necessary details of the pickup goal message
    pickup_goal.goal.target_name = 'cricket_ball'
    pickup_goal.goal.possible_grasps = []  # You can provide a list of possible grasps if available
    pickup_goal.goal.allowed_touch_objects = []  # You can specify objects that are allowed to be touched during pickup
    pickup_goal.goal.planning_options.planning_scene_diff.is_diff = True

    # Publish the pickup goal
    pub.publish(pickup_goal)

    # Stop the node after publishing the goal
    rospy.signal_shutdown('Goal sent.')


if __name__ == '__main__':
    rospy.init_node('pickup_publisher')

    # Create a publisher for the PickupActionGoal topic
    pub = rospy.Publisher('/locobot/pickup/goal', PickupActionGoal, queue_size=10)

    # Subscribe to the ModelStates topic
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

    # Spin to process callbacks
    rospy.spin()