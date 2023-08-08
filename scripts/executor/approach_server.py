#!/usr/bin/env python3

import rospy
# from locobot_custom.srv import Approach 
from locobot_custom.srv import Approach
 

from geometry_msgs.msg import PoseStamped, Point, Quaternion

GOAL_LOCATIONS = {
    "table": {
        "position": Point(x=-2.9408016204833984, y=-1.3139156103134155, z=0.0),
        "orientation": Quaternion(x=0.0, y=0.0, z=-0.9152988066802662, w=0.402775488938541)
    },
    "chair": {
        "position": Point(x=2.521484613418579, y=-2.043757200241089, z=0.0),
        "orientation": Quaternion(x=0.0, y=0.0, z=-0.7040760069197588, w=0.7101246203871034)
    },
    "doorway_facing_green": {
        "position": Point(x=0.3962286114692688, y=0.09294455498456955, z=0.0),
        "orientation": Quaternion(x=0.0, y=0.0, z=0.9999957096443621, w=0.0029292819715136826)
    },
    "doorway_facing_blue": {
        "position": Point(x=-0.6024703979492188, y=0.1385389119386673, z=0.0),
        "orientation": Quaternion(x=0.0, y=0.0, z=-0.004361087136426318, w=0.9999904904142781)
    },
    "large_sofa": {
        "position": Point(x=-4.568149566650391, y=0.24872761964797974, z=0.0),
        "orientation": Quaternion(x=0.0, y=0.0, z=-0.9999849932886496, w=0.005478430203923843)
    },
    "small_sofa": {
        "position": Point(x=-1.1403510570526123, y=2.014651298522949, z=0.0),
        "orientation": Quaternion(x=0.0, y=0.0, z=0.012268661057075136, w=0.9999247371456846)
    },
    "bin":{
        "position": Point(x=5.079290390014648, y=1.5830466747283936, z=0.0),
        "orientation": Quaternion(x=0.0, y=0.0, z=0.8494417972561054, w=0.527682322116553)
    }
}

def initialize_ros_node():
    if not rospy.get_node_uri():
        rospy.init_node('approach_server', anonymous=True)

def send_goal(goal_name: str):
    goal_pub = rospy.Publisher('/locobot/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # Allow time for publishers to initialize

    pose_dict = GOAL_LOCATIONS[goal_name]

    goal = PoseStamped()
    goal.header.frame_id = 'map'  # Replace 'map' with your desired frame
    goal.pose.position = pose_dict['position']
    goal.pose.orientation = pose_dict['orientation']

    goal_pub.publish(goal)
    rospy.loginfo("Goal sent!")

    return True, "Goal sent!"

def handle_approach(req):
    rospy.loginfo("Executing approach action towards %s", req.target)

    success, info = send_goal(req.target)

    return {"success": success, "message": info}

if __name__ == "__main__":
    initialize_ros_node()
    rospy.Service('approach', Approach, handle_approach)
    rospy.spin()
