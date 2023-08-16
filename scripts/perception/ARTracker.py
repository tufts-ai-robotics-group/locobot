import rospy
import numpy as np

# ROS based
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped

class ARTagProcessor:
    def __init__(self, tag_ids):
        self.robot_pose = PoseStamped()
        self.ar_tags = {}  # Dictionary to store detected AR tags
        self.tag_ids = tag_ids  # List of predefined AR tag IDs

        # ROS subscribers
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback)
        rospy.Subscriber("/locobot/pose", PoseStamped, self.pose_callback)

    def ar_callback(self, data):
        for marker in data.markers:
            self.ar_tags[marker.id] = marker.pose.pose

    def pose_callback(self, data):
        self.robot_pose = data.pose

    def calculate_relative_orientation(self, tag_pose):
        # Calculate the relative orientation between the robot and the AR tag
        # This is a simplified calculation and assumes that the robot and AR tag are on a flat surface
        dx = tag_pose.position.x - self.robot_pose.position.x
        dy = tag_pose.position.y - self.robot_pose.position.y
        yaw = np.arctan2(dy, dx)
        return yaw

    def is_facing_tag(self, yaw):
        # Check if the robot is facing the AR tag based on the yaw angle
        # This is a basic check and assumes a field of view of 60 degrees for the robot
        return -np.pi/6 <= yaw <= np.pi/6

    def process(self):
        one_hot_vector = [0] * len(self.tag_ids)
        for idx, tag_id in enumerate(self.tag_ids):
            if tag_id in self.ar_tags:
                yaw = self.calculate_relative_orientation(self.ar_tags[tag_id])
                if self.is_facing_tag(yaw):
                    one_hot_vector[idx] = 1
        return one_hot_vector

if __name__ == "__main__":
    predefined_tag_ids = [0, 1, 2, 3]  # Adjust this list based on your predefined AR tags
    rospy.init_node('ar_tag_processor', anonymous=True)
    processor = ARTagProcessor(predefined_tag_ids)
    rospy.spin()
