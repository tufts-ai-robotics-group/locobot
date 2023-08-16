import rospy
import cv2
import numpy as np

# ROS Libraries
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

class ObjectOrientationCalculator:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.robot_position = (0, 0, 0)  # Default position

        # ROS subscribers
        rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/locobot/pose", PoseStamped, self.pose_callback)

    def color_callback(self, data):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    def pose_callback(self, data):
        self.robot_position = (data.pose.position.x, data.pose.position.y, data.pose.position.z)

    def get_object_position(self, mask):
        # Assuming mask is a binary image of the segmented object
        moments = cv2.moments(mask)
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        # Get depth at the centroid
        z = self.depth_image[cy, cx]
        return (cx, cy, z)

    def calculate_relative_orientation(self, object_position):
        direction_vector = np.subtract(object_position, self.robot_position)
        pitch = np.arctan2(direction_vector[2], direction_vector[1])
        yaw = np.arctan2(direction_vector[0], direction_vector[1])
        return pitch, yaw

    def process(self):
        # Segment the object based on its color
        lower_bound = np.array([0, 0, 100])  # Adjust these values
        upper_bound = np.array([50, 50, 255])  # Adjust these values
        mask = cv2.inRange(self.color_image, lower_bound, upper_bound)

        object_position = self.get_object_position(mask)
        pitch, yaw = self.calculate_relative_orientation(object_position)
        return pitch, yaw

if __name__ == "__main__":
    rospy.init_node('object_orientation_calculator', anonymous=True)
    calculator = ObjectOrientationCalculator()
    rospy.spin()
