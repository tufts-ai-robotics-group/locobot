"""
yolo_find_objects.py
--------------------

author: Andrew Esteves (Modified by: Brennan)

This script is used to find objects using YOLO.
"""

#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from locobot_msgs.srv import YoloObject, YoloObjectResponse


class YoloFindObject:
    def __init__(self) -> None:
        rospy.init_node("yolo_find_object", anonymous=True)  # Initialize node

        self.yolo_pub = rospy.Publisher("Center_bound", Point, queue_size=1000)
        rospy.Service("yolo_find_object", YoloObject, self.yolo_find_object)
        rospy.spin()

    def yolo_find_object(self, req):
        """
        I/P: Takes in the bounding box and depth image
        O/P: returns centroid point on target object
        Description:Gets bouding box from yolo and returns point on target object
        """
        try:
            # Check if any detected objects are in the specified array
            yolo_msg = rospy.wait_for_message(
                "/darknet_ros/bounding_boxes", BoundingBoxes, timeout=10
            )
            depth_map = rospy.wait_for_message(
                "/locobot/camera/depth/image_rect_raw", Image, timeout=10
            )
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for message")
            raise rospy.ROSException("Timeout while waiting for message")

        cv_bridge = CvBridge()
        depth_image = cv_bridge.imgmsg_to_cv2(depth_map, desired_encoding="passthrough")

        detected_objects = yolo_msg.bounding_boxes  # Store bounding boxes
        for bbox in detected_objects:  # Looks through all bounding boxes to see if any contain object of interest
            if bbox.Class in req.objects:
                center_x = round(
                    (bbox.xmin + bbox.xmax) / 2
                )  # Finds center of bounding box
                center_y = round((bbox.ymin + bbox.ymax) / 2)

                # Convert pixel to 3D point in camera frame
                z_camera = (
                    depth_image[center_y, center_x] / 1000.0
                )  # Convert mm to meters
                x_camera = (center_x - self.cx) * z_camera / self.fx
                y_camera = (center_y - self.cy) * z_camera / self.fy

                point_camera = Point()
                point_camera.x = x_camera
                point_camera.y = y_camera
                point_camera.z = z_camera

                return YoloObjectResponse(point_camera)

        raise ValueError("No objects found")


if __name__ == "__main__":
    YoloFindObject()
