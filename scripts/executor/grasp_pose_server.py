
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
import math
from locobot_custom.srv import GraspPose, GraspPoseResponse

WORLD_TF = "map"
LOCOBOT_TF = "locobot/base_link"


class PickUpPoseCalculator:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.object_pose = None
        # i want to publish to a specific topic based on the handle_grasp_pose service request.
        # Placeholder for the publisher
        self.pose_publisher = None

    def model_state_callback(self, msg, object_name):
        if object_name in msg.name:
            index = msg.name.index(object_name)
            self.object_pose = PoseStamped(pose=msg.pose[index])
            self.object_pose.header.stamp = rospy.Time.now()
            self.object_pose.header.frame_id = "map"

    def handle_grasp_pose(self, req):
        # Create a publisher for the specific object
        self.pose_publisher = rospy.Publisher('computed_grasp_pose/{}'.format(req.object_name), PoseStamped, queue_size=10)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_state_callback, req.object_name)

        rate = rospy.Rate(10.0)
        timeout = rospy.Time.now() + rospy.Duration(5)  # 5 seconds timeout

        while self.object_pose is None and rospy.Time.now() < timeout:
            rate.sleep()

        if self.object_pose is None:
            return GraspPoseResponse(success=False, message="Object not found!")

        target_pose = self.world_to_locobot(self.object_pose)
        
        x = target_pose.pose.position.x
        y = target_pose.pose.position.y
        target_pose.pose.orientation = Quaternion(*quaternion_from_euler(ai=0, aj=1.4, ak=math.atan2(y, x)))
        target_pose.pose.position.x += 0.037943
        
        self.pose_publisher.publish(target_pose)
        return GraspPoseResponse(success=True, message="Pose computed and published successfully!")

    def world_to_locobot(self, pose_world: PoseStamped) -> PoseStamped:
        target_transform: tf2_ros.TransformStamped = self.tf_buffer.lookup_transform(LOCOBOT_TF, WORLD_TF, rospy.Time(0))
        return tf2_geometry_msgs.do_transform_pose(pose_world, target_transform)


def main():
    rospy.init_node('grasp_pose_calculator_service')
    calculator = PickUpPoseCalculator()
    s = rospy.Service('compute_and_publish_grasp_pose', GraspPose, calculator.handle_grasp_pose)
    rospy.loginfo("Ready to compute and publish grasping pose.")
    rospy.spin()


if __name__ == "__main__":
    main()