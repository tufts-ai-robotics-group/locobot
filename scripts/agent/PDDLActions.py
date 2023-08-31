import rospy
from locobot_custom.srv import Approach, ApproachResponse
from locobot_custom.srv import Grasp, GraspResponse
from locobot_custom.srv import GraspPose, GraspPoseResponse

class RecyleBotPDDLActions(object):

    OBJECT_GAZEBO_MAPPING = {
        "ball_1": "ball",
        "can": "can",
        # ... add more mappings as needed
    }

    def __init__(self):

        rospy.wait_for_service('approach')
        self._approach = rospy.ServiceProxy('approach', Approach)

        rospy.wait_for_service('compute_and_publish_grasp_pose')
        self._compute_grasp_pose = rospy.ServiceProxy('compute_and_publish_grasp_pose', GraspPose)
        
        rospy.wait_for_service('grasp')
        self._grasp = rospy.ServiceProxy('grasp', Grasp)

    def approach(self, object1, room1, object2):
        self._approach(object1)
        
    def pass_through_door(self, room1, room2, doorway1):
        self._approach(f"doorway_facing_{room2}")

    def pick(self, object1, room1):
        gazebo_object_name = self.OBJECT_GAZEBO_MAPPING.get(object1)
        if not gazebo_object_name:
            print(f"Object {object1} not found in Gazebo mapping.")
            return

        # First, compute and publish the grasp pose for the object
        grasp_pose_response = self._compute_grasp_pose(gazebo_object_name)
        if not grasp_pose_response.success:
            print(f"Failed to compute grasp pose for {object1}. Message: {grasp_pose_response.message}")
            return

        # Now, execute the grasp using the grasp service
        grasp_response = self._grasp(gazebo_object_name)
        if not grasp_response.success:
            print(f"Failed to grasp {object1}. Message: {grasp_response.message}")

    def place(self, object1, room1, object2):
        pass

