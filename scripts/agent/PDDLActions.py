
import rospy 

class RecyleBotPDDLActions(object):

    def __init__(self):

        rospy.wait_for_service('approach')
        self._approach = rospy.ServiceProxy('approach')

        rospy.wait_for_service('grasp')
        self._grasp = rospy.ServiceProxy('grasp')

        rospy.wait_for_service('place')
        self._place = rospy.ServiceProxy('place')

    def approach(self, object1, room1):
        self._approach(object1)
        
    def pass_through_door(self, room1, room2, doorway1):
        self._approach(f"doorway_facing_{room2}")

    def pick(self, object1, room1):
        self._grasp(object1)

    def place(self, object1, room1, object2):
        self._place(object1, object2)