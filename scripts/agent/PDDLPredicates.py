import rospy
from locobot_custom.srv import Hold, HoldResponse
from locobot_custom.srv import At, AtResponse
from locobot_custom.srv import Contain, ContainResponse
from locobot_custom.srv import Facing, FacingResponse


class RecyleBotPDDLPredicates(object):

    def __init__(self) -> None:
        rospy.wait_for_service('at')
        self._at = rospy.ServiceProxy('at', At)

        rospy.wait_for_service('hold')
        self._hold = rospy.ServiceProxy('hold', Hold)

        rospy.wait_for_service('contain')
        self._contain = rospy.ServiceProxy('contain', Contain)

        rospy.wait_for_service('facing')
        self._facing = rospy.ServiceProxy('facing', Facing)

    def facing(self, object1):
        return self._facing(object1)

    def hold(self, object1):
        return self._hold(object1)

    def at(self, room1, object):
        return self._at(room1, object)

    def connect(self, room1, room2, doorway1):
        return True 

    def contain(self, object1, object2):
        return self._contain(object1, object2)