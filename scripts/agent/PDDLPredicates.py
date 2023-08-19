import rospy

class RecyleBotPDDLPredicates(object):

    def __init__(self) -> None:
        rospy.wait_for_service('at')
        self._at = rospy.ServiceProxy('at')

        rospy.wait_for_service('hold')
        self._hold = rospy.ServiceProxy('hold')

        rospy.wait_for_service('contain')
        self._contain = rospy.ServiceProxy('contain')

        rospy.wait_for_service('facing')
        self._facing = rospy.ServiceProxy('facing')

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