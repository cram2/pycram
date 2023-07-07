import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped


class Pose(PoseStamped):

    def __init__(self, position=None, orientation=None, frame="map"):
        super().__init__()
        if position:
            self.pose.position.x = position[0]
            self.pose.position.y = position[1]
            self.pose.position.z = position[2]

        if orientation:
            self.pose.orientation.x = orientation[0]
            self.pose.orientation.y = orientation[1]
            self.pose.orientation.z = orientation[2]
            self.pose.orientation.w = orientation[3]
        else:
            self.pose.orientation.w = 1.0

        self.header.frame_id = frame
        self.header.stamp = rospy.Time.now()

        self.position = self.pose.position
        self.orientation = self.pose.orientation

        self.frame = frame

    @property
    def frame(self):
        return self.header.frame_id

    @frame.setter
    def frame(self, value):
        self.header.frame_id = value

    def to_list(self):
        return [[self.position.x, self.position.y, self.position.z],
                [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]]

    def to_transform(self, child_frame: str):
        t = TransformStamped()
        t.header = self.header
        t.transform.rotation = self.orientation
        t.child_frame_id = child_frame

        # Because TransformStamped used Vector3 and PoseStamped uses Point
        t.transform.translation.x = self.position.x
        t.transform.translation.y = self.position.y
        t.transform.translation.z = self.position.z

        return t


