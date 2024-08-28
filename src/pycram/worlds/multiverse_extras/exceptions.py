from pycram.datastructures.enums import JointType


class UnsupportedJointType(Exception):
    def __init__(self, joint_type: 'JointType'):
        super().__init__(f"Unsupported joint type: {joint_type}")
