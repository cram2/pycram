from ..robot_description import *


class UR5Description(RobotDescription):
    def __init__(self):
        # all joints which are not fix,
        super(UR5Description, self).__init__("ur5_robotiq", "world", "base_link")

        # Arm
        arm_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                      "wrist_3_joint"]
        arm_links = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]
        gripper_joints = ["robotiq_85_left_finger_joint", "robotiq_85_right_finger_joint",
                          "robotiq_85_left_inner_knuckle_joint", "robotiq_85_right_inner_knuckle_joint",
                          "robotiq_85_left_finger_tip_joint", "robotiq_85_left_finger_tip_joint"]
        gripper_links = ["robotiq_85_base_link", "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
                         "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
                         "robotiq_85_left_inner_knuckle_link", "robotiq_85_right_inner_knuckle_link",
                         "robotiq_85_left_finger_tip_link" "robotiq_85_right_finger_tip_link"]

        # Arm
        manipulator_chain = ChainDescription("manipulator", arm_joints, arm_links)
        manipulator_inter = InteractionDescription(manipulator_chain, "ee_link")
        # Gripper
        gripper = GripperDescription("gripper", gripper_links, gripper_joints)
        # Adding Arm + Gripper
        manipulator = ManipulatorDescription(manipulator_inter, tool_frame="ee_link",
                                             gripper_description=gripper)
        self.add_chains({"manipulator": manipulator})
        # Adding Static Joint Poses
        # Static Arm Positions
        manipulator_home = [0, 0, 0, 0, 0, 0]
        self.add_static_joint_chain("manipulator", "home", manipulator_home)
        # Static Gripper Positions
        gripper_confs = {"open": [0.0], "close": [1.0]}
        self.add_static_gripper_chains("gripper", gripper_confs)

    def get_camera_frame(self, name="camera"):
        # TODO: Hacky since only one optical camera frame from pr2 is used
        return super().get_camera_frame(name)


