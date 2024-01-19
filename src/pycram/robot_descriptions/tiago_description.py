from ..robot_description import *


class TiagoDescription(RobotDescription):

    def __init__(self):
        super().__init__("tiago_dual", "base_footprint", "base_link", "torso_lift_link", "torso_lift_joint")
        # Camera
        minimal_height = 1.0665
        maximal_height = 1.4165
        horizontal_angle = 0.99483
        vertical_angle = 0.75049

        optical = CameraDescription("xtion_optical_frame",
                                    horizontal_angle=horizontal_angle, vertical_angle=vertical_angle,
                                    minimal_height=minimal_height, maximal_height=maximal_height)
        rgb_optical = CameraDescription("xtion_rgb_optical_frame",
                                        horizontal_angle=horizontal_angle, vertical_angle=vertical_angle,
                                        minimal_height=minimal_height, maximal_height=maximal_height)
        depth = CameraDescription("xtion_rgb_optical_frame",
                                  horizontal_angle=horizontal_angle, vertical_angle=vertical_angle,
                                  minimal_height=minimal_height, maximal_height=maximal_height)

        self.add_cameras({"optical": optical, "rgb_optical": rgb_optical, "depth": depth})

        # The axis which points away from the camera and along which the picture of the camera is created
        self.front_facing_axis = [0, 0, 1]
        # Chains
        neck_static_joints = dict()
        neck_static_joints["forward"] = [0.0, 0.0]
        neck = ChainDescription("neck", ["head_1_joint", "head_2_joint"],
                                ["head_1_link", "head_2_link"],
                                static_joint_states=neck_static_joints)
        self.add_chain("neck", neck)
        # Arms
        r_joints = ["arm_right_1_joint",
                    "arm_right_2_joint",
                    "arm_right_3_joint",
                    "arm_right_4_joint",
                    "arm_right_5_joint",
                    "arm_right_6_joint",
                    "arm_right_7_joint"]
        l_joints = ["arm_left_1_joint",
                    "arm_left_2_joint",
                    "arm_left_3_joint",
                    "arm_left_4_joint",
                    "arm_left_5_joint",
                    "arm_left_6_joint",
                    "arm_left_7_joint"]
        r_links = ["arm_right_1_link",
                   "arm_right_2_link",
                   "arm_right_3_link",
                   "arm_right_4_link",
                   "arm_right_5_link",
                   "arm_right_6_link",
                   "arm_right_7_link",
                   "arm_right_tool_link",
                   "wrist_right_ft_link",
                   "wrist_right_ft_tool_link",
                   "gripper_right_link",
                   "gripper_right_left_finger_link",
                   "gripper_right_right_finger_link",
                   "gripper_right_tool_link"]
        l_links = ["arm_left_1_link",
                   "arm_left_2_link",
                   "arm_left_3_link",
                   "arm_left_4_link",
                   "arm_left_5_link",
                   "arm_left_6_link",
                   "arm_left_7_link",
                   "arm_left_tool_link",
                   "wrist_left_ft_link",
                   "wrist_left_ft_tool_link",
                   "gripper_left_link",
                   "gripper_left_left_finger_link",
                   "gripper_left_right_finger_link",
                   "gripper_left_tool_link"]
        r_gripper_joints = ["gripper_right_left_finger_joint", "gripper_right_right_finger_joint"]
        r_gripper_links = ["gripper_right_link",
                           "gripper_right_left_finger_link",
                           "gripper_right_right_finger_link",
                           "gripper_right_tool_link"]
        l_gripper_joints = ["gripper_left_left_finger_joint", "gripper_left_right_finger_joint"]
        l_gripper_links = ["gripper_left_link",
                           "gripper_left_left_finger_link",
                           "gripper_left_right_finger_link",
                           "gripper_left_tool_link"]
        # Arms
        left = ChainDescription("left", l_joints, l_links)
        right = ChainDescription("right", r_joints, r_links)
        left_inter = InteractionDescription(left, "arm_left_tool_link")
        right_inter = InteractionDescription(right, "arm_right_tool_link")
        # Gripper
        left_gripper = GripperDescription("left_gripper", l_gripper_links, l_gripper_joints,
                                          gripper_meter_to_jnt_multiplier=5.0,
                                          gripper_minimal_position=0.0,
                                          gripper_convergence_delta=0.005)
        right_gripper = GripperDescription("right_gripper", r_gripper_links, r_gripper_joints,
                                           gripper_meter_to_jnt_multiplier=5.0,
                                           gripper_minimal_position=0.0,
                                           gripper_convergence_delta=0.005)
        # Adding Arm + Gripper
        left_arm = ManipulatorDescription(left_inter, tool_frame="gripper_left_grasping_frame",
                                          gripper_description=left_gripper)
        right_arm = ManipulatorDescription(right_inter, tool_frame="gripper_right_grasping_frame",
                                           gripper_description=right_gripper)
        self.add_chains({"left": left_arm, "right": right_arm})
        # Adding Static Joint Poses
        # Static Arm Positions
        r_arm_park = [0.27, -1.07, 1.5, 2.0, -2.0, 1.2, 0.5]
        l_arm_park = [0.27, -1.07, 1.5, 1.96, -2.0, 1.2, 0.5]
        self.add_static_joint_chain("right", "park", r_arm_park)
        self.add_static_joint_chain("left", "park", l_arm_park)
        # Static Gripper Positions
        gripper_confs = {"open": [0.048, 0.048], "close": [0.0, 0.0]}
        self.add_static_gripper_chains("left", gripper_confs)
        self.add_static_gripper_chains("right", gripper_confs)

        self.grasps = GraspingDescription({"front": [0, 0, 0, 1],
                                           "left": [0, 0, -1, 1],
                                           "right": [0, 0, 1, 1],
                                           "top": [0, 1, 0, 1]})

    def get_camera_frame(self, name="optical"):
        # TODO: Hacky since only one optical camera frame from pr2 is used
        return super().get_camera_frame(name)
