from ..robot_description import *


class BoxyDescription(RobotDescription):

    def __init__(self):

        super().__init__("boxy", "base_footprint", "base_link", "triangle_base_link",
                         "triangle_base_joint", odom_frame="odom",
                         odom_joints=["odom_x_joint", "odom_y_joint", "odom_z_joint"])
        camera = CameraDescription("head_mount_kinect2_rgb_optical_frame",
                                   minimal_height=0.0, maximal_height=2.5,
                                   horizontal_angle=0.99483, vertical_angle=0.75049)
        self.add_camera("camera", camera)
        # The axis which points away from the camera and along which the picture of the camera is created
        self.front_facing_axis = [0, 0, 1]
        # Neck
        neck_links = ["neck_shoulder_link", "neck_upper_arm_link", "neck_forearm_link",
                      "neck_wrist_1_link", "neck_wrist_2_link", "neck_wrist_3_link"]
        neck_joints = ["neck_shoulder_pan_joint", "neck_shoulder_lift_joint", "neck_elbow_joint",
                       "neck_wrist_1_joint", "neck_wrist_2_joint", "neck_wrist_3_joint"]
        neck_base = "neck_base_link"
        neck_away = [-1.3155, -1.181355, -1.9562, 0.142417, 1.13492, 0.143467]  # up
        neck_down = [-1.176, -3.1252, -0.8397, 0.83967, 1.1347, -0.0266]
        neck_down_left = [-0.776, -3.1252, -0.8397, 0.83967, 1.1347, -0.0266]
        neck_down_right = [-2.176, -3.1252, -0.8397, 0.83967, 1.1347, -0.0266]
        neck_behind_up = [2.10, -1.11, -1.89, 0.64, 1.11, -0.43]
        neck_behind = [-1.68, -0.26, -0.12, -0.24, 1.49, 3.09]
        # neck_forward = [-1.39, -3.09, -0.78, 1.91, 1.51, -0.14]
        neck_forward = [-1.20, 2.39, 0.17, 1.77, 1.44, -0.35]
        # neck_right = [-1.20, 2.39, 0.17, 1.77, 1.44, 0.0]
        # neck_left = [-1.20, 2.39, 0.17, 1.77, 1.44, -0.7]
        neck = ChainDescription("neck", neck_joints, neck_links, base_link=neck_base)
        neck.add_static_joint_chains({"away": neck_away, "down": neck_down, "down_left": neck_down_left,
                                      "down_right": neck_down_right, "behind_up": neck_behind_up, "behind": neck_behind,
                                      "forward": neck_forward})
        self.add_chain("neck", neck)
        front = [-1.5, -1.57, 0, 1.58, -1.5, 0]
        neck_right = [3.14, -1.57, 0, 1.58, -1.5, 0]
        back = [1.5, -1.57, 0, 1.58, -1.5, 0]
        neck_left = [0, -1.57, 0, 1.58, -1.5, 0]
        self.add_static_joint_chains("neck", {"front": front, "neck_right": neck_right, "back": back,
                                              "neck_left": neck_left})

        # Arms
        left_joints = ["left_arm_0_joint", "left_arm_1_joint", "left_arm_2_joint",
                       "left_arm_3_joint", "left_arm_4_joint", "left_arm_5_joint",
                       "left_arm_6_joint"]
        left_links = ["left_arm_1_link", "left_arm_2_link", "left_arm_3_link",
                      "left_arm_4_link", "left_arm_5_link", "left_arm_6_link",
                      "left_arm_7_link"]
        right_joints = ["right_arm_0_joint", "right_arm_1_joint", "right_arm_2_joint",
                        "right_arm_3_joint", "right_arm_4_joint", "right_arm_5_joint",
                        "right_arm_6_joint"]
        right_links = ["right_arm_1_link", "right_arm_2_link", "right_arm_3_link",
                       "right_arm_4_link", "right_arm_5_link", "right_arm_6_link",
                       "right_arm_7_link"]
        # Gripper
        gripper_meter_to_jnt_multiplier = 1.0
        gripper_minimal_position = 0.0
        gripper_convergence_delta = 0.001
        l_gripper_links = ["left_gripper_base_link", "left_gripper_finger_left_link",
                           "left_gripper_finger_right_link", "left_gripper_gripper_left_link",
                           "left_gripper_gripper_right_link", "left_gripper_tool_frame"]
        l_gripper_joints = ["left_gripper_joint"]
        left_gripper = GripperDescription("left_gripper", gripper_links=l_gripper_links,
                                          gripper_joints=l_gripper_joints,
                                          gripper_meter_to_jnt_multiplier=gripper_meter_to_jnt_multiplier,
                                          gripper_minimal_position=gripper_minimal_position,
                                          gripper_convergence_delta=gripper_convergence_delta)
        r_gripper_links = ["right_gripper_base_link", "right_gripper_finger_left_link",
                           "right_gripper_finger_right_link", "right_gripper_gripper_left_link",
                           "right_gripper_gripper_right_link", "right_gripper_tool_frame"]
        r_gripper_joints = ["right_gripper_joint"]
        right_gripper = GripperDescription("right_gripper", gripper_links=r_gripper_links,
                                           gripper_joints=r_gripper_joints,
                                           gripper_meter_to_jnt_multiplier=gripper_meter_to_jnt_multiplier,
                                           gripper_minimal_position=gripper_minimal_position,
                                           gripper_convergence_delta=gripper_convergence_delta)
        # Arms
        left = ChainDescription("left", left_joints, left_links)
        left_inter = InteractionDescription(left, "left_arm_7_link")
        left_arm = ManipulatorDescription(left_inter, tool_frame="left_gripper_tool_frame",
                                          gripper_description=left_gripper)
        right = ChainDescription("right", right_joints, right_links)
        right_inter = InteractionDescription(right, "right_arm_7_link")
        right_arm = ManipulatorDescription(right_inter, tool_frame="right_gripper_tool_frame",
                                           gripper_description=right_gripper)
        self.add_chains({"left": left_arm, "right": right_arm})
        # Adding Static Joint Poses
        # Static Arm Positions
        l_carry = [-1.858, 0.70571, 0.9614, -0.602, -2.5922, -1.94065, -1.28735]
        r_carry = [1.858, -0.70571, -0.9614, 0.602, 2.5922, 1.94065, 1.28735]
        l_handover = [-0.32, 1.8, -0.74, -1.49, 2.29, 1.68, 0.2]
        l_flip = [-1.2274070978164673, 0.8496202230453491, -0.10349386930465698,
                  -1.0852965116500854, -0.4587952196598053, 1.259474515914917,
                  -0.06962397694587708]
        left_arm.add_static_joint_chains({"park": l_carry, "handover": l_handover, "flip": l_flip})
        right_arm.add_static_joint_chain("park", r_carry)

        # Grasping Poses
        self.grasps = GraspingDescription({"left":[1,0,0,1],
                                        "top": [1,1,0,0],
                                        "right": [0,1,1,0],
                                        "front": [1,0,1,0]})