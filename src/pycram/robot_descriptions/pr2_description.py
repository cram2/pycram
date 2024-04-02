from ..robot_description import *


class PR2Description(RobotDescription):

    def __init__(self):
        super().__init__("pr2", "base_footprint", "base_link", "torso_lift_link", "torso_lift_joint")
        # Camera
        minimal_height = 1.27
        maximal_height = 1.60
        horizontal_angle = 0.99483
        vertical_angle = 0.75049
        kinect = CameraDescription("head_mount_kinect_rgb_optical_frame",
                                   horizontal_angle=horizontal_angle, vertical_angle=vertical_angle,
                                   minimal_height=minimal_height, maximal_height=maximal_height)
        openni = CameraDescription("openni_rgb_optical_frame",
                                   horizontal_angle=horizontal_angle, vertical_angle=vertical_angle,
                                   minimal_height=minimal_height, maximal_height=maximal_height)
        stereo = CameraDescription("narrow_stereo_optical_frame",
                                   horizontal_angle=horizontal_angle, vertical_angle=vertical_angle,
                                   minimal_height=minimal_height, maximal_height=maximal_height)
        wide_stereo_optical_frame = CameraDescription("wide_stereo_optical_frame",
                                                      horizontal_angle=horizontal_angle,
                                                      vertical_angle=vertical_angle,
                                                      minimal_height=minimal_height, maximal_height=maximal_height)
        self.add_cameras({"optical_frame": wide_stereo_optical_frame, "kinect": kinect,
                          "openni": openni, "stereo": stereo})

        # The axis which points away from the camera and along which the picture of the camera is created
        self.front_facing_axis = [0, 0, 1]
        # Chains
        neck_static_joints = {}
        neck_static_joints["forward"] = [0.0, 0.0]
        neck = ChainDescription("neck", ["head_pan_joint", "head_tilt_joint"],
                                ["head_pan_link", "head_tilt_link"],
                                static_joint_states=neck_static_joints)
        self.add_chain("neck", neck)
        # Arms
        r_joints = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint",
                    "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint",
                    "r_wrist_roll_joint"]
        l_joints = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
                    "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint",
                    "l_wrist_roll_joint"]
        r_links = ["r_shoulder_pan_link", "r_shoulder_lift_link", "r_upper_arm_roll_link",
                   "r_upper_arm_link", "r_elbow_flex_link", "r_forearm_roll_link",
                   "r_forearm_link", "r_wrist_flex_link", "r_wrist_roll_link",
                   "r_gripper_led_frame", "r_gripper_motor_accelerometer_link", "r_gripper_tool_frame",
                   "r_gripper_motor_slider_link", "r_gripper_motor_screw_link"]
        l_links = ["l_shoulder_pan_link", "l_shoulder_lift_link", "l_upper_arm_roll_link",
                   "l_upper_arm_link", "l_elbow_flex_link", "l_forearm_roll_link", "l_forearm_link",
                   "l_wrist_flex_link", "l_wrist_roll_link", "l_gripper_led_frame",
                   "l_gripper_motor_accelerometer_link", "l_gripper_tool_frame",
                   "l_gripper_motor_slider_link", "l_gripper_motor_screw_link",
                   "l_force_torque_link", "l_force_torque_adapter_link"]
        r_gripper_joints = ["r_gripper_l_finger_joint", "r_gripper_r_finger_joint"]
        r_gripper_links = ["r_gripper_l_finger_tip_link", "r_gripper_r_finger_tip_link",
                           "r_gripper_l_finger_link", "r_gripper_r_finger_link",
                           "r_gripper_l_finger_tip_frame", "r_gripper_palm_link"]
        l_gripper_joints = ["l_gripper_l_finger_joint", "l_gripper_r_finger_joint"]
        l_gripper_links = ["l_gripper_l_finger_tip_link", "l_gripper_r_finger_tip_link",
                           "l_gripper_l_finger_link", "l_gripper_r_finger_link",
                           "l_gripper_l_finger_tip_frame", "l_gripper_palm_link"]
        # Arms
        left = ChainDescription("left", l_joints, l_links)
        right = ChainDescription("right", r_joints, r_links)
        left_inter = InteractionDescription(left, "l_wrist_roll_link")
        right_inter = InteractionDescription(right, "r_wrist_roll_link")
        # Gripper
        left_gripper = GripperDescription("left_gripper", l_gripper_links, l_gripper_joints,
                                          gripper_meter_to_jnt_multiplier=5.0,
                                          gripper_minimal_position=0.013,
                                          gripper_convergence_delta=0.005)
        right_gripper = GripperDescription("right_gripper", r_gripper_links, r_gripper_joints,
                                           gripper_meter_to_jnt_multiplier=5.0,
                                           gripper_minimal_position=0.013,
                                           gripper_convergence_delta=0.005)
        # Adding Arm + Gripper
        left_arm = ManipulatorDescription(left_inter, tool_frame="l_gripper_tool_frame",
                                          gripper_description=left_gripper)
        right_arm = ManipulatorDescription(right_inter, tool_frame="r_gripper_tool_frame",
                                           gripper_description=right_gripper)
        self.add_chains({"left": left_arm, "right": right_arm})
        # Adding Static Joint Poses
        # Static Arm Positions
        r_arm_park = [-1.712, -0.256, -1.463, -2.12, 1.766, -0.07, 0.051]
        l_arm_park = [1.712, -0.264, 1.38, -2.12, 16.996, -0.073, 0.0]
        self.add_static_joint_chain("right", "park", r_arm_park)
        self.add_static_joint_chain("left", "park", l_arm_park)
        # Static Gripper Positions
        gripper_confs = {"open": [0.548, 0.548], "close": [0.0, 0.0]}
        self.add_static_gripper_chains("left", gripper_confs)
        self.add_static_gripper_chains("right", gripper_confs)

        self.grasps = GraspingDescription({"front": [0, 0, 0, 1],
                                           "left": [0, 0, -1, 1],
                                           "right": [0, 0, 1, 1],
                                           "top": [0, 1, 0, 1]})

        self.gripper_action_topics = {"left": "l_gripper_controller/gripper_action",
                                      "right": "r_gripper_controller/gripper_action"}

    def get_camera_frame(self, name="optical_frame"):
        # TODO: Hacky since only one optical camera frame from pr2 is used
        return super().get_camera_frame(name)
