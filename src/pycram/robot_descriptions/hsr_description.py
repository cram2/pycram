from ..robot_description import *


class HSRDescription(RobotDescription):

    def __init__(self):

        super().__init__("hsrb", "base_footprint", "base_link", "arm_lift_link", "arm_lift_joint")
        # Camera
        head_center_camera = CameraDescription("head_center_camera_frame",
                                               horizontal_angle=0.99483, vertical_angle=0.75049)
        head_r_camera = CameraDescription("head_r_stereo_camera_link",
                                          horizontal_angle=0.99483, vertical_angle=0.75049)
        head_l_camera = CameraDescription("head_r_stereo_camera_link",
                                          horizontal_angle=0.99483, vertical_angle=0.75049)
        head_rgbd_camera = CameraDescription("head_rgbd_sensor_link",
                                             horizontal_angle=0.99483, vertical_angle=0.75049)
        hand_camera = CameraDescription("hand_camera_frame",
                                        horizontal_angle=0.99483, vertical_angle=0.75049)
        self.add_cameras({"head_center_camera": head_center_camera, "head_rgbd_camera": head_rgbd_camera,
                          "head_l_camera": head_l_camera, "head_r_camera": head_r_camera,
                          "hand_camera": hand_camera})
        # The axis which points away from the camera and along which the picture of the camera is created
        self.front_facing_axis = [0, 0, 1]
        # Neck
        neck_links = ["head_pan_link", "head_tilt_link"]
        neck_joints = ["head_pan_joint", "head_tilt_joint"]
        neck_forward = {"forward": [0.0, 0.0], "down": [0.0, -0.7]}
        neck_chain = ChainDescription("neck", neck_joints, neck_links, static_joint_states=neck_forward)
        self.add_chain("neck", neck_chain)
        # Arm
        arm_joints = ["arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        arm_links = ["arm_flex_link", "arm_roll_link", "wrist_flex_link", "wrist_roll_link"]
        arm_carry = {"park": [0, 1.5, -1.85, 0]}
        gripper_links = ["hand_l_distal_link", "hand_l_spring_proximal_link", "hand_palm_link",
                         "hand_r_distal_link", "hand_r_spring_proximal_link"]
        gripper_joints = ["hand_motor_joint"]
        gripper = GripperDescription("gripper", gripper_links=gripper_links, gripper_joints=gripper_joints,
                                     gripper_meter_to_jnt_multiplier=1.0, gripper_minimal_position=0.0,
                                     gripper_convergence_delta=0.001)
        arm_chain = ChainDescription("left", arm_joints, arm_links, static_joint_states=arm_carry)
        arm_inter = InteractionDescription(arm_chain, "wrist_roll_link")
        arm_manip = ManipulatorDescription(arm_inter, tool_frame="gripper_tool_frame", gripper_description=gripper)
        self.add_chain("left", arm_manip)
        self.add_static_gripper_chains("left", {"open": [0.3], "close": [0.0]})

    def get_camera_frame(self, name="head_center_camera"):
        # TODO: Hacky since only one optical camera frame from pr2 is used
        return super().get_camera_frame(name)