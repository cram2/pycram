from ..robot_description import *

class DonbotDescription(RobotDescription):

    def __init__(self):

        super().__init__("iai_donbot", "base_footprint", "base_link", "ur5_base_link",
                         "arm_base_mounting_joint", odom_frame="odom",
                         odom_joints=["odom_x_joint", "odom_y_joint", "odom_z_joint"])
        # Camera
        rgb_camera = CameraDescription("camera_link",
                                       horizontal_angle=0.99483, vertical_angle=0.75049,
                                       minimal_height=0.5, maximal_height=1.2)
        rs_camera = CameraDescription("rs_camera_link",
                                      horizontal_angle=0.99483, vertical_angle=0.75049,
                                      minimal_height=0.5, maximal_height=1.2)
        # not in the iai_donbot.urdf, although used in cram
        # realsense_camera = CameraDescription("rs_camera_depth_optical_frame",
        #                                     horizontal_angle=0.99483, vertical_angle=0.75049,
        #                                     minimal_height=0.5, maximal_height=1.2)
        # virtual_camera = CameraDescription("rs_camera_color_optical_frame",
        #                                   horizontal_angle=0.99483, vertical_angle=0.75049,
        #                                   minimal_height=0.5, maximal_height=1.2)
        self.add_cameras({"rgb_camera": rgb_camera, "rs_camera": rs_camera})
        # self.cameras["realsense_camera"] = realsense_camera
        # self.cameras["virtual_camera"] = virtual_camera
        # The axis which points away from the camera and along which the picture of the camera is created
        self.front_facing_axis = [0, 0, 1]

        # Gripper
        gripper_links = ["wrist_collision", "gripper_base_link", "gripper_finger_left_link",
                         "gripper_finger_right_link", "gripper_gripper_left_link", "gripper_gripper_right_link",
                         "marco_left_finger_link", "marco_right_finger_link"]
        gripper_joints = ["gripper_joint"]
        gripper = GripperDescription("gripper", gripper_links=gripper_links, gripper_joints=gripper_joints,
                                     gripper_meter_to_jnt_multiplier=1.0, gripper_minimal_position=0.0,
                                     gripper_convergence_delta=0.001)
        # Arm
        arm_links = ["ur5_shoulder_link", "ur5_upper_arm_link", "ur5_forearm_link", "ur5_wrist_1_link",
                     "ur5_wrist_2_link", "ur5_wrist_3_link"]
        arm_joints = ["ur5_shoulder_pan_joint", "ur5_shoulder_lift_joint", "ur5_elbow_joint",
                      "ur5_wrist_1_joint", "ur5_wrist_2_joint", "ur5_wrist_3_joint"]
        arm_chain = ChainDescription("left", arm_joints, arm_links)
        arm_inter = InteractionDescription(arm_chain, "ur5_ee_link")
        arm_manip = ManipulatorDescription(arm_inter, tool_frame="gripper_tool_frame",
                                           gripper_description=gripper)  # or ur5_tool0
        self.add_chain("left", arm_manip)
        self.add_chain("right", arm_manip)
        # Neck
        neck_base_link = "ur5_base_link"
        neck = ChainDescription("neck", arm_joints, arm_links, base_link=neck_base_link)
        self.add_chain("neck", neck)
        # Adding Static Joint Poses
        # Static Neck Positions
        down = [4.130944728851318, 0.04936718940734863, -1.9734209219561976,
                -1.7624157110797327, 1.6369260549545288, -1.6503327528582972]
        right = [1.6281344890594482, -1.1734271208392542, -1.1555221716510218,
                 # 2:-1.4734271208392542, 4: -0.9881671110736292,
                 -1.7555221716510218, 1.4996352195739746, -1.4276765028582972]
        left = [-1.6281344890594482, -1.1734271208392542, -1.1555221716510218,
                # 2:-1.4734271208392542, 4: -0.9881671110736292,
                -1.7555221716510218, 1.4996352195739746, -1.4276765028582972]
        right_separators = [1.4752850532531738, -1.4380276838885706, -1.9198325316058558,
                            -0.0680769125567835, 1.704722285270691, -1.5686963240252894]
        right_separators_preplace_state = [1.585883378982544, -0.49957687059511,
                                           -1.61414081255068, -1.1720898787127894,
                                           1.37771737575531, -1.3602331320392054]
        self.add_static_joint_chains("neck", {"down": down, "right": right, "left": left,
                                              "right_separators": right_separators,
                                              "right_separators_preplace_state": right_separators_preplace_state})
        # Static Arm Positions
        park = [3.234022855758667, -1.5068710486041468, -0.7870314756976526, -2.337625328694479,
                1.5699548721313477, -1.6504042784320276]
        self.add_static_joint_chain("left", "park", park)
        front = [-1.5, -1.5, 0, 1.5, -1.5, 1.5]
        arm_right = [-3, -1.5, 0, 1.5, -1.5, 1.5]
        back = [-4.7, -1.5, 0, 1.5, -1.5, 1.5]
        arm_left = [0, -1.5, 0, 1.5, -1.5, 1.5]
        self.add_static_joint_chains("left", {"front": front, "arm_right": arm_right, "back": back,
                                              "arm_left": arm_left})

    def get_camera_frame(self, name="rgb_camera"):
        # TODO: Hacky since only one optical camera frame from pr2 is used
        return super().get_camera_frame(name)
