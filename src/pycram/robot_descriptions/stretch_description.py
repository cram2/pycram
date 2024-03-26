from ..robot_description import *
import numpy as np
import tf


class StretchDescription(RobotDescription):

    def __init__(self):
        super().__init__("stretch", "base_link", "base_link", "link_lift", "joint_lift")

        realsense_color = CameraDescription('camera_color_optical_frame', horizontal_angle=1.047, vertical_angle=0.785,
                                            minimal_height=1.322, maximal_height=1.322)

        realsense_depth = CameraDescription('camera_depth_optical_frame', horizontal_angle=1.047, vertical_angle=0.785,
                                            minimal_height=1.307, maximal_height=1.307)

        realsense_infra1 = CameraDescription('camera_infra1_optical_frame', horizontal_angle=1.047,
                                             vertical_angle=0.785,
                                             minimal_height=1.307, maximal_height=1.307)
        realsense_infra2 = CameraDescription('camera_infra2_optical_frame', horizontal_angle=1.047,
                                             vertical_angle=0.785,
                                             minimal_height=1.257, maximal_height=1.257)
        self.add_cameras(
            {'color': realsense_color, 'depth': realsense_depth, 'infra1': realsense_infra1,
             'infra2': realsense_infra2})

        self.front_facing_axis = [0, 0, 1]

        neck = ChainDescription("neck", ["joint_head_pan", "joint_head_tilt"],
                                ["link_head_pan", "link_head_tilt"])
        self.add_chain("neck", neck)

        arm_joints = ['joint_lift','joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0', "joint_wrist_yaw"]
        arm_links = ['link_lift', 'link_arm_l3', 'link_arm_l2', 'link_arm_l1', 'link_arm_l0', "link_wrist_yaw"]

        arm_chain_desc = ChainDescription("arm", arm_joints, arm_links)
        arm_inter_desc = InteractionDescription(arm_chain_desc, "link_wrist_yaw")

        gripper_links = ['link_gripper_finger_left', 'link_gripper_fingertip_left',
                         'link_gripper_finger_right', 'link_gripper_fingertip_right', 'link_grasp_center']
        gripper_joints = ['joint_gripper_finger_left', 'joint_gripper_fingertip_left',
                          'joint_gripper_finger_right', 'joint_gripper_fingertip_right', 'joint_grasp_center']
        arm_gripper_desc = GripperDescription("gripper", gripper_links, gripper_joints,
                                              gripper_meter_to_jnt_multiplier=5.0,
                                              gripper_minimal_position=0.013,
                                              gripper_convergence_delta=0.005)

        arm_desc = ManipulatorDescription(arm_inter_desc, tool_frame="link_grasp_center", gripper_description=arm_gripper_desc)
        self.add_chain("arm", arm_desc)

        arm_park = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.add_static_joint_chain("arm", "park", arm_park)

        gripper_confs = {"open": [0.59, 0.59], "close": [0.0, 0.0]}
        self.add_static_gripper_chains("arm", gripper_confs)

        self.grasps = GraspingDescription({"front": [0, 0, 0, 1],
                                           "left": [0, 0, -1, 1],
                                           "right": [0, 0, 1, 1],
                                           "top": [0, 1, 0, 1]})

    def get_camera_frame(self, name="color"):
        return super().get_camera_frame(name)

    @staticmethod
    def stretch_orientation_generator(position, origin):
        angle = np.arctan2(position[1] - origin.position.y, position[0] - origin.position.x) + np.pi
        quaternion = list(tf.transformations.quaternion_from_euler(0, 0, angle + np.pi / 2, axes="sxyz"))
        return quaternion
