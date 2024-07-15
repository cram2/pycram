import rospkg

from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    CameraDescription, RobotDescriptionManager
from ..datastructures.enums import GripperState, Arms, Grasp

rospack = rospkg.RosPack()
filename = rospack.get_path('pycram') + '/resources/robots/' + "stretch_description" + '.urdf'

stretch_description = RobotDescription("stretch_description", "base_link", "link_lift", "joint_lift", filename)

################################## Right Arm ##################################
arm_description = KinematicChainDescription("arm", "link_mast", "link_wrist_roll", stretch_description.urdf_object,
                                            arm_type=Arms.RIGHT)

arm_description.add_static_joint_states("park", {'joint_lift': 0.0,
                                                 'joint_arm_l3': 0.0,
                                                 'joint_arm_l2': 0.0,
                                                 'joint_arm_l1': 0.0,
                                                 'joint_arm_l0': 0.0,
                                                 'joint_wrist_yaw': 0.0,
                                                 'joint_wrist_pitch': 0.0,
                                                 'joint_wrist_roll': 0.0})

stretch_description.add_kinematic_chain_description(arm_description)

################################## Right Gripper ##################################
gripper_description = EndEffectorDescription("arm", "link_straight_gripper", "link_grasp_center",
                                             stretch_description.urdf_object)

gripper_description.add_static_joint_states(GripperState.OPEN, {'joint_gripper_finger_left': 0.59,
                                                                'joint_gripper_finger_right': 0.59})
gripper_description.add_static_joint_states(GripperState.CLOSE, {'joint_gripper_finger_left': 0.0,
                                                                 'joint_gripper_finger_right': 0.0})

arm_description.end_effector = gripper_description

################################## Neck ##################################
neck = KinematicChainDescription("neck", "link_head", "link_head_tilt", stretch_description.urdf_object)

stretch_description.add_kinematic_chain_description(neck)

################################## Camera ##################################
realsense_color = CameraDescription("camera_color_optical_frame", "camera_color_optical_frame", 1.322, 1.322)
realsense_depth = CameraDescription("camera_depth_optical_frame", "camera_depth_optical_frame", 1.307, 1.307)
realsense_infra1 = CameraDescription("camera_infra1_optical_frame", "camera_infra1_optical_frame", 1.307, 1.307)
realsense_infra2 = CameraDescription("camera_infra2_optical_frame", "camera_infra2_optical_frame", 1.257, 1.257)

stretch_description.add_camera_description(realsense_color)
stretch_description.add_camera_description(realsense_depth)
stretch_description.add_camera_description(realsense_infra1)
stretch_description.add_camera_description(realsense_infra2)

################################## Grasps ##################################
stretch_description.add_grasp_orientations({Grasp.FRONT: [0, 0, 0, 1],
                                            Grasp.LEFT: [0, 0, -1, 1],
                                            Grasp.RIGHT: [0, 0, 1, 1],
                                            Grasp.TOP: [0, 1, 0, 1]})

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(stretch_description)
