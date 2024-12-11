import math

from ..datastructures.dataclasses import VirtualMobileBaseJoints
from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager, CameraDescription
from ..datastructures.enums import Arms, Grasp, GripperState, GripperType
from ..ros.ros_tools import get_ros_package_path

from ..helper import get_robot_mjcf_path

filename = get_ros_package_path('icub_model') + '/urdf/model' + '.urdf'

actuated_joints = {
        "torso": ["torso_yaw", "torso_roll", "torso_pitch"],
        "right_arm": ["r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw",
                      "r_elbow",
                      "r_wrist_prosup","r_wrist_pitch","r_wrist_yaw",
                      "r_hand_finger",
                      "r_thumb_oppose","r_thumb_proximal","r_thumb_distal",
                      "r_index_proximal","r_index_distal",
                      "r_middle_proximal","r_middle_distal",
                      "r_pinky"],
        "left_arm": ["l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw",
                     "l_elbow",
                     "l_wrist_prosup","l_wrist_pitch","l_wrist_yaw",
                     "l_hand_finger",
                     "l_thumb_oppose","l_thumb_proximal","l_thumb_distal",
                     "l_index_proximal","l_index_distal",
                     "l_middle_proximal","l_middle_distal",
                     "l_pinky"],
        "head":["neck_pitch","neck_roll","neck_yaw",
                "eyes_tilt","eyes_version","eyes_vergence"],
    }


icub_description = RobotDescription("icub",
                                    "root_link",
                                    "torso_1",
                                    "torso_pitch",
                                    filename,
                                    actuated_joints=actuated_joints)

################################## Left Arm ##################################
left_arm = KinematicChainDescription("left_arm", "chest", "l_hand",
                                     icub_description.urdf_object, arm_type=Arms.LEFT)
left_arm.add_static_joint_states("park", {'l_shoulder_pitch': math.radians(-80.0),
                                          'l_shoulder_roll': math.radians(70.0),
                                          'l_shoulder_yaw': math.radians(30.0),
                                          'l_elbow': math.radians(70.0),
                                          'l_wrist_prosup': math.radians(-25.0),
                                          'l_wrist_pitch': math.radians(0.0),
                                          'l_wrist_yaw': math.radians(0.0)})

icub_description.add_kinematic_chain_description(left_arm)



################################## Left Gripper ##################################
left_gripper = EndEffectorDescription("left_gripper", "l_hand", "l_hand",
                                      icub_description.urdf_object)

left_gripper.add_static_joint_states(GripperState.CLOSE, {'l_hand_index_0_joint': math.radians(-20.0),
                                                          'l_hand_index_1_joint': math.radians(20.0),
                                                          'l_hand_index_2_joint': math.radians(42.5),
                                                          'l_hand_index_3_joint': math.radians(42.5),
                                                          'l_hand_little_0_joint': math.radians(0.0),
                                                          'l_hand_little_1_joint': math.radians(28.0),
                                                          'l_hand_little_2_joint': math.radians(28.0),
                                                          'l_hand_little_3_joint': math.radians(28.0),
                                                          'l_hand_middle_0_joint': math.radians(0.0),
                                                          'l_hand_middle_1_joint': math.radians(20.0),
                                                          'l_hand_middle_2_joint': math.radians(42.5),
                                                          'l_hand_middle_3_joint': math.radians(42.5),
                                                          'l_hand_ring_0_joint': math.radians(0.0),
                                                          'l_hand_ring_1_joint': math.radians(28.0),
                                                          'l_hand_ring_2_joint': math.radians(28.0),
                                                          'l_hand_ring_3_joint': math.radians(28.0),
                                                          'l_hand_thumb_0_joint': math.radians(60.0),
                                                          'l_hand_thumb_1_joint': math.radians(0.0),
                                                          'l_hand_thumb_2_joint': math.radians(42.5),
                                                          'l_hand_thumb_3_joint': math.radians(42.5)})

left_gripper.add_static_joint_states(GripperState.OPEN, {'l_hand_index_0_joint': math.radians(0.0),
                                                          'l_hand_index_1_joint': math.radians(0.0),
                                                          'l_hand_index_2_joint': math.radians(0.0),
                                                          'l_hand_index_3_joint': math.radians(0.0),
                                                          'l_hand_little_0_joint': math.radians(20.0),
                                                          'l_hand_little_1_joint': math.radians(0.0),
                                                          'l_hand_little_2_joint': math.radians(0.0),
                                                          'l_hand_little_3_joint': math.radians(0.0),
                                                          'l_hand_middle_0_joint': math.radians(0.0),
                                                          'l_hand_middle_1_joint': math.radians(0.0),
                                                          'l_hand_middle_2_joint': math.radians(0.0),
                                                          'l_hand_middle_3_joint': math.radians(0.0),
                                                          'l_hand_ring_0_joint': math.radians(20.0),
                                                          'l_hand_ring_1_joint': math.radians(0.0),
                                                          'l_hand_ring_2_joint': math.radians(0.0),
                                                          'l_hand_ring_3_joint': math.radians(0.0),
                                                          'l_hand_thumb_0_joint': math.radians(0.0),
                                                          'l_hand_thumb_1_joint': math.radians(0.0),
                                                          'l_hand_thumb_2_joint': math.radians(0.0),
                                                          'l_hand_thumb_3_joint': math.radians(0.0)})

left_arm.end_effector = left_gripper



################################## Right Arm ##################################
right_arm = KinematicChainDescription("right_arm", "chest", "r_hand",
                                      icub_description.urdf_object, arm_type=Arms.RIGHT)
right_arm.add_static_joint_states("park", {'r_shoulder_pitch': math.radians(-80.0),
                                          'r_shoulder_roll': math.radians(70.0),
                                          'r_shoulder_yaw': math.radians(30.0),
                                          'r_elbow': math.radians(70.0),
                                          'r_wrist_prosup': math.radians(-25.0),
                                          'r_wrist_pitch': math.radians(0.0),
                                          'r_wrist_yaw': math.radians(0.0)})

icub_description.add_kinematic_chain_description(right_arm)


################################## Right Gripper ##################################
right_gripper = EndEffectorDescription("right_gripper", "r_hand", "r_hand",
                                       icub_description.urdf_object)


right_gripper.add_static_joint_states(GripperState.CLOSE, {'r_hand_index_0_joint': math.radians(-20.0),
                                                          'r_hand_index_1_joint': math.radians(20.0),
                                                          'r_hand_index_2_joint': math.radians(42.5),
                                                          'r_hand_index_3_joint': math.radians(42.5),
                                                          'r_hand_little_0_joint': math.radians(0.0),
                                                          'r_hand_little_1_joint': math.radians(28.0),
                                                          'r_hand_little_2_joint': math.radians(28.0),
                                                          'r_hand_little_3_joint': math.radians(28.0),
                                                          'r_hand_middle_0_joint': math.radians(0.0),
                                                          'r_hand_middle_1_joint': math.radians(20.0),
                                                          'r_hand_middle_2_joint': math.radians(42.5),
                                                          'r_hand_middle_3_joint': math.radians(42.5),
                                                          'r_hand_ring_0_joint': math.radians(0.0),
                                                          'r_hand_ring_1_joint': math.radians(28.0),
                                                          'r_hand_ring_2_joint': math.radians(28.0),
                                                          'r_hand_ring_3_joint': math.radians(28.0),
                                                          'r_hand_thumb_0_joint': math.radians(60.0),
                                                          'r_hand_thumb_1_joint': math.radians(0.0),
                                                          'r_hand_thumb_2_joint': math.radians(42.5),
                                                          'r_hand_thumb_3_joint': math.radians(42.5)})

right_gripper.add_static_joint_states(GripperState.OPEN, {'r_hand_index_0_joint': math.radians(0.0),
                                                          'r_hand_index_1_joint': math.radians(0.0),
                                                          'r_hand_index_2_joint': math.radians(0.0),
                                                          'r_hand_index_3_joint': math.radians(0.0),
                                                          'r_hand_little_0_joint': math.radians(20.0),
                                                          'r_hand_little_1_joint': math.radians(0.0),
                                                          'r_hand_little_2_joint': math.radians(0.0),
                                                          'r_hand_little_3_joint': math.radians(0.0),
                                                          'r_hand_middle_0_joint': math.radians(0.0),
                                                          'r_hand_middle_1_joint': math.radians(0.0),
                                                          'r_hand_middle_2_joint': math.radians(0.0),
                                                          'r_hand_middle_3_joint': math.radians(0.0),
                                                          'r_hand_ring_0_joint': math.radians(20.0),
                                                          'r_hand_ring_1_joint': math.radians(0.0),
                                                          'r_hand_ring_2_joint': math.radians(0.0),
                                                          'r_hand_ring_3_joint': math.radians(0.0),
                                                          'r_hand_thumb_0_joint': math.radians(0.0),
                                                          'r_hand_thumb_1_joint': math.radians(0.0),
                                                          'r_hand_thumb_2_joint': math.radians(0.0),
                                                          'r_hand_thumb_3_joint': math.radians(0.0)})
right_arm.end_effector = right_gripper



################################## Camera ##################################
left_camera = CameraDescription("left_camera", "l_eye", 1.27,
                           1.60, 0.99483, 0.75049)
icub_description.add_camera_description(left_camera)

right_camera = CameraDescription("right_camera", "r_eye", 1.27,
                           1.60, 0.99483, 0.75049)
icub_description.add_camera_description(right_camera)

################################## Neck ##################################
icub_description.add_kinematic_chain("neck", "chest", "head")


################################## l_fingers ##################################
icub_description.add_kinematic_chain("l_index", "l_hand", "l_hand_index_tip")
icub_description.add_kinematic_chain("l_little", "l_hand", "l_hand_little_tip")
icub_description.add_kinematic_chain("l_middle", "l_hand", "l_hand_middle_tip")
icub_description.add_kinematic_chain("l_ring", "l_hand", "l_hand_ring_tip")
icub_description.add_kinematic_chain("l_thumb", "l_hand", "l_hand_thumb_tip")


################################## r_ringers ##################################
icub_description.add_kinematic_chain("r_index", "r_hand", "r_hand_index_tip")
icub_description.add_kinematic_chain("r_little", "r_hand", "r_hand_little_tip")
icub_description.add_kinematic_chain("r_middle", "r_hand", "r_hand_middle_tip")
icub_description.add_kinematic_chain("r_ring", "r_hand", "r_hand_ring_tip")
icub_description.add_kinematic_chain("r_thumb", "r_hand", "r_hand_thumb_tip")


################################## legs ##################################
icub_description.add_kinematic_chain("l_leg", "root_link", "l_foot")
icub_description.add_kinematic_chain("r_leg", "root_link", "r_foot")

################################# Grasps ##################################
icub_description.add_grasp_orientations({Grasp.FRONT: [0, 0, 0, 1],
                                         Grasp.LEFT: [0, 0, -1, 1],
                                         Grasp.RIGHT: [0, 0, 1, 1],
                                         Grasp.TOP: [0, 1, 0, 1]})

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(icub_description)
