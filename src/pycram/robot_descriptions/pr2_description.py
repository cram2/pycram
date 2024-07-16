from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager, CameraDescription
from ..datastructures.enums import Arms, Grasp, GripperState, GripperType
import rospkg

rospack = rospkg.RosPack()
filename = rospack.get_path('pycram') + '/resources/robots/' + "pr2" + '.urdf'

pr2_description = RobotDescription("pr2", "base_link", "torso_lift_link", "torso_lift_joint",
                                   filename)

################################## Left Arm ##################################
left_arm = KinematicChainDescription("left", "torso_lift_link", "l_wrist_roll_link",
                                     pr2_description.urdf_object, arm_type=Arms.LEFT)
left_arm.add_static_joint_states("park", {'l_shoulder_pan_joint': 1.712,
                                          'l_shoulder_lift_joint': -0.264,
                                          'l_upper_arm_roll_joint': 1.38,
                                          'l_elbow_flex_joint': -2.12,
                                          'l_forearm_roll_joint': 16.996,
                                          'l_wrist_flex_joint': -0.073,
                                          'l_wrist_roll_joint': 0.0})
pr2_description.add_kinematic_chain_description(left_arm)

################################## Left Gripper ##################################
left_gripper = EndEffectorDescription("left_gripper", "l_gripper_palm_link", "l_gripper_tool_frame",
                                      pr2_description.urdf_object)
left_gripper.add_static_joint_states(GripperState.OPEN, {'l_gripper_l_finger_joint': 0.548,
                                              'l_gripper_r_finger_joint': 0.548})
left_gripper.add_static_joint_states(GripperState.CLOSE, {'l_gripper_l_finger_joint': 0.0,
                                               'l_gripper_r_finger_joint': 0.0})
left_gripper.end_effector_type = GripperType.PARALLEL
left_gripper.opening_distance = 0.548
left_arm.end_effector = left_gripper

################################## Right Arm ##################################
right_arm = KinematicChainDescription("right", "torso_lift_link", "r_wrist_roll_link",
                                      pr2_description.urdf_object, arm_type=Arms.RIGHT)
right_arm.add_static_joint_states("park", {'r_shoulder_pan_joint': -1.712,
                                           'r_shoulder_lift_joint': -0.256,
                                           'r_upper_arm_roll_joint': -1.463,
                                           'r_elbow_flex_joint': -2.12,
                                           'r_forearm_roll_joint': 1.766,
                                           'r_wrist_flex_joint': -0.07,
                                           'r_wrist_roll_joint': 0.051})
pr2_description.add_kinematic_chain_description(right_arm)

################################## Right Gripper ##################################
right_gripper = EndEffectorDescription("right_gripper", "r_gripper_palm_link", "r_gripper_tool_frame",
                                       pr2_description.urdf_object)
right_gripper.add_static_joint_states(GripperState.OPEN, {'r_gripper_l_finger_joint': 0.548,
                                               'r_gripper_r_finger_joint': 0.548})
right_gripper.add_static_joint_states(GripperState.CLOSE, {'r_gripper_l_finger_joint': 0.0,
                                                'r_gripper_r_finger_joint': 0.0})
right_gripper.end_effector_type = GripperType.PARALLEL
right_gripper.opening_distance = 0.548
right_arm.end_effector = right_gripper


################################## Camera ##################################
camera = CameraDescription("kinect_camera", "wide_stereo_optical_frame", 1.27,
                           1.60, 0.99483, 0.75049)
pr2_description.add_camera_description(camera)

################################## Neck ##################################
pr2_description.add_kinematic_chain("neck", "head_pan_link", "head_tilt_link")

################################# Grasps ##################################
pr2_description.add_grasp_orientations({Grasp.FRONT: [0, 0, 0, 1],
                                        Grasp.LEFT: [0, 0, -1, 1],
                                        Grasp.RIGHT: [0, 0, 1, 1],
                                        Grasp.TOP: [0, 1, 0, 1]})

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(pr2_description)
