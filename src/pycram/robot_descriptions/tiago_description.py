import rospkg
from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager, CameraDescription
from ..datastructures.enums import GripperState, Arms, Grasp

rospack = rospkg.RosPack()
filename = rospack.get_path('pycram') + '/resources/robots/' + "tiago_dual" + '.urdf'

tiago_description = RobotDescription("tiago_dual", "base_link", "torso_lift_link", "torso_lift_joint",
                                     filename)

################################## Left Arm ##################################
left_arm = KinematicChainDescription("left_arm", "torso_lift_link", "arm_left_7_link",
                                     tiago_description.urdf_object, arm_type=Arms.LEFT)

left_arm.add_static_joint_states("park", {'arm_left_1_joint': 0.27,
                                          'arm_left_2_joint': -1.07,
                                          'arm_left_3_joint': 1.5,
                                          'arm_left_4_joint': 1.96,
                                          'arm_left_5_joint': -2.0,
                                          'arm_left_6_joint': 1.2,
                                          'arm_left_7_joint': 0.5})

tiago_description.add_kinematic_chain_description(left_arm)

################################## Left Gripper ##################################
left_gripper = EndEffectorDescription("left_gripper", "gripper_left_link", "gripper_left_tool_link",
                                      tiago_description.urdf_object)

left_gripper.add_static_joint_states(GripperState.OPEN, {'gripper_left_left_finger_joint': 0.048,
                                                         'gripper_left_right_finger_joint': 0.048})

left_gripper.add_static_joint_states(GripperState.CLOSE, {'gripper_left_left_finger_joint': 0.0,
                                                          'gripper_left_right_finger_joint': 0.0})

left_arm.end_effector = left_gripper

################################## Right Arm ##################################
right_arm = KinematicChainDescription("right_arm", "torso_lift_link", "arm_right_7_link",
                                      tiago_description.urdf_object, arm_type=Arms.RIGHT)

right_arm.add_static_joint_states("park", {'arm_right_1_joint': 0.27,
                                           'arm_right_2_joint': -1.07,
                                           'arm_right_3_joint': 1.5,
                                           'arm_right_4_joint': 2.0,
                                           'arm_right_5_joint': -2.0,
                                           'arm_right_6_joint': 1.2,
                                           'arm_right_7_joint': 0.5})

tiago_description.add_kinematic_chain_description(right_arm)

################################## Right Gripper ##################################
right_gripper = EndEffectorDescription("right_gripper", "gripper_right_link", "gripper_right_tool_link",
                                       tiago_description.urdf_object)

right_gripper.add_static_joint_states(GripperState.OPEN, {'gripper_right_left_finger_joint': 0.048,
                                                          'gripper_right_right_finger_joint': 0.048})

right_gripper.add_static_joint_states(GripperState.CLOSE, {'gripper_right_left_finger_joint': 0.0,
                                                           'gripper_right_right_finger_joint': 0.0})

right_arm.end_effector = right_gripper

################################## Camera ##################################
camera = CameraDescription("xtion_optical_frame", "xtion_optical_frame", 0.99483, 0.75049, 1.0665, 1.4165)
tiago_description.add_camera_description(camera)

################################## Neck ##################################
tiago_description.add_kinematic_chain("neck", "torso_lift_link", "head_2_link")

################################# Grasps ##################################
tiago_description.add_grasp_orientations({Grasp.FRONT: [0, 0, 0, 1],
                                          Grasp.LEFT: [0, 0, -1, 1],
                                          Grasp.RIGHT: [0, 0, 1, 1],
                                          Grasp.TOP: [0, 1, 0, 1]})

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(tiago_description)
