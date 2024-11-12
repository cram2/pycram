import numpy as np
from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager, CameraDescription
from ..datastructures.enums import Arms, Grasp, GripperState, GripperType, TorsoState
import rospkg

# Get the path to the Fetch robot URDF file
rospack = rospkg.RosPack()
filename = rospack.get_path('fetch_description') + '/robots/fetch.urdf'

# Initialize the Fetch robot description
fetch_description = RobotDescription("fetch", "base_link", "torso_lift_link", "torso_lift_joint",
                                     filename)

################################## Left Arm ##################################
left_arm = KinematicChainDescription("left", "torso_lift_link", "wrist_roll_link",
                                     fetch_description.urdf_object, arm_type=Arms.LEFT)
left_arm.add_static_joint_states("park", {'shoulder_pan_joint': 1.6056,
                                          'shoulder_lift_joint': 1.518,
                                          'upperarm_roll_joint': 0.0,
                                          'elbow_flex_joint': 1.625,
                                          'forearm_roll_joint': 0.0,
                                          'wrist_flex_joint': 1.6,
                                          'wrist_roll_joint': 0.0})
fetch_description.add_kinematic_chain_description(left_arm)

################################## Gripper ##################################
gripper = EndEffectorDescription("gripper", "wrist_roll_link", "gripper_link",
                                 fetch_description.urdf_object)
gripper.add_static_joint_states(GripperState.OPEN, {'l_gripper_finger_joint': 0.04,
                                                    'r_gripper_finger_joint': 0.04})
gripper.add_static_joint_states(GripperState.CLOSE, {'l_gripper_finger_joint': 0.0,
                                                     'r_gripper_finger_joint': 0.0})
gripper.end_effector_type = GripperType.PARALLEL
gripper.opening_distance = 0.08  # Adjusted for Fetch gripper
left_arm.end_effector = gripper

################################## Torso ##################################
torso = KinematicChainDescription("torso", "base_link", "torso_lift_link",
                                  fetch_description.urdf_object)
torso.add_static_joint_states(TorsoState.HIGH, {"torso_lift_joint": 0.385})
torso.add_static_joint_states(TorsoState.MID, {"torso_lift_joint": 0.2})
torso.add_static_joint_states(TorsoState.LOW, {"torso_lift_joint": 0.0})
fetch_description.add_kinematic_chain_description(torso)

################################## Camera ##################################
camera = CameraDescription("head_camera", "head_camera_rgb_frame", 1.1,
                           1.49, 0.99483, 0.75049, [1, 0, 0])
fetch_description.add_camera_description(camera)

################################## Neck ##################################
fetch_description.add_kinematic_chain("neck", "torso_lift_link", "head_tilt_link")
fetch_description.set_neck("head_pan_joint", "head_tilt_joint")


################################# Grasps ##################################
gripper.generate_all_grasp_orientations_from_front_grasp([0, 0, 0, 1])

################################## Additionals ##################################
fetch_description.set_max_reach("torso_lift_link", "gripper_link")
fetch_description.set_palm_axis([1, 0, 0])

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(fetch_description)
