from ..datastructures.dataclasses import VirtualMobileBaseJoints
from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager, CameraDescription
from ..datastructures.enums import Arms, Grasp, GripperState, GripperType
from ..ros.ros_tools import get_ros_package_path

from ..helper import get_robot_mjcf_path
import icub_models

filename = get_ros_package_path('icub_model') + '/urdf/model' + '.urdf'


icub_description = RobotDescription("icub",
                                    "root_link",
                                    "torso_1",
                                    "torso_pitch",
                                    filename)

################################## Left Arm ##################################
left_arm = KinematicChainDescription("left", "chest", "l_hand",
                                     icub_description.urdf_object, arm_type=Arms.LEFT)
left_arm.add_static_joint_states("park", {'l_shoulder_pitch': -80.0,
                                          'l_shoulder_roll': 70.0,
                                          'l_shoulder_yaw': 30.0,
                                          'l_elbow': 70.0,
                                          'l_wrist_prosup': -25.0,
                                          'l_wrist_pitch': 0.0,
                                          'l_wrist_yaw': 0.0})

icub_description.add_kinematic_chain_description(left_arm)



################################## Left Gripper ##################################
left_gripper = EndEffectorDescription("left_gripper", "l_hand", "l_hand",
                                      icub_description.urdf_object)

left_arm.end_effector = left_gripper



################################## Right Arm ##################################
right_arm = KinematicChainDescription("right", "chest", "r_hand",
                                      icub_description.urdf_object, arm_type=Arms.RIGHT)
right_arm.add_static_joint_states("park", {'r_shoulder_pitch': 80.0,
                                          'r_shoulder_roll': 70.0,
                                          'r_shoulder_yaw': 30.0,
                                          'r_elbow': 70.0,
                                          'r_wrist_prosup': -25.0,
                                          'r_wrist_pitch': 0.0,
                                          'r_wrist_yaw': 0.0})

icub_description.add_kinematic_chain_description(right_arm)


################################## Right Gripper ##################################
right_gripper = EndEffectorDescription("right_gripper", "r_hand", "r_hand",
                                       icub_description.urdf_object)


right_arm.end_effector = right_gripper



################################## Camera ##################################
camera = CameraDescription("left_camera", "eyes_tilt_frame", 1.27,
                           1.60, 0.99483, 0.75049)
icub_description.add_camera_description(camera)

################################## Neck ##################################
icub_description.add_kinematic_chain("neck", "neck_1", "head")

################################# Grasps ##################################
icub_description.add_grasp_orientations({Grasp.FRONT: [0, 0, 0, 1],
                                         Grasp.LEFT: [0, 0, -1, 1],
                                         Grasp.RIGHT: [0, 0, 1, 1],
                                         Grasp.TOP: [0, 1, 0, 1]})

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(icub_description)
