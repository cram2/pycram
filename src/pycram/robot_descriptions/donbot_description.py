import rospkg
from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager, CameraDescription
from ..datastructures.enums import Arms, Grasp, GripperState

rospack = rospkg.RosPack()
filename = rospack.get_path('pycram') + '/resources/robots/' + "iai_donbot" + '.urdf'

donbot_description = RobotDescription("iai_donbot", "base_link", "ur5_base_link", "arm_base_mounting_joint",
                                      filename)

################################## Right Arm ##################################
right_arm = KinematicChainDescription("right_arm", "ur5_base_link", "ur5_wrist_3_link", donbot_description.urdf_object, arm_type=Arms.RIGHT)

right_arm.add_static_joint_states("park", {'ur5_shoulder_pan_joint': 3.23,
                                           'ur5_shoulder_lift_joint': -1.51,
                                           'ur5_elbow_joint': -0.79,
                                           'ur5_wrist_1_joint': -2.33,
                                           'ur5_wrist_2_joint': 1.57,
                                           'ur5_wrist_3_joint': -1.65})

donbot_description.add_kinematic_chain_description(right_arm)

################################## Right Gripper ##################################

right_gripper = EndEffectorDescription("right_gripper", "gripper_base_link", "gripper_tool_frame",
                                       donbot_description.urdf_object)

right_gripper.add_static_joint_states(GripperState.OPEN, {'gripper_joint': 0.0})
right_gripper.add_static_joint_states(GripperState.CLOSE, {'gripper_joint': 0.2})

right_arm.end_effector = right_gripper

################################## Camera ##################################
camera = CameraDescription("camera_link", "camera_link", 0.75049, 0.5, 1.2)
donbot_description.add_camera_description(camera)

################################## Neck ##################################
donbot_description.add_kinematic_chain("neck", "ur5_base_link", "ur5_base_link")

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(donbot_description)
