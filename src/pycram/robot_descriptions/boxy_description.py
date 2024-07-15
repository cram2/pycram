import rospkg
from ..robot_description import RobotDescription, CameraDescription, KinematicChainDescription, \
    EndEffectorDescription, RobotDescriptionManager
from ..datastructures.enums import Arms, Grasp, GripperState

rospack = rospkg.RosPack()
filename = rospack.get_path('pycram') + '/resources/robots/' + "boxy" + '.urdf'

boxy_description = RobotDescription("boxy", "base_link", "triangle_base_link", "triangle_base_joint",
                                    filename)

################################## Right Arm ##################################
right_arm = KinematicChainDescription("right_arm", "calib_right_arm_base_link", "right_arm_7_link",
                                      boxy_description.urdf_object, arm_type=Arms.RIGHT)

right_arm.add_static_joint_states("park", {"right_arm_0_joint": 1.858,
                                           "right_arm_1_joint": -0.70571,
                                           "right_arm_2_joint": -0.9614,
                                           "right_arm_3_joint": 0.602,
                                           "right_arm_4_joint": 2.5922,
                                           "right_arm_5_joint": 1.94065,
                                           "right_arm_6_joint": 1.28735})

boxy_description.add_kinematic_chain_description(right_arm)

################################## Right Gripper ##################################

right_gripper = EndEffectorDescription("right_gripper", "right_gripper_base_link", "right_gripper_tool_frame",
                                       boxy_description.urdf_object)

right_gripper.add_static_joint_states(GripperState.OPEN, {"right_gripper_joint": 0.548})
right_gripper.add_static_joint_states(GripperState.CLOSE, {"right_gripper_joint": 0.0})

right_arm.end_effector = right_gripper

################################## Left Arm ##################################
left_arm = KinematicChainDescription("left_arm", "calib_left_arm_base_link", "left_arm_7_link",
                                     boxy_description.urdf_object, arm_type=Arms.LEFT)

left_arm.add_static_joint_states("park", {"left_arm_0_joint": -1.858,
                                          "left_arm_1_joint": 0.70571,
                                          "left_arm_2_joint": 0.9614,
                                          "left_arm_3_joint": -0.602,
                                          "left_arm_4_joint": -2.5922,
                                          "left_arm_5_joint": -1.94065,
                                          "left_arm_6_joint": -1.28735})

boxy_description.add_kinematic_chain_description(left_arm)

################################## Left Gripper ##################################

left_gripper = EndEffectorDescription("left_gripper", "left_gripper_base_link", "left_gripper_tool_frame",
                                      boxy_description.urdf_object)

left_gripper.add_static_joint_states(GripperState.OPEN, {"left_gripper_joint": 0.548})
left_gripper.add_static_joint_states(GripperState.CLOSE, {"left_gripper_joint": 0.0})

left_arm.end_effector = left_gripper

################################## Camera ##################################
camera = CameraDescription("head_mount_kinect2_rgb_optical_frame", "head_mount_kinect2_rgb_optical_frame",
                           2.5, 0.99483, 0.75049)
boxy_description.add_camera_description(camera)

################################## Neck ##################################
boxy_description.add_kinematic_chain("neck", "neck_base_link", "neck_wrist_3_link")

################################# Grasps ##################################
boxy_description.add_grasp_orientations({Grasp.LEFT: [1, 0, 0, 1],
                                         Grasp.TOP: [1, 1, 0, 0],
                                         Grasp.RIGHT: [0, 1, 1, 0],
                                         Grasp.FRONT: [1, 0, 1, 0]})

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(boxy_description)
