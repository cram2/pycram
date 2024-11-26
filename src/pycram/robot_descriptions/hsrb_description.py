import rospkg

from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager, CameraDescription
from ..datastructures.enums import GripperState, Grasp, Arms, TorsoState

rospack = rospkg.RosPack()
filename = rospack.get_path('pycram') + '/resources/robots/' + "hsrb" + '.urdf'

hsrb_description = RobotDescription("hsrb", "base_link", "arm_lift_link", "arm_lift_joint",
                                    filename)

################################## Left Arm ##################################
left_arm = KinematicChainDescription("left_arm", "arm_lift_link", "hand_palm_link",
                                     hsrb_description.urdf_object, arm_type=Arms.LEFT)

left_arm.add_static_joint_states("park", {'arm_flex_joint': 0.0,
                                          'arm_roll_joint': 1.5,
                                          'wrist_flex_joint': -1.85,
                                          'wrist_roll_joint': 0.0})
hsrb_description.add_kinematic_chain_description(left_arm)

################################## Left Gripper ##################################
left_gripper = EndEffectorDescription("left_gripper", "wrist_roll_link", "hand_gripper_tool_frame",
                                      hsrb_description.urdf_object)

left_gripper.add_static_joint_states(GripperState.OPEN, {'hand_l_proximal_joint': 0.3,
                                                         'hand_r_proximal_joint': 0.3,
                                                         'hand_motor_joint': 0.3})
left_gripper.add_static_joint_states(GripperState.CLOSE, {'hand_l_proximal_joint': 0.0,
                                                          'hand_r_proximal_joint': 0.0,
                                                          'hand_motor_joint': 0.0})

left_arm.end_effector = left_gripper

################################## Camera ##################################

head_center_camera = CameraDescription("head_center_camera_frame", "head_center_camera_frame", 0.99483, 0.75049)
head_r_camera = CameraDescription("head_r_stereo_camera_link", "head_r_stereo_camera_link", 0.99483, 0.75049)
head_l_camera = CameraDescription("head_l_stereo_camera_link", "head_l_stereo_camera_link", 0.99483, 0.75049)
head_rgbd_camera = CameraDescription("head_rgbd_sensor_link", "head_rgbd_sensor_link", 0.99483, 0.75049)
hand_camera = CameraDescription("hand_camera_frame", "hand_camera_frame", 0.99483, 0.75049)

hsrb_description.add_camera_description(head_center_camera)
hsrb_description.add_camera_description(head_r_camera)
hsrb_description.add_camera_description(head_l_camera)
hsrb_description.add_camera_description(head_rgbd_camera)
hsrb_description.add_camera_description(hand_camera)

################################## Neck ##################################
neck = KinematicChainDescription("neck", "head_pan_link", "head_tilt_link",
                                 hsrb_description.urdf_object)
hsrb_description.set_neck("head_pan_joint", "head_tilt_joint")


################################## Torso ##################################
torso = KinematicChainDescription("torso", "base_link", "torso_lift_link",
                                  hsrb_description.urdf_object)

torso.add_static_joint_states(TorsoState.HIGH, {"torso_lift_joint": 0.34})

torso.add_static_joint_states(TorsoState.MID, {"torso_lift_joint": 0.17})

torso.add_static_joint_states(TorsoState.LOW, {"torso_lift_joint": 0})

hsrb_description.add_kinematic_chain_description(torso)

################################## Grasps ##################################
left_gripper.add_grasp_orientations({Grasp.FRONT: [-1, 0, -1, 0],
                                     Grasp.LEFT: [0, -1, 1, 0],
                                     Grasp.RIGHT: [0, -1, -1, 0.0],
                                     Grasp.TOP: [-1, 0, 0, 0]})

hsrb_description.add_kinematic_chain_description(neck)

############################## Additionals ################################
# hsrb_description.set_costmap_offset(0.3)  # set to correct offset if needed
# hsrb_description.set_palm_axis([0, 0, -1])  # set to correct axis if needed
# hsrb_description.set_max_reach("torso_lift_link", "gripper_left_tool_link")  # set to correct links if needed

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(hsrb_description)
