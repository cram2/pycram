import rospkg
from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager, CameraDescription
from ..datastructures.enums import Arms, Grasp, GripperState, TorsoState

rospack = rospkg.RosPack()
filename = rospack.get_path('pycram') + '/resources/robots/' + "iai_donbot" + '.urdf'

donbot_description = RobotDescription("iai_donbot", "base_footprint", "ur5_base_link", "arm_base_mounting_joint",
                                      filename)

################################## Right Arm ##################################
left_arm = KinematicChainDescription("left_arm", "ur5_base_link", "ur5_wrist_3_link", donbot_description.urdf_object,
                                     arm_type=Arms.LEFT)

left_arm.add_static_joint_states("park", {'ur5_shoulder_pan_joint': 3.23,
                                          'ur5_shoulder_lift_joint': -1.51,
                                          'ur5_elbow_joint': -0.79,
                                          'ur5_wrist_1_joint': -2.33,
                                          'ur5_wrist_2_joint': 1.57,
                                          'ur5_wrist_3_joint': -1.65})

donbot_description.add_kinematic_chain_description(left_arm)

################################## Right Gripper ##################################

left_gripper = EndEffectorDescription("left_gripper", "gripper_base_link", "gripper_tool_frame",
                                      donbot_description.urdf_object)

left_gripper.add_static_joint_states(GripperState.OPEN, {'gripper_joint': 0.109,
                                                         "gripper_base_gripper_left_joint": -0.055})
left_gripper.add_static_joint_states(GripperState.CLOSE, {'gripper_joint': 0.0065,
                                                          "gripper_base_gripper_left_joint": -0.0027})

left_arm.end_effector = left_gripper

################################## Torso ##################################
torso = KinematicChainDescription("torso", "base_footprint", "ur5_base_link",
                                  donbot_description.urdf_object, include_fixed_joints=True)

# fixed joint, so all states set to 0
torso.add_static_joint_states(TorsoState.HIGH, {"arm_base_mounting_joint": 0})

torso.add_static_joint_states(TorsoState.MID, {"arm_base_mounting_joint": 0})

torso.add_static_joint_states(TorsoState.LOW, {"arm_base_mounting_joint": 0})

donbot_description.add_kinematic_chain_description(torso)

################################## Camera ##################################
camera = CameraDescription("camera_link", "camera_link", 0.5,
                           1.2, 0.99483, 0.75049,
                           [0, 0, 1])
donbot_description.add_camera_description(camera)

################################## Neck ##################################
neck = KinematicChainDescription("neck", "ur5_base_link", "ur5_wrist_3_link", donbot_description.urdf_object)

# TODO: may be able to remove this already
neck.add_static_joint_states("down", {'ur5_shoulder_pan_joint': 4.130944728851318,
                                      'ur5_shoulder_lift_joint': 0.04936718940734863,
                                      'ur5_elbow_joint': -1.9734209219561976,
                                      'ur5_wrist_1_joint': -1.7624157110797327,
                                      'ur5_wrist_2_joint': 1.6369260549545288,
                                      'ur5_wrist_3_joint': -1.6503327528582972})
neck.add_static_joint_states("right", {'ur5_shoulder_pan_joint': 1.6281344890594482,
                                       'ur5_shoulder_lift_joint': -1.1734271208392542,
                                       'ur5_elbow_joint': -1.1555221716510218,
                                       'ur5_wrist_1_joint': -1.7555221716510218,
                                       'ur5_wrist_2_joint': 1.4996352195739746,
                                       'ur5_wrist_3_joint': -1.4276765028582972})
neck.add_static_joint_states("left", {'ur5_shoulder_pan_joint': -1.6281344890594482,
                                      'ur5_shoulder_lift_joint': -1.1734271208392542,
                                      'ur5_elbow_joint': -1.1555221716510218,
                                      'ur5_wrist_1_joint': -1.7555221716510218,
                                      'ur5_wrist_2_joint': 1.4996352195739746,
                                      'ur5_wrist_3_joint': -1.4276765028582972})
neck.add_static_joint_states("right_separators", {'ur5_shoulder_pan_joint': 1.4752850532531738,
                                                  'ur5_shoulder_lift_joint': -1.4380276838885706,
                                                  'ur5_elbow_joint': -1.9198325316058558,
                                                  'ur5_wrist_1_joint': -0.0680769125567835,
                                                  'ur5_wrist_2_joint': 1.704722285270691,
                                                  'ur5_wrist_3_joint': -1.5686963240252894})
neck.add_static_joint_states("right_separators_preplace_state", {'ur5_shoulder_pan_joint': 1.585883378982544,
                                                                 'ur5_shoulder_lift_joint': -0.49957687059511,
                                                                 'ur5_elbow_joint': -1.61414081255068,
                                                                 'ur5_wrist_1_joint': -1.1720898787127894,
                                                                 'ur5_wrist_2_joint': 1.37771737575531,
                                                                 'ur5_wrist_3_joint': -1.3602331320392054})

left_arm.add_static_joint_states("front", {'ur5_shoulder_pan_joint': -1.5,
                                           'ur5_shoulder_lift_joint': -1.5,
                                           'ur5_elbow_joint': 0,
                                           'ur5_wrist_1_joint': 1.5,
                                           'ur5_wrist_2_joint': -1.5,
                                           'ur5_wrist_3_joint': 1.5})
left_arm.add_static_joint_states("arm_right", {'ur5_shoulder_pan_joint': -3,
                                               'ur5_shoulder_lift_joint': -1.5,
                                               'ur5_elbow_joint': 0,
                                               'ur5_wrist_1_joint': 1.5,
                                               'ur5_wrist_2_joint': -1.5,
                                               'ur5_wrist_3_joint': 1.5})
left_arm.add_static_joint_states("back", {'ur5_shoulder_pan_joint': -4.7,
                                          'ur5_shoulder_lift_joint': -1.5,
                                          'ur5_elbow_joint': 0,
                                          'ur5_wrist_1_joint': 1.5,
                                          'ur5_wrist_2_joint': -1.5,
                                          'ur5_wrist_3_joint': 1.5})
left_arm.add_static_joint_states("arm_left", {'ur5_shoulder_pan_joint': 0,
                                              'ur5_shoulder_lift_joint': -1.5,
                                              'ur5_elbow_joint': 0,
                                              'ur5_wrist_1_joint': 1.5,
                                              'ur5_wrist_2_joint': -1.5,
                                              'ur5_wrist_3_joint': 1.5})

donbot_description.set_neck("ur5_wrist_2_joint", "ur5_wrist_1_joint")


################################# Grasps ##################################
left_gripper.generate_all_grasp_orientations_from_front_grasp([0.707, -0.707, 0.707, -0.707])

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(donbot_description)
