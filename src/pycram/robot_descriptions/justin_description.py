from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager, CameraDescription
from ..datastructures.enums import Arms, Grasp, GripperState, GripperType, TorsoState, StaticJointState
from ..ros import get_ros_package_path

filename = get_ros_package_path('pycram') + '/resources/robots/' + "rollin_justin" + '.urdf'

justin_description = RobotDescription("rollin_justin", "base_link", "torso2", "torso2_joint",
                                      filename)

################################## Left Arm ##################################
left_arm = KinematicChainDescription("left", "base_link", "left_arm7",
                                     justin_description.urdf_object, arm_type=Arms.LEFT)

left_arm.add_static_joint_states(StaticJointState.Park, {"torso1_joint": 0,
                                          "torso2_joint": 0,
                                          "torso3_joint": 0.174533,
                                          "torso4_joint": 0,
                                          "left_arm1_joint": 0,
                                          "left_arm2_joint": -1.9,
                                          "left_arm3_joint": 0,
                                          "left_arm4_joint": 1,
                                          "left_arm5_joint": 0,
                                          "left_arm6_joint": -1,
                                          "left_arm7_joint": 0})

justin_description.add_kinematic_chain_description(left_arm)

################################## Left Gripper ##################################
left_gripper = EndEffectorDescription("left_gripper", "left_arm7", "l_gripper_tool_frame",
                                      justin_description.urdf_object)
left_gripper.add_static_joint_states(GripperState.OPEN, {"left_1thumb_base_joint": 0.0,
                                                         "left_1thumb1_joint": 0.0,
                                                         "left_1thumb2_joint": 0.0,
                                                         "left_1thumb3_joint": 0.0,
                                                         "left_1thumb4_joint": 0.0,
                                                         "left_2tip_base_joint": 0.0,
                                                         "left_2tip1_joint": 0.0,
                                                         "left_2tip2_joint": 0.0,
                                                         "left_2tip3_joint": 0.0,
                                                         "left_2tip4_joint": 0.0,
                                                         "left_3middle_base_joint": 0.0,
                                                         "left_3middle1_joint": 0.0,
                                                         "left_3middle2_joint": 0.0,
                                                         "left_3middle3_joint": 0.0,
                                                         "left_3middle4_joint": 0.0,
                                                         "left_4ring_base_joint": 0.0,
                                                         "left_4ring1_joint": 0.0,
                                                         "left_4ring2_joint": 0.0,
                                                         "left_4ring3_joint": 0.0,
                                                         "left_4ring4_joint": 0.0})
left_gripper.add_static_joint_states(GripperState.CLOSE, {"left_1thumb_base_joint": 0.0,
                                                          "left_1thumb1_joint": 0.523599,
                                                          "left_1thumb2_joint": 1.50098,
                                                          "left_1thumb3_joint": 1.76278,
                                                          "left_1thumb4_joint": 1.76278,
                                                          "left_2tip_base_joint": 0.0,
                                                          "left_2tip1_joint": 0.523599,
                                                          "left_2tip2_joint": 1.50098,
                                                          "left_2tip3_joint": 1.76278,
                                                          "left_2tip4_joint": 1.76278,
                                                          "left_3middle_base_joint": 0.0,
                                                          "left_3middle1_joint": 0.523599,
                                                          "left_3middle2_joint": 1.50098,
                                                          "left_3middle3_joint": 1.76278,
                                                          "left_3middle4_joint": 1.76278,
                                                          "left_4ring_base_joint": 0.0,
                                                          "left_4ring1_joint": 0.523599,
                                                          "left_4ring2_joint": 1.50098,
                                                          "left_4ring3_joint": 1.76278,
                                                          "left_4ring4_joint": 1.76278})

left_gripper.end_effector_type = GripperType.FINGER
# left_gripper.opening_distance = 0.548
left_arm.end_effector = left_gripper

################################## Right Arm ##################################
right_arm = KinematicChainDescription("right", "base_link", "right_arm7",
                                      justin_description.urdf_object, arm_type=Arms.RIGHT)

right_arm.add_static_joint_states(StaticJointState.Park, {"torso1_joint": 0,
                                           "torso2_joint": 0,
                                           "torso3_joint": 0.174533,
                                           "torso4_joint": 0,
                                           "right_arm1_joint": 0,
                                           "right_arm2_joint": -1.9,
                                           "right_arm3_joint": 0,
                                           "right_arm4_joint": 1,
                                           "right_arm5_joint": 0,
                                           "right_arm6_joint": -1,
                                           "right_arm7_joint": 0})

justin_description.add_kinematic_chain_description(right_arm)

################################## Right Gripper ##################################
right_gripper = EndEffectorDescription("right_gripper", "right_arm7", "r_gripper_tool_frame",
                                       justin_description.urdf_object)
right_gripper.add_static_joint_states(GripperState.OPEN, {"right_1thumb_base_joint": 0.0,
                                                          "right_1thumb1_joint": 0.0,
                                                          "right_1thumb2_joint": 0.0,
                                                          "right_1thumb3_joint": 0.0,
                                                          "right_1thumb4_joint": 0.0,
                                                          "right_2tip_base_joint": 0.0,
                                                          "right_2tip1_joint": 0.0,
                                                          "right_2tip2_joint": 0.0,
                                                          "right_2tip3_joint": 0.0,
                                                          "right_2tip4_joint": 0.0,
                                                          "right_3middle_base_joint": 0.0,
                                                          "right_3middle1_joint": 0.0,
                                                          "right_3middle2_joint": 0.0,
                                                          "right_3middle3_joint": 0.0,
                                                          "right_3middle4_joint": 0.0,
                                                          "right_4ring_base_joint": 0.0,
                                                          "right_4ring1_joint": 0.0,
                                                          "right_4ring2_joint": 0.0,
                                                          "right_4ring3_joint": 0.0,
                                                          "right_4ring4_joint": 0.0})
right_gripper.add_static_joint_states(GripperState.CLOSE, {"right_1thumb_base_joint": 0.0,
                                                           "right_1thumb1_joint": 0.523599,
                                                           "right_1thumb2_joint": 1.50098,
                                                           "right_1thumb3_joint": 1.76278,
                                                           "right_1thumb4_joint": 1.76278,
                                                           "right_2tip_base_joint": 0.0,
                                                           "right_2tip1_joint": 0.523599,
                                                           "right_2tip2_joint": 1.50098,
                                                           "right_2tip3_joint": 1.76278,
                                                           "right_2tip4_joint": 1.76278,
                                                           "right_3middle_base_joint": 0.0,
                                                           "right_3middle1_joint": 0.523599,
                                                           "right_3middle2_joint": 1.50098,
                                                           "right_3middle3_joint": 1.76278,
                                                           "right_3middle4_joint": 1.76278,
                                                           "right_4ring_base_joint": 0.0,
                                                           "right_4ring1_joint": 0.523599,
                                                           "right_4ring2_joint": 1.50098,
                                                           "right_4ring3_joint": 1.76278,
                                                           "right_4ring4_joint": 1.76278})

right_gripper.end_effector_type = GripperType.FINGER
# right_gripper.opening_distance = 0.548
right_arm.end_effector = right_gripper

################################## Torso ##################################
torso = KinematicChainDescription("torso", "torso1", "torso4",
                                  justin_description.urdf_object)

torso.add_static_joint_states(TorsoState.HIGH, {"torso2_joint": 0,
                                                "torso3_joint": 0.174533,
                                                "torso4_joint": 0})

torso.add_static_joint_states(TorsoState.MID, {"torso2_joint": -0.8,
                                               "torso3_joint": 1.57,
                                               "torso4_joint": -0.77})

torso.add_static_joint_states(TorsoState.LOW, {"torso2_joint": -0.9,
                                               "torso3_joint": 2.33874,
                                               "torso4_joint": -1.57})

justin_description.add_kinematic_chain_description(torso)

################################## Camera ##################################
# real camera unknown at the moment of writing (also missing in urdf), so using dummy camera for now
camera = CameraDescription("dummy_camera", "head2", 1.27,
                           1.85, 0.99483, 0.75049,
                           [1, 0, 0])
justin_description.add_camera_description(camera)

################################## Neck ##################################
justin_description.add_kinematic_chain("neck", "torso4", "head2")
justin_description.set_neck("head1_joint", "head2_joint")

################################# Grasps ##################################
orientation = [0.707, -0.707, 0.707, -0.707]
left_gripper.update_all_grasp_orientations(orientation)

orientation = [0.707, 0.707, 0.707, 0.707]
right_gripper.update_all_grasp_orientations(orientation)


################################# Additionals ##################################
# justin_description.set_costmap_offset(0.5)
# justin_description.set_palm_axis([0, 0, 1])

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(justin_description)
