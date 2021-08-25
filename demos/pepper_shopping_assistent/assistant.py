from pycram.knowrob import get_pose_for_product_type
from pycram.process_module import with_real_robot, real_robot, with_simulated_robot, simulated_robot
from pycram.motion_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.ik import request_ik
from navigation import navigation
from std_msgs.msg import String
from pycram.robot_description import InitializedRobotDescription as robot_description
import numpy as np
import pybullet as p
import pepper_process_modules
import action_desig_grounding
import motion_desig_grounding
import rospy
import tf
import time

#node = rospy.init_node("assistant")
#rospy.Subscriber('/assistant', String, app_callback)
#rospy.spin()
# http://knowrob.org/kb/shop.owl#FruitOrCereal

type_to_knowrob = {}

@with_real_robot
def assistant(product_type):
    MotionDesignator(MoveArmJointsMotionDescription(left_arm_config="park", right_arm_config="park")).perform()
    tf_listener = tf.TransformListener()
    time.sleep(2)
    item_pose = get_pose_for_product_type(product_type)
    #item_pose = [[0, -3 , 0], [0, 0, 0, 1]]
    robot_pose = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    route = navigation(item_pose[0], robot_pose[0])
    print(route)
    #route = navigation([-0.1, -3.2, 0], [0, 0, -0.70, 0.70])
    for pose in route:
        robot_pose = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        angle_to_goal = np.arctan2(pose[1] - robot_pose[0][1], pose[0] - robot_pose[0][0])
        angle_as_quanternion = list(tf.transformations.quaternion_from_euler(0, 0, angle_to_goal, axes="sxyz"))
        print(angle_as_quanternion)
        MotionDesignator(MoveMotionDescription(target=pose, orientation=angle_as_quanternion)).perform()
    point_to_2(item_pose)

@with_real_robot
def simple_assistant():
    tf_listener = tf.TransformListener()
    time.sleep(2)
    robot_pose = tf_listener.lookupTransform('/base_footprint', '/map', rospy.Time(0))
    print(robot_pose)
    route = navigation([1.5, -3.4, 0], robot_pose[0])
    for pose, orientation in route:
        print(pose)
        print(orientation)
        MotionDesignator(MoveMotionDescription(target=pose, orientation=orientation)).perform()



def app_callback(msg):
    knowrob_type = type_to_knowrob[msg.data]
    assistant(knowrob_type)

@with_real_robot
def point_to_2(goal_pose):
    tf_listener = tf.TransformListener()
    time.sleep(2)
    base_pose = tf_listener.lookupTransform('/map', '/base_link',  rospy.Time(0))

    #base_pose = pepper.get_position_and_orientation()
    #print(base_pose)
    angle_to_goal = np.arctan2(goal_pose[1] - base_pose[0][1], goal_pose[0] - base_pose[0][0])
    angle_as_quanternion = list(tf.transformations.quaternion_from_euler(0, 0, angle_to_goal, axes="sxyz"))
    MotionDesignator(MoveMotionDescription(target=base_pose[0], orientation=angle_as_quanternion)).perform()
    time.sleep(1)
    MotionDesignator(MoveArmJointsMotionDescription(left_arm_config='point')).perform()

    shoulder_link = "LShoulder"
    #shoulder_pose = pepper.get_link_position(shoulder_link)
    shoulder_pose = tf_listener.lookupTransform('/map', shoulder_link, rospy.Time(0))[0]
    shoulder_angle = np.arctan2(goal_pose[2] - shoulder_pose[2], goal_pose[0]-shoulder_pose[0])
    print(shoulder_angle)
    shoulder_joint = "LShoulderPitch"
    shoulder_angle = -shoulder_angle + 0.1
    MotionDesignator(MoveArmJointsMotionDescription(left_arm_poses={shoulder_joint:shoulder_angle})).perform()
    p.addUserDebugLine(shoulder_pose, goal_pose)

#point_to_2([3, 2, 0.8])
#assistant('http://knowrob.org/kb/shop.owl#FruitOrCereal')
with real_robot:
    MotionDesignator(MoveMotionDescription(target=[2, 0, 0], orientation=[0, 0, 0, 1])).perform()


# def point_to(arm, goal):
#     shoulder_link = "RShoulder" if arm == "right" else "LShoulder"
#     hand_link = "RHand" if arm == "right" else "LHand"
#     tf_listener = tf.TransformListener()
#     time.sleep(2)
#
#     shoulder_pose = tf_listener.lookupTransform('/map', shoulder_link, rospy.Time(0))
#     print(f"tf shoulder: {shoulder_pose}")
#     foot_print_pose = tf_listener.lookupTransform('/base_footprint', shoulder_link, rospy.Time(0))
#
#     ee_goal = _calculate_ee_pose(shoulder_pose[0], goal)
#     #ee_goal_in_footprint = list(np.array(foot_print_pose[0]) + ee_goal)
#
#     arm_joints = robot_description.i._safely_access_chains(arm).joints
#     ik = request_ik(shoulder_link, hand_link, [ee_goal_in_footprint, [0, 0, 0, 1]], arm_joints)
#
#     values = {}
#     for joint, value in zip(arm_joints, ik):
#         values[joint] = value
#
#     description = MoveArmJointsMotionDescription()
#     description.__dict__[arm+'_arm_poses'] = values
#
#     MotionDesignator(describtion).perform()
#
# def _calculate_orientation(y_axis):
#     up_vector = np.array([0, 0, 1])
#     y_axis = y_axis / np.linalg.norm(y_axis)
#
#     z_axis = np.cross(y_axis, up_vector)
#     x_axis = np.cross(y_axis, z_axis)
#
#     x_axis = x_axis / np.linalg.norm(x_axis)
#     z_axis = z_axis / np.linalg.norm(z_axis)
#
#     m = np.array([x_axis, y_axis, z_axis])
#
#     print(f"det: {np.linalg.det(m)}")
#     qw = np.sqrt(1 + m[0][0] + m[1][1] + m[2][2])
#     q = [(m[2][1] - m[1][2])/(4*qw), (m[0][2] - m[2][0])/(4*qw), (m[1][0] - m[0][1])/(4*qw), qw]
#     return q
#
#
# def _calculate_ee_vector(shoulder_pose, goal_pose):
#     shoulder_pose = np.array(shoulder_pose)
#     goal_pose = np.array(goal_pose)
#
#     # Vector from the shoulder to the goal
#     direction_vector = goal_pose - shoulder_pose
#     dir_length = np.linalg.norm(direction_vector)
#     # Short vector has rigt direction and length
#     short_vector = (0.4 / dir_length)*direction_vector
#
#     vector_in_map = shoulder_pose + short_vector
#     print(f"shoulder pose: {shoulder_pose}")
#     print(f"short vector: {short_vector}")
#     print(f"vector in map: {vector_in_map}")
#     # Return vector is in shoulder frame
#     return vector_in_map



#
# world = BulletWorld()
# pepper = Object("pepper", "robot", "resources/pepper.urdf", [0, 0, 0.83], [0, 0, 0, 1])
# BulletWorld.robot = pepper
# world.add_vis_axis([[1,1,0.5], [0, 0, 0, 1]])
# point_to_2([1 ,1, 0.5])
#
# with simulated_robot:
#      MotionDesignator(MoveArmJointsMotionDescription(left_arm_config="park", right_arm_config="park")).perform()
#      #["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
#      #[-0.46, -0.37, 0.46, 0.69, 1.82]
#      arm_poses = {"LShoulderPitch": 0.0, "LShoulderRoll": 0.37, "LElbowYaw": 0.1,"LElbowRoll":-0.6, "LWristYaw":-1.82 }
#      MotionDesignator(MoveArmJointsMotionDescription(left_arm_poses=arm_poses)).perform()



#with real_robot:
    #MotionDesignator(MoveArmJointsMotionDescription(left_arm_config="park", right_arm_config="park")).perform()
    #MotionDesignator(MoveMotionDescription(target=[2, 0, 0], orientation=[0, 0, 0, 1])).perform()
    #assistant()


#print(get_pose_for_product_type('http://knowrob.org/kb/shop.owl#FruitOrCereal'))
