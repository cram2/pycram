import signal
import sys

import actionlib
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pr2_controllers_msgs.msg import JointControllerState, JointTrajectoryAction, JointTrajectoryControllerState, JointTrajectoryGoal, Pr2GripperCommandAction, Pr2GripperCommandGoal, PointHeadAction, PointHeadGoal, SingleJointPositionAction, SingleJointPositionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sound_play.msg import SoundRequestAction, SoundRequestGoal

signal.signal(signal.SIGINT, exit)

rospy.init_node('control')

move_client = actionlib.SimpleActionClient("/nav_pcontroller/move_base", MoveBaseAction)
move_client.wait_for_server()

head_client = actionlib.SimpleActionClient("/head_traj_controller/point_head_action", PointHeadAction)
head_client.wait_for_server()

l_arm_client = actionlib.SimpleActionClient("/l_arm_controller/joint_trajectory_action", JointTrajectoryAction)
l_arm_client.wait_for_server()

r_arm_client = actionlib.SimpleActionClient("/r_arm_controller/joint_trajectory_action", JointTrajectoryAction)
r_arm_client.wait_for_server()

l_gripper_client = actionlib.SimpleActionClient("/l_gripper_controller/gripper_action", Pr2GripperCommandAction)
l_gripper_client.wait_for_server()

r_gripper_client = actionlib.SimpleActionClient("/r_gripper_controller/gripper_action", Pr2GripperCommandAction)
r_gripper_client.wait_for_server()

torso_client = actionlib.SimpleActionClient("/torso_controller/position_joint_action", SingleJointPositionAction)
torso_client.wait_for_server()

sound_client = actionlib.SimpleActionClient("/sound_play", SoundRequestAction)
sound_client.wait_for_server()

def exit(sig, frame):
	move_client.cancel_all_goals()
	head_client.cancel_all_goals()
	l_arm_client.cancel_all_goals()
	r_arm_client.cancel_all_goals()
	l_gripper_client.cancel_all_goals()
	r_gripper_client.cancel_all_goals()
	torso_client.cancel_all_goals()
	sound_client.cancel_all_goals()
	sys.exit()

def subscribe_pose(fl):
	rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, lambda msg: fl.set_value(msg.pose))

def subscribe_gripper(gripper, fl):
	rospy.Subscriber('{}_gripper_controller/state'.format('l' if gripper == '0' else 'r'), JointControllerState, lambda msg: fl.set_value(msg.process_value))

def move_to(pose, wait=True, frame="base_footprint"):
	msg = MoveBaseGoal()
	msg.target_pose.header.frame_id = frame
	msg.target_pose.pose.position.x = pose.position.x
	msg.target_pose.pose.position.y = pose.position.y
	msg.target_pose.pose.position.z = pose.position.z
	msg.target_pose.pose.orientation.x = pose.orientation.x
	msg.target_pose.pose.orientation.y = pose.orientation.y
	msg.target_pose.pose.orientation.z = pose.orientation.z
	msg.target_pose.pose.orientation.w = pose.orientation.w
	move_client.send_goal(msg)
	if wait:
		move_client.wait_for_result()

def cancel_movement():
	move_client.cancel_all_goals()

def point_head_to(x, y, z, wait=True, frame="base_footprint"):
	msg = PointHeadGoal()
	msg.target.header.frame_id = frame
	msg.target.point.x = x
	msg.target.point.y = y
	msg.target.point.z = z
	msg.max_velocity = 1.0
	head_client.send_goal(msg)
	if wait:
		head_client.wait_for_result()

def set_arm_joints(arm, positions, wait=True):
	msg = JointTrajectoryGoal()
	msg.trajectory.joint_names = ['{}_shoulder_pan_joint'.format(arm), '{}_shoulder_lift_joint'.format(arm), '{}_upper_arm_roll_joint'.format(arm), '{}_elbow_flex_joint'.format(arm), '{}_forearm_roll_joint'.format(arm), '{}_wrist_flex_joint'.format(arm), '{}_wrist_roll_joint'.format(arm)]
	msg.trajectory.points = [JointTrajectoryPoint()]
	msg.trajectory.points[0].positions = positions
	msg.trajectory.points[0].velocities = [0, 0, 0, 0, 0, 0, 0]
	if arm.lower() == 'l':
		l_arm_client.send_goal(msg)
		if wait:
			l_arm_client.wait_for_result()
	elif arm.lower() == 'r':
		r_arm_client.send_goal(msg)
		if wait:
			r_arm_client.wait_for_result()

def set_gripper(gripper, pos, wait=True, max_effort=25):
	msg = Pr2GripperCommandGoal()
	msg.command.position = pos
	msg.command.max_effort = max_effort
	if gripper.lower() == 'l':
		l_gripper_client.send_goal(msg)
		if wait:
			l_gripper_client.wait_for_result()
	elif gripper.lower() == 'r':
		r_gripper_client.send_goal(msg)
		if wait:
			r_gripper_client.wait_for_result()

def set_torso(pos, wait=True):
	msg = SingleJointPositionGoal()
	msg.position = pos
	torso_client.send_goal(msg)
	if wait:
		torso_client.wait_for_result()

def say(text, wait=True):
	msg = SoundRequestGoal()
	msg.sound_request.sound = -3
	msg.sound_request.command = 1
	msg.sound_request.arg = text
	sound_client.send_goal(msg)
	if wait:
		sound_client.wait_for_result()

def play(path, wait=True):
	msg = SoundRequestGoal()
	msg.sound_request.sound = -2
	msg.sound_request.command = 1
	msg.sound_request.arg = path
	sound_client.send_goal(msg)
	if wait:
		sound_client.wait_for_result()
