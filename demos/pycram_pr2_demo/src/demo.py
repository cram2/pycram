import math
import motion_designators
import process_modules
import rospy

from geometry_msgs.msg import Pose
from pycram.designator import MotionDesignator
from pycram.fluent import macros, Fluent, whenever
from pycram.language import macros, failure_handling, par, seq, State, try_in_order
from pycram.process_module import ProcessModule

pose = Fluent()
r_gripper_state = Fluent()

def reset_robot():
	with par as state:
		ProcessModule.perform(MotionDesignator([('type', 'setting_gripper'), ('gripper', '0'), ('position', 1)]))
		ProcessModule.perform(MotionDesignator([('type', 'setting_gripper'), ('gripper', '1'), ('position', 1)]))
		ProcessModule.perform(MotionDesignator([('type', 'setting_arm_joints'), ('arm', '0'), ('positions', [0.863628806499912, -0.35204634320186196, 0.062598299461055, -1.5624834948006732, 12.337630347710876, -2.015093455343633, -109.33697662946868])]))
		ProcessModule.perform(MotionDesignator([('type', 'setting_arm_joints'), ('arm', '1'), ('positions', [-0.5229603971359181, -0.3527442747622196, 0.11999344300251935, -1.6681661141106483, 94.62327918044838, -2.0109160876170535, 0.11287135350578059])]))
		ProcessModule.perform(MotionDesignator([('type', 'setting_torso'), ('position', 0.3)]))

	state.wait_for()
	goal = Pose()
	goal.orientation.w = 1.0
	ProcessModule.perform(MotionDesignator([('type', 'moving'), ('pose', goal), ('frame', 'map')]))

print('Initializing demo...')
ProcessModule.perform(MotionDesignator([('type', 'subscribing_pose'), ('fl', pose)]))
ProcessModule.perform(MotionDesignator([('type', 'subscribing_gripper'), ('gripper', '1'), ('fl', r_gripper_state)]))

reset_robot()

goal = Pose()
look_goal = Pose()

def grab_item():
	with par as state:
		ProcessModule.perform(MotionDesignator([('type', 'moving'), ('pose', goal), ('frame', 'map')]))
		while True:
			p = pose.get_value().pose.position
			if math.hypot(goal.position.x-p.x, goal.position.y-p.y) < 0.1:
				joints1 = [-0.0859596170472463, -0.11647065453147773, -2.041268906810807, -0.6065624875352394, 93.16130975194685, -1.3021992881147604, 0.3898056299396988]
				joints2 = [-0.059014966406225944, 0.2889082169850691, -1.447956049242228, -0.46179177615171163, 92.38934037659165, -1.3021992881147604, 0.3898056299396988]
				joints3 = [-0.3636967851931485, -0.04329604604612444, 0.1373117642504671, -0.3598731953377081, 98.97627247347805, -1.619118262120206, -0.18285877641116244]

				ProcessModule.perform(MotionDesignator([('type', 'setting_arm_joints'), ('arm', '1'), ('positions', joints1)]))
				rospy.sleep(3)
				ProcessModule.perform(MotionDesignator([('type', 'setting_arm_joints'), ('arm', '1'), ('positions', joints2)]))
				rospy.sleep(3)
				ProcessModule.perform(MotionDesignator([('type', 'setting_gripper'), ('gripper', '1'), ('position', 0)]))
				rospy.sleep(3)
				ProcessModule.perform(MotionDesignator([('type', 'setting_arm_joints'), ('arm', '1'), ('positions', joints3)]))
				break

			rospy.sleep(0.1)

	if r_gripper_state.get_value() < 0.01:
		ProcessModule.perform(MotionDesignator([('type', 'setting_gripper'), ('gripper', '1'), ('position', 1)]))
		raise

def place_item():
	joints1 = [-0.34272141100183123, 0.05644136829402769, -1.524766011073404, -0.7687056842847906, 67.41096823133962, -1.6232834354990002, -31.599376507705365]
	joints2 = [-0.3636967851931485, -0.04329604604612444, 0.1373117642504671, -0.3598731953377081, 98.97627247347805, -1.619118262120206, -0.18285877641116244]

	ProcessModule.perform(MotionDesignator([('type', 'setting_arm_joints'), ('arm', '1'), ('positions', joints1)]))
	rospy.sleep(3)
	ProcessModule.perform(MotionDesignator([('type', 'setting_gripper'), ('gripper', '1'), ('position', 1)]))
	rospy.sleep(3)
	ProcessModule.perform(MotionDesignator([('type', 'setting_arm_joints'), ('arm', '1'), ('positions', joints2)]))

with par as s:
	with whenever(pose.pulsed()):
		ProcessModule.perform(MotionDesignator([('type', 'pointing_head'), ('coordinates', [look_goal.position.x, look_goal.position.y, look_goal.position.z]), ('wait', False), ('frame', 'map')]))
		rospy.sleep(0.2)
	with seq as s:
		# try to grab item -> move and try again if fail
		with try_in_order as state:
			if True:
				goal.position.x = 0.1
				goal.position.y = -0.6
				goal.orientation.z = -0.700573543807
				goal.orientation.w = 0.713580205526

				look_goal.position.x = 0.1
				look_goal.position.y = -1.1
				look_goal.position.z = 0.8

				grab_item()
			if True:
				goal.position.x = -0.2
				goal.position.y = -0.6
				goal.orientation.z = -0.700573543807
				goal.orientation.w = 0.713580205526

				look_goal.position.x = -0.2
				look_goal.position.y = -1.1
				look_goal.position.z = 0.8

				grab_item()
			if True:
				goal.position.x = 0.4
				goal.position.y = -0.6
				goal.orientation.z = -0.700573543807
				goal.orientation.w = 0.713580205526

				look_goal.position.x = 0.4
				look_goal.position.y = -1.1
				look_goal.position.z = 0.8

				grab_item()

		if state.get_value() == State.FAILED:
			print('Could not grab any item.')
			raise

		# move to cooker #1
		# simultaneously check if pr2 lost the item
		goal.position.x = -0.1
		goal.position.y = 1.9
		goal.orientation.z = 0.9999981068
		goal.orientation.w = 0.00194586635055

		look_goal.position.x = -0.6
		look_goal.position.y = 1.9
		look_goal.position.z = 1.0

		with failure_handling:
			try:
				if r_gripper_state.get_value() > 0.01:
					ProcessModule.perform(MotionDesignator([('type', 'moving'), ('pose', goal), ('wait', False), ('frame', 'map')]))

				while True:
					p = pose.get_value().pose.position

					if r_gripper_state.get_value() < 0.01: # if lost item
						ProcessModule.perform(MotionDesignator([('type', 'cancelling_movement')]))
						ProcessModule.perform(MotionDesignator([('type', 'setting_gripper'), ('gripper', '1'), ('position', 1)]))
						input('Put the item into the right gripper and hit [Enter] to continue.')
						ProcessModule.perform(MotionDesignator([('type', 'setting_gripper'), ('gripper', '1'), ('position', 0)]))
						raise
					elif math.hypot(goal.position.x-p.x, goal.position.y-p.y) < 0.1:
						break

					rospy.sleep(0.1)
			except:
				retry()

		place_item()
