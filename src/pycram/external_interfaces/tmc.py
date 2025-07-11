# from typing_extensions import Optional
#
# from ..datastructures.enums import GripperState
# from ..robot_plans.motions import MoveGripperMotion
# from ..ros import  loginfo
# from ..ros import  create_publisher
# from ..ros import  Rate
#
# is_init = False
#
#
# def init_tmc_interface():
#     global is_init
#     if is_init:
#         return
#     from tmc_control_msgs.msg import GripperApplyEffortActionGoal
#     from tmc_msgs.msg import Voice
#     is_init = True
#     loginfo("Successfully initialized tmc interface")
#
#
#
# def tmc_gripper_control(designator: MoveGripperMotion, topic_name: Optional[str] = '/hsrb/gripper_controller/grasp/goal'):
#     """
#     Publishes a message to the gripper controller to open or close the gripper for the HSR.
#
#     :param designator: The designator containing the motion to be executed
#     :param topic_name: The topic name to publish the message to
#     """
#     if (designator.motion == GripperState.OPEN):
#         pub_gripper = create_publisher(topic_name, GripperApplyEffortActionGoal, 10)
#         rate = Rate(10)
#         msg = GripperApplyEffortActionGoal()
#         msg.goal.effort = 0.8
#         pub_gripper.publish(msg)
#
#     elif (designator.motion == GripperState.CLOSE):
#         pub_gripper = create_publisher(topic_name, GripperApplyEffortActionGoal, 10)
#         rate = Rate(10)
#         msg = GripperApplyEffortActionGoal()
#         msg.goal.effort = -0.8
#         pub_gripper.publish(msg)
#
#
# def tmc_talk(designator: TalkingMotion, topic_name: Optional[str] = '/talk_request'):
#     """
#     Publishes a sentence to the talk_request topic of the HSRB robot
#
#     :param designator: The designator containing the sentence to be spoken
#     :param topic_name: The topic name to publish the sentence to
#     """
#     pub = create_publisher(topic_name, Voice, 10)
#     texttospeech = Voice()
#     # language 1 = english (0 = japanese)
#     texttospeech.language = 1
#     texttospeech.sentence = designator.cmd
#
#     pub.publish(texttospeech)
