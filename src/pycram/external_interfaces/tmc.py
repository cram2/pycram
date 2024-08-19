import rospy
from typing_extensions import Optional

from ..datastructures.enums import GripperState
from ..designators.motion_designator import MoveGripperMotion, TalkingMotion

is_init = False


def init_tmc_interface():
    global is_init
    if is_init:
        return
    try:
        from tmc_control_msgs.msg import GripperApplyEffortActionGoal
        from tmc_msgs.msg import Voice
        is_init = True
        rospy.loginfo("Successfully initialized tmc interface")
    except ModuleNotFoundError as e:
        rospy.logwarn(f"Could not import TMC messages, tmc interface could not be initialized")


def tmc_gripper_control(designator: MoveGripperMotion, topic_name: Optional[str] = '/hsrb/gripper_controller/grasp/goal'):
    """
    Publishes a message to the gripper controller to open or close the gripper for the HSR.

    :param designator: The designator containing the motion to be executed
    :param topic_name: The topic name to publish the message to
    """
    if (designator.motion == GripperState.OPEN):
        pub_gripper = rospy.Publisher(topic_name, GripperApplyEffortActionGoal,
                                      queue_size=10)
        rate = rospy.Rate(10)
        msg = GripperApplyEffortActionGoal()
        msg.goal.effort = 0.8
        pub_gripper.publish(msg)

    elif (designator.motion == GripperState.CLOSE):
        pub_gripper = rospy.Publisher(topic_name, GripperApplyEffortActionGoal,
                                      queue_size=10)
        rate = rospy.Rate(10)
        msg = GripperApplyEffortActionGoal()
        msg.goal.effort = -0.8
        pub_gripper.publish(msg)


def tmc_talk(designator: TalkingMotion, topic_name: Optional[str] = '/talk_request'):
    """
    Publishes a sentence to the talk_request topic of the HSRB robot

    :param designator: The designator containing the sentence to be spoken
    :param topic_name: The topic name to publish the sentence to
    """
    pub = rospy.Publisher(topic_name, Voice, queue_size=10)
    texttospeech = Voice()
    # language 1 = english (0 = japanese)
    texttospeech.language = 1
    texttospeech.sentence = designator.cmd

    pub.publish(texttospeech)
