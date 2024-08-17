import rospy

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


def tmc_gripper_control(designator):
    if (designator.motion == "open"):
        pub_gripper = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                      queue_size=10)
        rate = rospy.Rate(10)
        rospy.sleep(2)
        msg = GripperApplyEffortActionGoal()  # sprechen joint gripper_controll_manager an, indem wir goal publishen type den giskard fürs greifen erwartet
        msg.goal.effort = 0.8
        pub_gripper.publish(msg)

    elif (designator.motion == "close"):
        pub_gripper = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                      queue_size=10)
        rate = rospy.Rate(10)
        rospy.sleep(2)
        msg = GripperApplyEffortActionGoal()
        msg.goal.effort = -0.8
        pub_gripper.publish(msg)


def tmc_talk(designator):
    pub = rospy.Publisher('/talk_request', Voice, queue_size=10)

    # fill message of type Voice with required data:
    texttospeech = Voice()
    # language 1 = english (0 = japanese)
    texttospeech.language = 1
    texttospeech.sentence = designator.cmd

    rospy.sleep(1)
    pub.publish(texttospeech)
