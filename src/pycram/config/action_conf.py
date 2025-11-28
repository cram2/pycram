from datetime import timedelta


class ActionConfig:
    pick_up_prepose_distance = 0.03

    grasping_prepose_distance = 0.03

    navigate_keep_joint_states = True

    face_at_keep_joint_states = True

    execution_delay: timedelta = timedelta(seconds=0.0)
    """
    The delay between the execution of actions/motions to imitate real world execution time.
    """
