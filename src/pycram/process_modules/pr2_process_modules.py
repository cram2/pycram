from .default_process_modules import *
from ..datastructures.enums import GripperState, Arms, ExecutionType
from ..datastructures.world import World
from ..designators.motion_designator import MoveGripperMotion
from ..process_module import ProcessModule, ProcessModuleManager
from ..ros import Duration, create_action_client
from ..ros import  loginfo, logwarn, logdebug

try:
    from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2
except ImportError:
    pass

try:
    from ..worlds.multiverse import Multiverse
except ImportError:
    Multiverse = type(None)

try:
    from control_msgs.msg import GripperCommandGoal, GripperCommandAction
except ImportError:
    if Multiverse is not None:
        logwarn("Import for control_msgs for gripper in Multiverse failed")

try:
    from ..worlds import Multiverse
except ImportError:
    Multiverse = type(None)

try:
    from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2
except ImportError:
    logdebug("Pr2GripperCommandGoal not found")

class Pr2MoveGripperMultiverse(ProcessModule):
    """
    Opens or closes the gripper of the real PR2, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion):
        def activate_callback():
            loginfo("Started gripper Movement")

        def done_callback(state, result):
            loginfo(f"Reached goal {designator.motion}: {result.reached_goal}")

        def feedback_callback(msg):
            loginfo(f"Gripper Action Feedback: {msg}")

        goal = GripperCommandGoal()
        goal.command.position = 0.0 if designator.motion == GripperState.CLOSE else 0.4
        goal.command.max_effort = 50.0
        if designator.gripper == "right":
            controller_topic = "/real/pr2/right_gripper_controller/gripper_cmd"
        else:
            controller_topic = "/real/pr2/left_gripper_controller/gripper_cmd"
        client = create_action_client(controller_topic, GripperCommandAction)
        loginfo("Waiting for action server")
        client.wait_for_server()
        client.send_goal(goal, active_cb=activate_callback, done_cb=done_callback, feedback_cb=feedback_callback)
        wait = client.wait_for_result(Duration(5))
        # client.cancel_all_goals()

class Pr2MoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real PR2, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion):
        def activate_callback():
            loginfo("Started gripper Movement")

        def done_callback(state, result):
            loginfo(f"Reached goal {designator.motion}")

        def feedback_callback(msg):
            pass

        goal = Pr2GripperCommandGoal()

        position_map = {
            GripperState.CLOSE: 0.0,
            GripperState.MEDIUM: 0.015,
            GripperState.OPEN: 0.1  # or whatever default fits
        }
        goal.command.position = position_map.get(designator.motion, 0.1)
        goal.command.max_effort = -1
        if designator.gripper == Arms.RIGHT:
            controller_topic = "r_gripper_controller/gripper_action"
        else:
            controller_topic = "l_gripper_controller/gripper_action"
        client = create_action_client(controller_topic, Pr2GripperCommandAction)
        loginfo("Waiting for action server")
        client.wait_for_server()
        client.send_goal(goal, active_cb=activate_callback, done_cb=done_callback, feedback_cb=feedback_callback)
        wait = client.wait_for_result()

class Pr2Manager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "pr2"

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            if (isinstance(World.current_world, Multiverse) and
                    World.current_world.conf.use_multiverse_process_modules):
                return Pr2MoveGripperMultiverse(self._move_gripper_lock)
            else:
                return Pr2MoveGripperReal(self._move_gripper_lock)

# Initialize the PR2 manager and register it with the process module manager
Pr2Manager()