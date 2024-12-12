from copy import deepcopy

import rospy
from ipywidgets import Output

from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import NavigateAction
from pycram.process_module import simulated_robot
from pycram.utils import axis_angle_to_quaternion
from pycram.world_concepts.world_object import Object


class VariableHandler:
    """
    Singleton class to manage robot and human objects for the binder version of demo "follow".
    Stores all relevant variables in this class to manage later on.
    """
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        """
        Initialize the RobotDescriptionManager, if no instance exists a new instance is created.
        """
        if self._initialized: return
        # Fifo queue to store movements of human
        self.robot = None
        # fifo queue to store movements of robots
        self.human = None
        # lifo queue to move same way back
        self.way_back = []

        self.output = None
        self.stop_demo = False

        self.ori_s = axis_angle_to_quaternion((0, 0, 1), 90)
        self.ori_a = axis_angle_to_quaternion((0, 0, 1), 0)
        self.ori_d = axis_angle_to_quaternion((0, 0, 1), 180)
        self.ori_w = axis_angle_to_quaternion((0, 0, 1), 270)

        self.original_positions = {}

        self._initialized = True

    def set_robot(self, robot: Object):
        self.robot = robot
        self.original_positions["robot"] = self.robot.get_pose()

    def set_human(self, human: Object):
        self.human = human
        self.original_positions["human"] = self.human.get_pose()

    def set_output(self, output: Output):
        self.output = output

    def collision_check(self, target: Pose, possible_collision: Pose):
        """
        function to determine if a movement would lead to a collision
        returns True if human or robot would collide with its surroundings
        :param target: goal pose of operator that wants to move
        :param possible_collision: position of other moving operators (usually human or robot pose)
        """
        goalx = target.pose.position.x
        goaly = target.pose.position.y

        obstaclex = possible_collision.pose.position.x
        obstacley = possible_collision.pose.position.y

        kitchen_island = [[2, 3], [1, 4]]
        kitchen = [[0], [1, 3]]

        # check if goal pose is in space where the kitchen island is
        if kitchen_island[0][0] <= goalx <= kitchen_island[0][1] and kitchen_island[1][0] <= goaly <= kitchen_island[1][
            1]:
            return True

        # check if goal pose is in the space of the kitchen
        if goalx == 0 and kitchen[1][0] <= goaly <= kitchen[1][1]:
            return True

        # check if goal pose is in the table
        if goalx == 5 and goaly == 4:
            return True

        # check if another operator stands on goal pose
        if goalx == obstaclex and goaly == obstacley:
            return True
        else:
            return False

    def calc_modified_pose(self, old_pose: Pose, pose_modifier: Pose):
        new_pose = Pose(position=[old_pose.pose.position.x + pose_modifier.pose.position.x,
                                  old_pose.pose.position.y + pose_modifier.pose.position.y,
                                  old_pose.pose.position.z + pose_modifier.pose.position.z],
                        orientation=pose_modifier.pose.orientation)

        return new_pose

    def move_human_and_robot(self, new_human_pose: Pose, new_robot_pose: Pose, robot_pose_back: Pose):
        self.human.set_pose(new_human_pose)

        with simulated_robot:
            NavigateAction([new_robot_pose]).resolve().perform()

        self.way_back.append(robot_pose_back)

    def move_forward(self):
        if self.stop_demo:
            return

        old_human_pose = deepcopy(self.human.get_pose())
        old_robot_pose = deepcopy(self.robot.get_pose())

        human_pose_modifier = Pose([1, 0, 0], self.ori_s)
        new_robot_pose = Pose(old_human_pose.position, self.ori_a)
        robot_pose_back = Pose(old_robot_pose.position, self.ori_d)

        new_human_pose = self.calc_modified_pose(old_human_pose, human_pose_modifier)

        # if no collision is caused, move human
        if not self.collision_check(new_human_pose, old_robot_pose):
            self.move_human_and_robot(new_human_pose, new_robot_pose, robot_pose_back)

    def move_right(self):
        if self.stop_demo:
            return

        old_human_pose = deepcopy(self.human.get_pose())
        old_robot_pose = deepcopy(self.robot.get_pose())

        human_pose_modifier = Pose([0, 1, 0], self.ori_d)
        new_robot_pose = Pose(old_human_pose.position, self.ori_s)
        robot_pose_back = Pose(old_robot_pose.position, self.ori_w)

        new_human_pose = self.calc_modified_pose(old_human_pose, human_pose_modifier)

        # if no collision is caused, move human
        if not self.collision_check(new_human_pose, old_robot_pose):
            self.move_human_and_robot(new_human_pose, new_robot_pose, robot_pose_back)

    def move_left(self):
        if self.stop_demo:
            return

        old_human_pose = deepcopy(self.human.get_pose())
        old_robot_pose = deepcopy(self.robot.get_pose())

        human_pose_modifier = Pose([0, -1, 0], self.ori_a)
        new_robot_pose = Pose(old_human_pose.position, self.ori_w)
        robot_pose_back = Pose(old_robot_pose.position, self.ori_s)

        new_human_pose = self.calc_modified_pose(old_human_pose, human_pose_modifier)

        # if no collision is caused, move human
        if not self.collision_check(new_human_pose, old_robot_pose):
            self.move_human_and_robot(new_human_pose, new_robot_pose, robot_pose_back)

    def move_back(self):
        if self.stop_demo:
            return

        old_human_pose = deepcopy(self.human.get_pose())
        old_robot_pose = deepcopy(self.robot.get_pose())

        human_pose_modifier = Pose([-1, 0, 0], self.ori_w)
        new_robot_pose = Pose(old_human_pose.position, self.ori_d)
        robot_pose_back = Pose(old_robot_pose.position, self.ori_a)

        new_human_pose = self.calc_modified_pose(old_human_pose, human_pose_modifier)

        # if no collision is caused, move human
        if not self.collision_check(new_human_pose, old_robot_pose):
            self.move_human_and_robot(new_human_pose, new_robot_pose, robot_pose_back)

    def go_back(self):
        if self.stop_demo:
            return

        with simulated_robot:
            for pose in reversed(self.way_back):
                NavigateAction([pose]).resolve().perform()
                rospy.sleep(0.3)
        self.stop_demo = True

    def reset_demo(self):
        self.way_back = []
        self.human.set_pose(self.original_positions["human"])

        with simulated_robot:
            NavigateAction([self.original_positions["robot"]]).resolve().perform()

            pose = Pose([1, 0, 0], self.ori_a)
            NavigateAction([pose]).resolve().perform()

        self.stop_demo = False
