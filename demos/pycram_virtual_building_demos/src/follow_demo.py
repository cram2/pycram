import queue
from pycram.designators.action_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode, TorsoState
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.utils import axis_angle_to_quaternion
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
from pynput import keyboard
import time

# Fifo queue to store movements of human
query = queue.Queue()
# fifo queue to store movements of robots
robot_poses = queue.Queue()
# lifo queue to move same way back
way_back = []

# counter to delay movement of robot
counter = 0


def on_press(key):
    """
    function that gets activated when a key is pressed
    for moving the human in the simulation
    :param key: key that is pressed
    """

    global counter
    counter += 1

    try:
        query.put(key.char)
    except AttributeError:
        # in case arrow keys are used as controls
        if key == keyboard.Key.up:
            query.put('w')
        elif key == keyboard.Key.down:
            query.put('s')
        elif key == keyboard.Key.left:
            query.put('a')
        elif key == keyboard.Key.right:
            query.put('d')


def collision_check(target: Pose, possible_collision: Pose):
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
    if kitchen_island[0][0] <= goalx <= kitchen_island[0][1] and kitchen_island[1][0] <= goaly <= kitchen_island[1][1]:
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


def follow_simple_example(robot):
    """
    demo function for carry my luggage
    robot follows human. human is can be moved with w/a/s/d or arrow keys
    :param robot: used robot in the simulation
    """
    global query
    global robot_poses
    global counter
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    with simulated_robot:
        # robot standard position
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        # orientation of human when certain key is pressed
        ori_s = axis_angle_to_quaternion((0, 0, 1), 90)
        ori_a = axis_angle_to_quaternion((0, 0, 1), 0)
        ori_w = axis_angle_to_quaternion((0, 0, 1), 270)
        ori_d = axis_angle_to_quaternion((0, 0, 1), 180)

        # starting pose of human
        pose = Pose([2, 0, 0], ori_s)

        # spawn human
        human = Object("human", ObjectType.MILK, "human.stl", pose=pose,
                       color=Color(1, 0, 0, 1))

        pose = Pose([1, 0, 0], ori_a)
        # position robot near the human
        NavigateAction([pose]).resolve().perform()

        # counter for delayed movement of robot
        counter = 0
        try:
            while listener.is_alive():

                # if keys have been pressed
                if not query.empty():

                    # get pressed key from fifo queue
                    entry = query.get()

                    # save human pose for robot later
                    old_human_pose = human.get_pose()

                    # key determines next step of human
                    if entry == 'w':
                        x = old_human_pose.pose.position.x - 1
                        y = old_human_pose.pose.position.y
                        new_human_pose = Pose([x, y, 0], ori_w)

                        # if no collision is caused, move human
                        if not collision_check(new_human_pose, robot.get_pose()):
                            human.set_pose(new_human_pose)

                            # orientation of robot off by 90 degree
                            # put in twice for slower movement of robot
                            robot_poses.put(Pose([x+1, y, 0], ori_d))
                            robot_poses.put(Pose([x + 1, y, 0], ori_d))
                            way_back.append(Pose([x+1, y, 0], ori_a))

                    elif entry == 'a':
                        x = old_human_pose.pose.position.x
                        y = old_human_pose.pose.position.y - 1
                        new_human_pose = Pose([x, y, 0], ori_a)
                        if not collision_check(new_human_pose, robot):
                            human.set_pose(new_human_pose)

                            robot_poses.put(Pose([x, y+1, 0], ori_w))
                            robot_poses.put(Pose([x, y + 1, 0], ori_w))
                            way_back.append(Pose([x, y + 1, 0], ori_s))

                    elif entry == 's':
                        x = old_human_pose.pose.position.x + 1
                        y = old_human_pose.pose.position.y
                        new_human_pose = Pose([x, y, 0], ori_s)

                        if not collision_check(new_human_pose, robot):
                            human.set_pose(new_human_pose)

                            robot_poses.put(Pose([x-1, y, 0], ori_a))
                            robot_poses.put(Pose([x - 1, y, 0], ori_a))
                            way_back.append(Pose([x - 1, y, 0], ori_d))

                    elif entry == 'd':
                        x = old_human_pose.pose.position.x
                        y = old_human_pose.pose.position.y + 1
                        new_human_pose = Pose([x, y, 0], ori_d)

                        if not collision_check(new_human_pose, robot):
                            human.set_pose(new_human_pose)

                            robot_poses.put(Pose([x, y-1, 0], ori_s))
                            robot_poses.put(Pose([x, y - 1, 0], ori_s))
                            way_back.append(Pose([x, y - 1, 0], ori_w))

                    # check if robot should return to starting position
                    elif entry == "r":
                        human_pose = human.get_pose()
                        while way_back:
                            pose = way_back.pop()
                            NavigateAction([pose]).resolve().perform()
                        break

                # if queue of robot is filled and human has moved at least two steps
                if not robot_poses.empty() and counter > 1:
                    pose = robot_poses.get()
                    if pose:
                        # move robot if it is not crashing into something
                        if not collision_check(pose, human.get_pose()):
                            NavigateAction([pose]).resolve().perform()

            time.sleep(0.5)

        except KeyboardInterrupt:
            print("Exit")
