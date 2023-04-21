import pycram.task
from pycram.resolver.plans import Arms
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.designators.object_designator import *
import pycram.plan_failures
import numpy as np
from pycram.costmaps import SemanticCostmap
from pycram.pose_generator_and_validator import pose_generator
from pycram.external_interfaces.ik import IKError
from time import sleep


def get_n_random_positions(pose_list, n=4, dist=0.5, random=True):
    positions = [pos[0] for pos in pose_list[:1000]]
    all_indices = list(range(len(positions)))
    print(len(all_indices))
    pos_idx = np.random.choice(all_indices) if random else all_indices[0]
    all_indices.remove(pos_idx)
    n_positions = np.zeros((n, 3))
    for i in range(n):
        n_positions[i, :] = positions[pos_idx]
    found_count = 1
    found_indices = [pos_idx]
    for i in range(len(positions) - 1):
        pos_idx = np.random.choice(all_indices) if random else all_indices[i]
        diff = np.absolute(np.linalg.norm(n_positions - positions[pos_idx], axis=1))
        # print(diff)
        min_diff = np.min(diff)
        # print(min_diff)
        if min_diff >= dist:
            # print("found")
            n_positions[found_count, :] = positions[pos_idx]
            found_indices.append(pos_idx)
            found_count += 1
        all_indices.remove(pos_idx)
        if found_count == n:
            break
    found_poses = [pose_list[i] for i in found_indices]
    return found_poses


@pycram.task.with_tree
def plan(world, robot_desig, env_desig, obj_desig, torso=0.2, place="countertop"):
    world.reset_bullet_world()
    good_torso = []
    with simulated_robot:
        ParkArmsAction.Action(Arms.BOTH).perform()

        MoveTorsoAction.Action(torso).perform()
        location = CostmapLocation(target=obj_desig, reachable_for=robot_desig)
        pose = location.resolve()
        print()
        NavigateAction.Action(pose.pose).perform()
        ParkArmsAction.Action(Arms.BOTH).perform()
        good_torso.append(torso)
        picked_up_arm = pose.reachable_arms[0]
        PickUpAction.Action(object_designator=obj_desig, arm=pose.reachable_arms[0], grasp="front").perform()

        ParkArmsAction.Action(Arms.BOTH).perform()
        scm = SemanticCostmapLocation(place, env_desig, obj_desig)
        pose_island = scm.resolve()

        place_location = CostmapLocation(target=pose_island.pose, reachable_for=robot_desig,
                                         reachable_arm=picked_up_arm)
        pose = place_location.resolve()

        NavigateAction.Action(pose.pose).perform()

        PlaceAction.Action(object_designator=obj_desig, target_location=pose_island.pose, arm=picked_up_arm).perform()

        ParkArmsAction.Action(Arms.BOTH).perform()

        return good_torso


class CRAMPlan:
    def __init__(self):
        # Define world, robot, and environment
        self.world = BulletWorld()
        self.robot = Object(robot_description.i.name, "robot", robot_description.i.name + ".urdf")
        self.robot_desig = ObjectDesignatorDescription(names=['pr2']).resolve()
        self.apartment = Object("apartment", "environment",
                                "/home/abassi/cram_ws/src/iai_maps/iai_apartment/urdf/apartment.urdf",
                                position=[-1.5, -2.5, 0])
        self.apartment_desig = ObjectDesignatorDescription(names=['apartment']).resolve()
        self.table_top = self.apartment.get_link_position("cooktop")
        self.object_names = ["bowl", "milk", "breakfast_cereal", "spoon"]
        sleep(5)
        self.objects = {}
        self.object_desig = {}
        self.place_objects(inistantiate=True)
        self.good_torsos = []

    def place_objects(self, inistantiate=False):
        # Define and place the objects
        scm = SemanticCostmap(self.apartment, "island_countertop")
        poses_list = list(pose_generator(scm, number_of_samples=-1))
        poses_list.sort(reverse=True, key=lambda x: np.linalg.norm(x[0]))
        object_poses = get_n_random_positions(poses_list)
        for obj_name, obj_pose in zip(self.object_names, object_poses):
            if inistantiate:
                print(obj_name)
                print(obj_pose)
                self.objects[obj_name] = Object(obj_name, obj_name, obj_name + ".stl",
                                                position=[obj_pose[0][0], obj_pose[0][1], self.table_top[2]])
                self.object_desig[obj_name] = ObjectDesignatorDescription(names=[obj_name], types=[obj_name]).resolve()
            else:
                self.objects[obj_name].set_position_and_orientation(
                    position=[obj_pose[0][0], obj_pose[0][1], self.table_top[2]],
                    orientation=obj_pose[1])
            self.objects[obj_name].move_base_to_origin_pos()
            self.objects[obj_name].original_pose = self.objects[obj_name].get_position_and_orientation()

        print(object_poses)

    def execute_plan(self):
        # Execute plan
        if len(self.object_names) > 0:
            self.place_objects(inistantiate=False)
        else:
            self.place_objects(inistantiate=True)
        for obj_name in self.object_names:
            done = False
            torso = 0.25 if len(self.good_torsos) == 0 else self.good_torsos[-1]
            while not done:
                try:
                    gt = plan(self.world, self.robot_desig, self.apartment_desig, self.object_desig[obj_name], torso=torso,
                         place="island_countertop")
                    self.good_torsos.extend(gt)
                    done = True
                    self.objects[obj_name].original_pose = self.objects[obj_name].get_position_and_orientation()
                except (StopIteration, IKError) as e:
                    print(type(e))
                    print(e)
                    print("no solution")
                    torso += 0.05
                    if torso > 0.3:
                        break
        print(self.good_torsos)


if __name__ == '__main__':
    cram_plan = CRAMPlan()
    cram_plan.execute_plan()
