from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
from pycram.worlds.multiverse import Multiverse


def param_plan(robot_path,environment_path):
    world = BulletWorld(WorldMode.GUI)
    # world= Multiverse(simulation='pycram_test')
    apartment = Object(environment_path[:environment_path.find(".")], ObjectType.ENVIRONMENT, environment_path)
    robot = Object("pr2", ObjectType.ROBOT, robot_path, pose=Pose([1, 2, 0]))

    
    milk_pos=Pose([1, -1.78, 0.55],[1,0,0,0])
    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1, -1.78, 0.55],[1,0,0,0]),
                  color=Color(1, 0, 0, 1))

    pick_pose = Pose([1, -1.78, 0.55])

    robot_desig = BelieveObject(names=["pr2"])
    apartment_desig = BelieveObject(names=["apartment"])


    @with_simulated_robot
    def move_and_detect(obj_type):
        NavigateAction(target_locations=[Pose([2, -1.89, 0])]).resolve().perform()

        LookAtAction(targets=[pick_pose]).resolve().perform()

        object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

        return object_desig


    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.25]).resolve().perform()

        milk_desig = move_and_detect(ObjectType.MILK)

        TransportAction(milk_desig, [Arms.LEFT], [Pose([4.8, 3.55, 0.8])]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

# param_plan("pr2.urdf","tmp/house_666/house_666.urdf")


def param_plan2(robot_path, environment_path):
    cheatcode={"/home/nleusmann/catkin_ws/src/pycram/resources/apartment.urdf":[Pose([2.62,2.22,1.1]),Pose([4.8, 3.55, 0.8])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_1/house_1.urdf": [Pose([9.16,-1.07,1.20]), Pose([11.82,-6.2,0.8])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_2/house_2.urdf":[Pose([1, -1.78, 0.55]),Pose([2.71,-0.22,0.75])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_3/house_3.urdf":[Pose([6.02, -5.41, 1]),Pose([10.49,-5.83,0.80])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_4/house_4.urdf":[Pose([5.33, -8.81, 1]),Pose([3.78,-8.71,0.75])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_5/house_5.urdf": [Pose([0.5, -6.61, 1]), Pose([4.46, -4.42, 0.85])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_6/house_6.urdf": [Pose([9.73, -9, 0.80]), Pose([7.17, -5.57, 0.85])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_7/house_7.urdf": [Pose([8.77, -2.88, 1]), Pose([9.34, -4.44, 0.80])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_8/house_8.urdf": [Pose([12.28, -7.76, 1]), Pose([10.02, -12.34, 1])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_9/house_9.urdf": [Pose([6.54, -8.51, 1]), Pose([3.5, -7.8, 0.85])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_10/house_10.urdf": [Pose([8.74, -1.3, 1]), Pose([5.74, -2.91, 1])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_11/house_11.urdf": [Pose([7.03, -2.19, 1]), Pose([3.66, -5.84, 0.75],[1,0,0,0]),Pose([6.7, -3.5, 0])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_42/house_42.urdf":[Pose([7.43,-9.37,1.05]),Pose([11.12,-8.35,0.9])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_666/house_666.urdf":[Pose([7.06,-6.82,1]),Pose([1.25,-3.92,0.75],[1,0,0,0]),Pose([7.17,-7.77,0])],
               "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_1337/house_1337.urdf":[Pose([2.45,-0.36,0.9]),Pose([0.61,-5.82,0.8],[1,0,0,0]),Pose([2.34,-1.47,0])]}
    world = BulletWorld(WorldMode.GUI)
    # world= Multiverse(simulation='pycram_test')
    print(environment_path+35*'#')
    apartment = Object(environment_path[:environment_path.find(".")], ObjectType.ENVIRONMENT, environment_path)
    robot = Object("pr2", ObjectType.ROBOT, robot_path, pose=Pose([1, 2, 0]))
    milk_pos = cheatcode[environment_path][0]
    put_down_pos=cheatcode[environment_path][1]
    move_pos=milk_pos.copy()
    move_pos.pose.position.x=move_pos.pose.position.x-0.7
    move_pos.pose.position.z=0
    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=milk_pos,color=Color(1, 0, 0, 1))

    pick_pose = Pose([1, -1.78, 0.55])

    robot_desig = BelieveObject(names=["pr2"])
    apartment_desig = BelieveObject(names=["apartment"])

    @with_simulated_robot
    def move_and_detect(obj_type):
        NavigateAction(target_locations=[cheatcode[environment_path][2]]).resolve().perform()

        LookAtAction(targets=[milk_pos]).resolve().perform()

        object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

        return object_desig

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.25]).resolve().perform()

        milk_desig = move_and_detect(ObjectType.MILK)

        TransportAction(milk_desig, [Arms.LEFT], [put_down_pos]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

# param_plan2("pr2.urdf","/home/nleusmann/catkin_ws/src/pycram/resources/tmp/house_6/house_6.urdf")

def loader():
    for i in range(15):
        print(i)
        world = BulletWorld(WorldMode.GUI)
        # world= Multiverse(simulation='pycram_test')
        apartment = Object("house6", ObjectType.ENVIRONMENT,"/home/nleusmann/catkin_ws/src/pycram/resources/apartment.urdf")
        world.remove_object_from_simulator(apartment)

# loader()


def generic_plan(inWorld):
    world = inWorld

    # milk_pos = Pose([1, -1.78, 0.55], [1, 0, 0, 0])
    # milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1, -1.78, 0.55], [1, 0, 0, 0]),
    #               color=Color(1, 0, 0, 1))

    pick_pose = Pose([1, -1.78, 0.55])

    robot_desig = BelieveObject(names=["pr2"])
    apartment_desig = BelieveObject(names=["apartment"])

    @with_simulated_robot
    def move_and_detect(obj_type):
        NavigateAction(target_locations=[Pose([2, -1.89, 0])]).resolve().perform()

        LookAtAction(targets=[pick_pose]).resolve().perform()

        object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

        return object_desig

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.25]).resolve().perform()

        milk_desig = move_and_detect(ObjectType.MILK)

        TransportAction(milk_desig, [Arms.LEFT], [Pose([4.8, 3.55, 0.8])]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()