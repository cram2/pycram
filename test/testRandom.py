import math

import pycrap
from pycram.testing import BulletWorldTestCase
from pycram.process_module import ProcessModuleManager, ProcessModule, real_robot
from pycram.robot_description import RobotDescriptionManager
from pycram.datastructures.enums import ExecutionType
from pycram.ros.ros_tools import sleep
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot
from pycram.world_concepts.world_object import Object
from pycram.yarp_utils.yarp_joint_state_updater import run_icub_state_updater
from pycram.yarp_utils.yarp_networking import interrupt_and_close


def run_look_and_move_tcp():
    rm = RobotDescriptionManager()
    rm.load_description('icub')

    with simulated_robot:
        pm = ProcessModuleManager.get_manager()

        LookingMotion(Pose([-2, -2, 3])).perform()
        time.sleep(2)
        MoveTCPMotion(Pose([-0.2, 0.1, 0.1]), Arms.LEFT).perform()
        MoveJointsMotion(["torso_roll"], [20.0]).perform()

        pm.exit()


class ICUBTestCase(BulletWorldTestCase):

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld(mode=WorldMode.DIRECT)
        # cls.milk = Object("milk", pycrap.Milk, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        cls.robot = Object("icub", pycrap.Robot, "icub" + cls.extension, pose=Pose([0, 0, 0.5]))
        cls.kitchen = Object("kitchen", pycrap.Kitchen, "kitchen" + cls.extension)
        # cls.cereal = Object("cereal", pycrap.Cereal, "breakfast_cereal.stl",
        #                     pose=Pose([1.3, 0.7, 0.95]))
        ProcessModule.execution_delay = False
        ProcessModuleManager.execution_type = ExecutionType.SIMULATED
        cls.viz_marker_publisher = VizMarkerPublisher()

    def test_move_joint_direct(self):
        self.robot.set_joint_position("l_hand_thumb_0_joint",math.radians(30))


    def test_move_joints_simulated(self):
        with simulated_robot:
            MoveJointsMotion(["torso_roll"], [math.radians(-20.0)]).perform()
            sleep(3)
            ParkArmsActionPerformable(arm=Arms.BOTH).perform()
            sleep(3)
            MoveArmJointsMotion(left_arm_poses={"l_elbow": math.radians(20.0)}).perform()
            sleep(3)
            MoveArmJointsMotion(right_arm_poses={"r_elbow": math.radians(40.0)}).perform()
            sleep(3)
            LookingMotion(Pose([-2, -2, 3])).perform()
            sleep(100)




    def test_move_joints(self):


        with real_robot:
            yarpModule = run_icub_state_updater(['/', '--robot', 'icubSim'])
            if yarpModule is None:
                print("Failed to open yarp module")
            MoveJointsMotion(["torso_roll"], [math.radians(-20.0)]).perform()

            ParkArmsActionPerformable(arm=Arms.BOTH).perform()

            MoveArmJointsMotion(left_arm_poses={"l_elbow": math.radians(20.0)}).perform()

            MoveArmJointsMotion(right_arm_poses={"r_elbow": math.radians(40.0)}).perform()
            LookingMotion(Pose([-2, -2, 3])).perform()

            if yarpModule is None:
                print("stopping the module")
                interrupt_and_close(yarpModule)


    def test_park_arms(self):
        with real_robot:
            ParkArmsActionPerformable(arm=Arms.BOTH).perform()


    def test_try_world(self):
        sleep(100)

# run_look_and_move_tcp()
