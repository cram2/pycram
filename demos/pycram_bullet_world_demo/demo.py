from pycram.external_interfaces.ik import request_ik
from pycram.plan_failures import IKError
from pycram.ros.viz_marker_publisher import VizMarkerPublisher, AxisMarkerPublisher, CostmapPublisher
from pycram.utils import _apply_ik
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode, TorsoState
from pycram.datastructures.pose import Pose, Transform
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color

extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.DIRECT)
viz = VizMarkerPublisher()
robot_name = "pr2"
robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}{extension}", pose=Pose([1, 2, 0]))

apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment-small{extension}")

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]),
              color=Color(1, 0, 0, 1))
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                pose=Pose([2.5, 2.5, 1.05]), color=Color(0, 1, 0, 1))
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.4, 2.2, 0.98]),
              color=Color(1, 1, 0, 1))
spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]),
               color=Color(0, 0, 1, 1))
apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=[robot_name])
apartment_desig = BelieveObject(names=["apartment"])


@with_simulated_robot
def move_and_detect(obj_type):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig


with simulated_robot:
    NavigateAction([Pose([0, 0, 0])]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()
    grid_test = True
    if grid_test:
        # Define a 3D grid for the costmap with dimensions in centimeters
        grid_resolution = 0.1  # 5cm per voxel (0.05m)
        costmap_size_cm = 250  # 200cm or 2m in each direction

        # Calculate the number of voxels based on the resolution and size
        grid_size = (int(costmap_size_cm / (grid_resolution * 100)),
                     int(costmap_size_cm / (grid_resolution * 100)),
                     int(costmap_size_cm / (grid_resolution * 100)))  # 40x40x40

        costmap_3d = np.zeros(grid_size)

        start_pose = Pose([-1, -1, 0])

        NavigateAction([start_pose]).resolve().perform()

        # Robot starts at position [0, 0, 0] and is rotated using quaternion [0, 0, 0, 1]
        grid_origin = start_pose.position_as_list()  # Centered at [0, 0, 0]
        grid_origin = [grid_origin[0]+0.1, grid_origin[1]+0.1, costmap_size_cm / 100 / 2]

        def grid_to_world(grid_index, grid_origin, resolution):
            size = grid_size[0]
            center = np.array([size // 2, size // 2, size // 2])
            grid_origin = Pose(grid_origin)
            # The position is calculated by creating a vector from the 2D position in the costmap (given by x and y)
            # and the center of the costmap (since this is the origin). This vector is then turned into a transformation
            # and muiltiplied with the transformation of the origin.
            vector_to_origin = (center - grid_index) * resolution
            point_to_origin = Transform([*vector_to_origin], frame="point", child_frame="origin")
            origin_to_map = grid_origin.to_transform("origin").invert()
            point_to_map = point_to_origin * origin_to_map
            map_to_point = point_to_map.invert()
            return Pose(map_to_point.translation_as_list())


        poses = []
        arm = Arms.LEFT
        current_pose = start_pose


        def is_reachable(test_robot, target_pose, arm=None):
            test_robot = World.robot
            l_gripper = RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT).get_tool_frame()
            r_gripper = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame()
            l_joints = RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT).joints
            r_joints = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).joints

            try:
                inv = request_ik(target_pose, test_robot, l_joints, l_gripper)
            except IKError as e:
                try:
                    inv = request_ik(target_pose, test_robot, r_joints, r_gripper)
                except IKError as e:
                    # print(e)
                    return False
            print("reached")
            _apply_ik(test_robot, inv)
            return True


        def euclidean_distance(pose1: Pose, pose2: Pose):
            point1 = pose1.position_as_list()
            point2 = pose2.position_as_list()
            return np.sqrt(np.sum((np.array(point1) - np.array(point2)) ** 2))


        robot_object = robot_desig.resolve()
        test_robot = World.current_world.get_prospection_object_for_object(robot_object.world_object)

        voxels = itertools.product(range(grid_size[0]), range(grid_size[1]), range(grid_size[2]))
        pub = False
        if pub:
            ps = []
            for x, y, z in tqdm(voxels, total=grid_size[0] * grid_size[1] * grid_size[2]):
                target_pos = grid_to_world([x, y, z], grid_origin, grid_resolution)
                ps.append(target_pos)

            marker = CostmapPublisher()
            marker.publish(ps, 0.02)
        else:
            marker = CostmapPublisher()
        max_distance = RobotDescription.current_robot_description.get_max_reach()
        print(max_distance)
        for x, y, z in tqdm(voxels, total=grid_size[0] * grid_size[1] * grid_size[2]):
            # Convert the grid index to a world position
            target_pos = grid_to_world([x, y, z], grid_origin, grid_resolution)
            target = target_pos.position_as_list()
            # Check if the position is in front of the robot (x-coordinate >= -1)
            # Skip poses further than 1.5 meters from the origin
            if euclidean_distance(target_pos, Pose(grid_origin)) > max_distance:
                continue
            if target[0] >= -0.5 and (target[1] <= -1.25 or target[1] >= -0.25) and target[2] < 2:
                ParkArmsAction([Arms.BOTH]).resolve().perform()
                # Use IK solver to check if this position is reachable
                if is_reachable(test_robot, target_pos):
                    cpy: Pose = target_pos.copy()
                    marker.publish([cpy, target_pos], 0.02)
                    costmap_3d[x, y, z] = 1  # Mark as reachable
                else:
                    marker.publish([target_pos], 0.02)
                    costmap_3d[x, y, z] = 0  # Mark as unreachable

        # Collapse the 3D costmap to 2D by summing over the Z-axis (height)
        costmap_2d = np.sum(costmap_3d, axis=2)

        # Normalize the 2D costmap to a desired range (0 to 1)
        costmap_2d_normalized = costmap_2d / np.max(costmap_2d)

        # Visualize the 2D costmap
        import matplotlib.pyplot as plt

        plt.imshow(costmap_2d_normalized, cmap='hot', interpolation='nearest')
        plt.colorbar()
        plt.show()


    else:

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([TorsoState.HIGH]).resolve().perform()

        milk_desig = move_and_detect(ObjectType.MILK)

        TransportAction(milk_desig, [Arms.LEFT], [Pose([4.8, 3.55, 0.8])]).resolve().perform()

        cereal_desig = move_and_detect(ObjectType.BREAKFAST_CEREAL)

        TransportAction(cereal_desig, [Arms.LEFT], [Pose([5.2, 3.4, 0.8], [0, 0, 1, 1])]).resolve().perform()

        bowl_desig = move_and_detect(ObjectType.BOWL)

        TransportAction(bowl_desig, [Arms.LEFT], [Pose([5, 3.3, 0.8], [0, 0, 1, 1])]).resolve().perform()

        # Finding and navigating to the drawer holding the spoon
        handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())

        if robot.name == "rollin_justin":
            pose = Pose([1.4, 1.6, 0], [0, 0, 0.2040033016133158, 0.9789702002261697])
            drawer_open_loc = AccessingLocation.Location(pose, [Arms.LEFT])
        else:
            drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
                                                robot_desig=robot_desig.resolve()).resolve()

        NavigateAction([drawer_open_loc.pose]).resolve().perform()

        OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
        spoon.detach(apartment)

        # Detect and pickup the spoon
        LookAtAction([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        spoon_desig = DetectAction(BelieveObject(types=[ObjectType.SPOON])).resolve().perform()

        if robot.name == "iai_donbot":
            ParkArmsAction([Arms.BOTH]).resolve().perform()
            PickUpAction(spoon_desig, [Arms.LEFT], [Grasp.TOP]).resolve().perform()

            ParkArmsAction([Arms.BOTH]).resolve().perform()

            # Find a pose to place the spoon, move and then place it
            spoon_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])
            placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve()).resolve()

            NavigateAction([placing_loc.pose]).resolve().perform()

            PlaceAction(spoon_desig, [spoon_target_pose], [Arms.LEFT]).resolve().perform()

            ParkArmsAction([Arms.BOTH]).resolve().perform()

            NavigateAction([drawer_open_loc.pose]).resolve().perform()

            CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()

            ParkArmsAction([Arms.BOTH]).resolve().perform()

        else:
            if robot.name == "tiago_dual":
                NavigateAction([Pose([1.45, 2.7, 0], [0, 0, 0, 1])]).resolve().perform()

            pickup_arm = Arms.LEFT if drawer_open_loc.arms[0] == Arms.RIGHT else Arms.RIGHT
            ParkArmsAction([Arms.BOTH]).resolve().perform()
            PickUpAction(spoon_desig, [pickup_arm], [Grasp.TOP]).resolve().perform()

            ParkArmsAction([Arms.LEFT if pickup_arm == Arms.LEFT else Arms.RIGHT]).resolve().perform()

            NavigateAction([drawer_open_loc.pose]).resolve().perform()

            CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()

            ParkArmsAction([Arms.BOTH]).resolve().perform()

            MoveTorsoAction([TorsoState.MID]).resolve().perform()

            # Find a pose to place the spoon, move and then place it
            spoon_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])
            placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve(),
                                          reachable_arm=pickup_arm, used_grasps=[Grasp.TOP]).resolve()

            NavigateAction([placing_loc.pose]).resolve().perform()

            PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()

            ParkArmsAction([Arms.BOTH]).resolve().perform()
