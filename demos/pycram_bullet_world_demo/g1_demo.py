import time
from math import sin, cos
from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import PoseStamped
from pycram.process_module import simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription as URDFObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
from pycram.robot_plans.motions.robot_body import MoveJointsMotion
from pycrap.ontologies import Robot, Apartment, Milk

# ---------- helpers ----------
def q_from_yaw(yaw):
    h = 0.5 * yaw
    return [0.0, 0.0, sin(h), cos(h)]

def pause(sec=1.0):
    time.sleep(sec)

def set_base(robot, x, y, z, yaw, wait=1.0):
    robot.set_pose(PoseStamped.from_list([x, y, z], q_from_yaw(yaw)))
    pause(wait)

def _get_current_positions(robot, names):
    cur = []
    for n in names:
        try:
            cur.append(float(robot.get_joint_position(n)))
        except Exception:
            cur.append(0.0)
    return cur

def set_joints(robot, names, target_values, seconds=1.0, steps=24):
    """Interpolate joints from current -> target in small steps"""
    start = _get_current_positions(robot, names)
    tgt   = [float(v) for v in target_values]
    if steps < 1:
        steps = 1
    dt = max(seconds / steps, 0.001)

    for i in range(1, steps + 1):
        a = i / float(steps)
        pos = [s + (t - s) * a for s, t in zip(start, tgt)]
        MoveJointsMotion(names=names, positions=pos).perform()
        time.sleep(dt)

def set_joint(robot, name, value, seconds=1.0, steps=24):
    set_joints(robot, [name], [value], seconds=seconds, steps=steps)

def set_obj_pose(obj, x, y, z, wait=0.8):
    obj.set_pose(PoseStamped.from_list([x, y, z], [0, 0, 0, 1]))
    pause(wait)

def pick_and_place(robot, obj, pick_base, place_base, final_xyz, wrist_link):
    px, py, pz, pyaw = pick_base
    tx, ty, tz, tyaw = place_base
    fx, fy, fz = final_xyz

    # move base
    set_base(robot, px, py, pz, pyaw, wait=1.0)

    # elbows together
    set_joints(
        robot,
        names=["left_elbow_joint", "right_elbow_joint"],
        target_values=[1.00, 1.00],
        seconds=1.0, steps=28
    )

    # reach to pre-grasp
    set_joint(robot, "right_shoulder_pitch_joint", -1.00, seconds=0.9, steps=22)
    set_joint(robot, "right_elbow_joint",          1.10, seconds=0.9, steps=22)

    # attach
    try:
        robot.attach(obj, wrist_link)
    except Exception:
        pass
    pause(0.6)

    # small lift
    set_joint(robot, "right_elbow_joint", 0.70, seconds=0.9, steps=22)


    set_base(robot, tx, ty, tz, tyaw, wait=1.0)

    # place posture
    set_joint(robot, "right_shoulder_pitch_joint", -0.60, seconds=0.9, steps=22)
    set_joint(robot, "right_elbow_joint",          1.10, seconds=0.9, steps=22)

    # detach
    try:
        robot.detach(obj)
    except Exception:
        pass
    pause(0.5)

    set_obj_pose(obj, fx, fy, fz, wait=0.8)

    # stow right arm
    set_joint(robot, "right_elbow_joint",          1.00, seconds=0.9, steps=22)
    set_joint(robot, "right_shoulder_pitch_joint", 0.00, seconds=0.9, steps=22)


# ---------- world / robot / props ----------
world = BulletWorld(WorldMode.GUI)
extension = URDFObjectDescription.get_file_extension()

robot = Object("g1", Robot, f"g1{extension}", pose=PoseStamped.from_list([1.00, 2.00, 0.80]))
apartment = Object("apartment", Apartment, f"apartment{extension}")

milk1 = Object("milk_1", Milk, "milk.stl",
               pose=PoseStamped.from_list([2.50, 2.01, 1.02], [0, 0, 0, 1]),
               color=Color(1.0, 0.2, 0.2, 1))
milk2 = Object("milk_2", Milk, "milk.stl",
               pose=PoseStamped.from_list([2.50, 1.81, 1.02], [0, 0, 0, 1]),
               color=Color(0.2, 1.0, 0.2, 1))
milk3 = Object("milk_3", Milk, "milk.stl",
               pose=PoseStamped.from_list([2.50, 1.61, 1.02], [0, 0, 0, 1]),
               color=Color(0.2, 0.4, 1.0, 1))

print("[INIT]  Unitree G1 at (1.00, 2.00, 0.80)")

RIGHT_WRIST = "right_wrist_roll_rubber_hand"

pick_bases  = [
    (2.05, 2.02, 0.80, 0.00),
    (2.05, 1.82, 0.80, 0.00),
    (2.05, 1.62, 0.80, 0.00),
]
place_bases = [
    (4.80, 3.00, 0.80, 1.50),
    (5.00, 3.00, 0.80, 1.50),
    (5.20, 3.00, 0.80, 1.50),
]
final_poses = [
    (4.89, 3.34, 0.80),
    (5.09, 3.34, 0.80),
    (5.29, 3.34, 0.80),
]

# ---------- sequence ----------
with simulated_robot:
    print("\n=== Pick & place of 1st milk carton ===")
    pick_and_place(robot, milk1, pick_bases[0], place_bases[0], final_poses[0], RIGHT_WRIST)
    print("\n=== Pick & place of 2nd milk carton ===")
    pick_and_place(robot, milk2, pick_bases[1], place_bases[1], final_poses[1], RIGHT_WRIST)
    print("\n=== Pick & place of 3rd milk carton ===")
    pick_and_place(robot, milk3, pick_bases[2], place_bases[2], final_poses[2], RIGHT_WRIST)

    print("\n=== DONE ===")

pause(6)
world.exit()