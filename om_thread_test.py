from pycram.bullet_world import BulletWorld, Object
import pycram.costmaps as c
import time

world = BulletWorld()
kitchen = Object("Kitchen", "env", "kitchen.urdf")

o = c.OccupancyCostmap(0.2, False, 200, 0.2)
