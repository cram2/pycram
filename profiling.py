import yappi
from pycram.bullet_world import BulletWorld, Object
import pycram.costmaps as c
import time

world = BulletWorld()
kitchen = Object("Kitchen", "env", "kitchen.urdf")
time.sleep(0.5)

yappi.set_clock_type("cpu") # Use set_clock_type("wall") for wall time
yappi.start()

#o = c.OccupancyCostmap(0.2, False, 200, 0.2)

#s = c.SemanticCostmap(kitchen, "kitchen_island_surface")

#g = c.GaussianCostmap(200, 2)

print("before")
v = c.VisibilityCostmap(1, 1.5)
print("after")


yappi.get_func_stats().print_all()
yappi.get_thread_stats().print_all()

world.exit()
