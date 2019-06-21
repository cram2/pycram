import sys
sys.path.append('/home/jonas/Documents/Studium/BA/pycram/src/')

from pycram.bullet_world import stable, contact, Object, BulletWorld
import time
import pybullet as p

world = BulletWorld()
plane = Object("floor", "plane.urdf", world=world)
obj = Object("box", "box.urdf", [0, 0, 2], world)

world.set_realtime()
#world.simulate(2500)
time.sleep(10)


'''
with simulated_robot as world:
    #print(world)
    o1 = Object("box", "box.urdf", world)
    s = stable(o1.id)
    print(str(s))
'''
