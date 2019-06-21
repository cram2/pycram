import sys
sys.path.append('/home/jonas/Documents/Studium/BA/pycram/src/')

from pycram.bullet_world import stable, contact, Object, BulletWorld
import time
import pybullet as p

world = BulletWorld()
plane = Object("floor", "plane.urdf", world=world)
obj = Object("box", "box.urdf", [0, 0, 2], world)

world.set_realtime()
time.sleep(10)
#print(p.getContactPoints(plane.id ,obj.id))
print(contact(plane, obj))
#world.simulate(2500)

