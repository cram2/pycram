from pycram.projection import macros, simulated_robot
from pycram.bullet_world import stable, Object, BulletWorld

world = BulletWorld()
obj = Object("box", "box.urdf", world)
#print(world.client_id)

'''
with simulated_robot as world:
    #print(world)
    o1 = Object("box", "box.urdf", world)
    s = stable(o1.id)
    print(str(s))
'''
