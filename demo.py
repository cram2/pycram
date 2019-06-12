from pycram.projection import macros, simulated_robot
from pycram.bullet_world import stable, Object

with simulated_robot:
    o1 = Object("box", "box.urdf")
    s = stable(o1.id)
    print(str(s))
