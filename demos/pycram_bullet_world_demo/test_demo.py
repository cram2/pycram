from pycram.datastructures.world import World

try:
    import demo
except Exception as e:
    World.current_world.exit()
    raise e