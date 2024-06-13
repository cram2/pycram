from .aspects import ReachableAspect


def is_reachable(entity: ReachableAspect) -> bool:
    entity.pose = None
