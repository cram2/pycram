class PlanFailure(Exception):
    """Implementation of plan failures."""

    def __init__(self, *args, **kwargs):
        """Create a new plan failure."""
        Exception.__init__(self, *args, **kwargs)