from dataclasses import dataclass

class SomeClass:
    pass

@dataclass
class TestClass:
    f: float
    """This is a float variable"""
    b: str
    """
    This is a string variable
    named b.
    """
    a: str = "Var A"
    """
    This is a string variable
    named a with a default value.
    """
    c = "Not annotated"
    d = None
    "Comment for d"
    e: SomeClass = SomeClass()
    "eeee"