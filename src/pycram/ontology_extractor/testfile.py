from dataclasses import dataclass

class SomeClass:
    pass


@dataclass
class TestClass:
    f: float # aaaaa
    """
    This is a float variable
    """

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
    """
    Comment for d
    """

    e: SomeClass = SomeClass()
    """
    eeee
    """

    g = h = i = 3
    """
    Multi Assignment
    """

    def __init__(self):
        self.affe = 2
        self.banane = "banana"

    def funcy(self, varry):
        varry = "lol"
        return varry