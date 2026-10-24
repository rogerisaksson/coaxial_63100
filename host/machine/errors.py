"""What the machine layer raises: one class, a refusal in words."""


class MachineError(Exception):
    """A refusal or a failure, in words; a board family's errors subclass it."""


class BusFault(MachineError):
    """A bus whose frames are refused systematically: her drives shut down."""
