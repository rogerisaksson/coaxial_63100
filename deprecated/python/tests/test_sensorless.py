"""Moved from host/tests/test_sensorless.py: stand_in_drive - nothing in the tree named it."""
from coaxial.simulated import SimulatedDrive


def stand_in_drive(rig):
    """The stand-in's own drive, which these checks reach inside."""
    drive = rig.board.drive
    if not isinstance(drive, SimulatedDrive):
        raise AssertionError('not the stand-in: %r' % (drive,))
    return drive
