"""Moved from host/machine/world/room.py: door_open - nothing in the tree named it."""
import math


def door_open(world):
    """How far the door stands open, deg."""
    m = world.model
    return math.degrees(float(world.data.qpos[m.jnt_qposadr[m.joint('door').id]]))
