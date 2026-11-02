"""Moved from host/tools/sim/gvs.py: ALL_JOINTS - nothing in the tree named it."""
from machine.body import figure
from tools.sim import fbx


ALL_JOINTS = tuple(fbx.bone(seg[0]) for seg in figure.SEGMENTS)
