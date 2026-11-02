"""Moved from host/machine/process/worker.py: WINDOW_STEPS - nothing in the tree named it."""
from machine.identify import wear
from machine.process.worker import DT


WINDOW_STEPS = int(round(wear.WINDOW_S / DT))
