"""Moved from host/machine/process/sprung.py: _heard_say - nothing in the tree named it."""


def _heard_say(g, gym, entry):
    from machine.process import worker
    return worker.heard_say(g, gym, entry)
