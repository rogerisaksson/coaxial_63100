"""The robot's manifest's numbers: the (defrobot NAME ..) its gym's text includes.
Written by `tools/cores/manifest.py`.

    r = defrobot.read()      # {name, body, modes, drives}: the one named `mechatronics.ROBOT`
"""
import os

from machine.body import mechatronics
from machine.texts import ortho


def read(text=None, here=None):
    """The robot's manifest's (defrobot ..) in `text` (or the robot's gym's file, `ortho._FILE`, and its includes),
    read: {name, body {key: n}, modes {mode: {key: n}}, drives {joint: {key: v}}}; the one named ROBOT where a text
    extends another robot's, else the first; None with none."""
    here = here or os.path.dirname(ortho._FILE)
    text = ortho.source(ortho._FILE) if text is None else text
    found = [ortho.robot_form(f) for f in ortho._forms(text, here) if isinstance(f, list) and f and f[0] == 'defrobot']
    return next((r for r in found if r['name'] == mechatronics.ROBOT), found[0] if found else None)
