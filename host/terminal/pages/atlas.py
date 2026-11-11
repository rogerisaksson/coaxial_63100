"""SYSTEM ATLAS: the system as a tree in 3D, the terminal up to the shunts, opened a node at a time."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'SYSTEM ATLAS', 'V', '3D tree, the terminal up to the shunts'
ORDER, NAME = 80, 'atlas'


def run(args, name):
    from terminal.views import show_atlas
    return call(show_atlas.main, common(args))
