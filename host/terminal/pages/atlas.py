"""SYSTEM ATLAS: the trunk from the terminal up to the shunts, every layer, folder, file and tie."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'SYSTEM ATLAS', 'V', 'the system from the terminal up to the shunts'
ORDER, NAME = 80, 'atlas'


def run(args, name):
    from terminal.views import show_atlas
    return call(show_atlas.main, common(args))
