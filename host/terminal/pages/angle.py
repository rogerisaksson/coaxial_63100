"""SHAFT ANGLE: the magnet and the air gap, on a protractor."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'SHAFT ANGLE', 'A', 'motor axle rotation position'
ORDER, NAME = 30, 'angle'


def run(args, name):
    import show_angle
    return call(show_angle.main, common(args, hz=20.0))
