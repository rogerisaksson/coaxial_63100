"""BOARD ATTITUDE: the board turned by the IMU, drawn from the STL."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'BOARD ATTITUDE', 'B', 'board orientation visualizer'
ORDER, NAME = 20, 'imu'


def run(args, name):
    from terminal.views import show_orientation
    return call(show_orientation.main, common(args, hz=20.0))
