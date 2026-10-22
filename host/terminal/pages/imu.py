"""BOARD ATTITUDE: the board turned by the IMU, drawn from the STL.

Light to import - the loader's own imports in `run` -: `python -m terminal imu` reads its AHEAD
before the loader is imported.
"""

HEADLINE, KEY, WHAT = 'BOARD ATTITUDE', 'B', 'board orientation visualizer'
ORDER, NAME = 20, 'imu'

#: Its drawing's process (terminal.ui.ahead), started as the terminal opens: up when it is picked.
AHEAD = ('coaxial.draw.easel', 'render')


def run(args, name):
    from terminal.loader import call, common
    from terminal.views import show_orientation
    return call(show_orientation.main, common(args))
