"""SESSION: one dash over one port - analog, thermals, bridges, DIO, the
IMU, the shaft angle."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'SESSION', 'S', 'board dashpanel'
ORDER, NAME = 10, 'session'


def run(args, name):
    from terminal.views import show_session
    return call(show_session.main, common(args))
