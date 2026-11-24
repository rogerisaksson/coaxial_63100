"""HELICOPTER: two coaxial boards flying a 3D helicopter, its aerobatic stunts and gates."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'HELICOPTER 3D FLYING', 'E', '3D helicopter, collective stunts and gates'
ORDER, NAME = 67, 'helicopter'
#: The control system its page lays, compiled behind the front page while it is behind (`terminal.compiling`).
COMPILED = 'helicopter'


def run(args, name):
    from terminal.views import show_helicopter
    return call(show_helicopter.main, common(args))
