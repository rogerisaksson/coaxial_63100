"""AIRCRAFT XE AEROBATICS: two coaxial boards flying a 3D monoplane, its stunts and gates."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'AIRCRAFT XE AEROBATICS', 'P', '3D aerobatic monoplane, prop-hang stunts and gates'
ORDER, NAME = 69, 'aerobat'
#: The control system its page lays, compiled behind the front page while it is behind (`terminal.compiling`).
COMPILED = 'aerobat'


def run(args, name):
    from terminal.views import show_aerobat
    return call(show_aerobat.main, common(args))
