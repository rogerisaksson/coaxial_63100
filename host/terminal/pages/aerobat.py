"""AEROBAT: two coaxial boards flying a 3D aerobatic monoplane, prop-hang stunts and gates."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'AEROBAT', 'P', '3D aerobatic monoplane, prop-hang stunts and gates'
ORDER, NAME = 69, 'aerobat'


def run(args, name):
    from terminal.views import show_aerobat
    return call(show_aerobat.main, common(args))
