"""QUAD: four coaxial boards lifting a frame, full tilt into the sky and a stop 10 cm up."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'QUAD', 'D', 'four rotors, full tilt and back'
ORDER, NAME = 66, 'quad'


def run(args, name):
    from terminal.views import show_quad
    return call(show_quad.main, common(args))
