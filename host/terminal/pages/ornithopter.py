"""ORNITHOPTER: two coaxial boards flying a 4-wing Dune ornithopter, its stunts and gates."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'ORNITHOPTER', 'O', '4-wing Dune ornithopter, wing-fold dives and gates'
ORDER, NAME = 68, 'ornithopter'


def run(args, name):
    from terminal.views import show_ornithopter
    return call(show_ornithopter.main, common(args))
