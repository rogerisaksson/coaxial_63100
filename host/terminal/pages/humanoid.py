"""HUMANOID: a slender gynoid walking, her twenty joints on virtual actuators."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'HUMANOID', 'H', 'gynoid on virtual actuators'
ORDER, NAME = 65, 'humanoid'


def run(args, name):
    from terminal.views import show_humanoid
    return call(show_humanoid.main, common(args, hz=30.0))
