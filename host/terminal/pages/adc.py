"""METER BRIDGE: every analog channel on a meter bridge."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'METER BRIDGE', 'M', 'metered channels'
ORDER, NAME = 40, 'adc'


def run(args, name):
    from terminal.views import show_desk
    return call(show_desk.main, common(args, hz=8.0))
