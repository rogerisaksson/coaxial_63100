"""THERMAL OBSERVER: where the heat sits, drawn on the board itself."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'THERMAL OBSERVER', 'T', 'thermals estimation'
ORDER, NAME = 60, 'thermal_observer'


def run(args, name):
    import show_thermal_observer
    return call(show_thermal_observer.main, common(args, hz=2.0))
