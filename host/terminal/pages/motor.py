"""MOTOR CONTROLLER: the gate drivers, or the rotor observer - the
second question."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = ('MOTOR CONTROLLER', 'G',
                       'the gate drivers, or the rotor observer')
ORDER, NAME = 50, 'motor'
ITEMS = ('which half', (
    ('G', 'GATE DRIVERS', 'half bridge control', 'gate_drivers'),
    ('R', 'ROTOR OBSERVER', 'the drive, on the model or the converters',
     'rotor_observer')))


def run(args, name):
    if name == 'rotor_observer':
        import show_rotor_observer
        return call(show_rotor_observer.main,
                    common(args, hz=8.0) + ['--source', 'model'])
    import show_gate_drivers
    return call(show_gate_drivers.main, common(args, hz=8.0))
