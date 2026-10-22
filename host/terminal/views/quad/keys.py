"""A flying page's keys: its rows of the terminal's one table (PAD) on its `held` state.

    run_view(.., pad=keys.PAD, state=held)     # held['zoomed'], ['record'], ['send'] its own

The quad's, the helicopter's, the ornithopter's and the aerobat's alike (`show_quad.run_craft_view`):
the view's zoom (`zoomed`, k), R's recording (`record`), the flight's commands (`send`,
`flying.Flight.apply`); every key kept for the recording's next row (`show_quad.flown_row`).
"""
from coaxial.graphics import quadcopter
from terminal.ui import bindings

#: A key's zoom on the view, the wheel's a notch (`show_quad.wheeled`).
ZOOM_STEP = 1.15


def _keyed(held, key, _now):
    """A key typed, kept for the recording's next row; its row acts on it."""
    held['keyed'] = held.get('keyed', '') + key
    return bindings.PASS


def _toggled(name):
    """The act that turns `held[name]` on, or off."""
    return bindings.did(lambda held: held.update({name: not held[name]}))


_did, _cased = bindings.did, bindings.cased
ROWS = (
    ('VIEW', '+ -', 'nearer, farther - the wheel too', (('+ -', 'ZOOM'),),
     dict.fromkeys('+=-_', lambda held, key, _now: held['zoomed'](ZOOM_STEP if key in '+=' else 1.0 / ZOOM_STEP))),
    ('FLIGHT', 'W', 'the wind blowing, or still', (('W', 'WIND'),), _cased('w', _did(lambda held: held['send'](wind=True)))),
    ('VIEW', 'T', "the rotors' thrust drawn, or not", (('T', 'THRUST'),), _cased('t', _toggled('arrows'))),
    ('VIEW', 'S', 'the next of what is drawn: %s' % ', '.join(quadcopter.LAYERS), (('S', 'LAYER'),),
     _cased('s', _did(lambda held: held.update(see=(held['see'] + 1) % len(quadcopter.LAYERS))))),
    ('VIEW', 'L', "the course's line drawn, or not", (('L', 'LINE'),), _cased('l', _toggled('line'))),
    ('VIEW', 'C', "the carried camera's picture, or none", (('C', 'CAMERA'),), _cased('c', _toggled('carried'))),
    ('FLIGHT', 'R', 'the flight recorded / saved', (('R', 'RECORD'),), _cased('r', _did(lambda held: held['record']()))),
    ('FLIGHT', 'A', 'the flight laid anew', (('A', 'RESET'),), _cased('a', _did(lambda held: held['send'](reset=True)))),
)
PAD = bindings.page(ROWS, take=_keyed)
