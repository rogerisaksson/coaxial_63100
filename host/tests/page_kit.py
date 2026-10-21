"""The QUAD page headless: its flight set up in its process, its steps read as the page takes them."""
import math
import time

#: The page's flights drawn this long at most, s.
LANDED_S = 200.0


def flown_as(flown, card, pack_ah):
    """The flight's process set up (`flying.Flying`): `card`, a pack of `pack_ah` A h."""
    from machine import flightpack
    flown.CARD = card
    if pack_ah:
        flightpack.PACK_AH = pack_ah


def paged(card, done, pack_ah=None):
    """The page headless on `card` - and a pack of `pack_ah` A h - until `done(the stages flown,
    the one it is in)` at a frame, LANDED_S drawn at the most: (a row a step of its flight, as
    the page took it, a row a frame - its stage, its art's braille cells, the gate lit - its
    frames a second)."""
    from terminal.ui.screen import FPS_CAP
    from terminal.views import show_quad as view
    from terminal.views.quad import flight as flown
    from tools.render import page
    steps, frames, flew = [], [], set()
    was = view.compose, view.flying.Flying, view.Ahead

    class Done(Exception):
        """What it was flown for is flown."""

    class Flying(was[1]):
        def __init__(self, craft, flown_mod, count, port=None):
            super().__init__(craft, flown_mod, count, setup=(flown_as, (card, pack_ah)), port=port)

        def said(self):
            out = super().said()
            for state in out:
                frame, flight, rotors = state['frame'], state['flight'], state['rotors']
                lap, cells = flight['lap'] or {}, flight['cells']
                flew.add(flight['stage'])
                steps.append({'name': flight['stage'], 't': time.monotonic(), 'dt': flown.STEP_S,
                              'x': float(frame['at'][0]), 'y': frame['h'], 'z': float(frame['at'][2]),
                              'v': math.hypot(*(float(c) for c in frame['vel'])),
                              'share': flight['share'], 'laps': lap.get('laps', 0), 'of': lap.get('of', 0),
                              'amps': max(r['amps'] for r in rotors),
                              'soa': max((r['budget'] or {}).get('worst') or 0.0 for r in rotors),
                              'states': [(r['ident'] or {}).get('state') for r in rotors],
                              'tripped': any((r['budget'] or {}).get('tripped') for r in rotors),
                              'volts': cells['volts'], 'watts': cells['watts'], 'left': cells['left']})
            return out

    class Drawing(was[2]):
        """The page's drawing, a frame recorded as it is asked: its stage, the gate lit."""

        def drawn(self, *args, **kw):
            frames.append({'t': time.monotonic(), 'name': steps[-1]['name'] if steps else '',
                           'gate': kw.get('gate'), 'cells': 0})
            return super().drawn(*args, **kw)

    def compose(console, origin, rotors, frame, flight, trace, now, art, **kw):
        frames[-1]['cells'] = sum(0x2800 < ord(c) <= 0x28FF for c in art)
        # Flown, or LANDED_S of the wall's time drawn.
        if done(flew, flight['stage']) or frames[-1]['t'] - frames[0]['t'] > LANDED_S:
            raise Done
        return was[0](console, origin, rotors, frame, flight, trace, now, art, **kw)
    view.compose, view.flying.Flying, view.Ahead = compose, Flying, Drawing
    try:
        page.frame('quad', 150, 44, frames=int(LANDED_S * FPS_CAP))
    except Done:
        pass
    finally:
        view.compose, view.flying.Flying, view.Ahead = was
    return steps, frames, (len(frames) - 1) / max(1e-9, frames[-1]['t'] - frames[0]['t'])
