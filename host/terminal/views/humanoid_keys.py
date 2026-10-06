"""The HUMANOID page's keys: what each does to the view's state and asks of her body.

    KEYS = humanoid_keys.table(viewpoint.KEYS, CALLING, SHOWN)
    humanoid_keys.act_on(KEYS, typed, state, wheel)

The view (`terminal.views.show_humanoid`) holds the state and draws it; a key's handler takes
that state, moves it and sends her body (`machine.running`) what it asks. R's recordings and
their rows (HEADER, `row`) are here with the key that writes them.
"""
import csv
import math
import os
import time

from machine import gaits, style
from machine.figure import JOINTS, SEGMENTS, frames, quat
from terminal.views import viewpoint
from tools import REPO

#: The cadence's bounds and a key's step, strides a second: under 0.6 she fell from her first
#: stride (2026-09-25). At 1.0 she walks 0.90 m/s, four minutes on with her hottest drive at
#: 0.35 of its span; at 1.05 1.0-1.08 m/s, at 1.10 down in 2 s (2026-10-06) - 0.9 till then:
#: the meter one cell up (the user).
CADENCE, CADENCE_STEP = (0.6, 1.0), 0.05

#: J: her going on the one law (`machine.pace`), S and F then a step of LEVELS on her way
#: (`gaits.between`): her stand, two slow walks, her walk, her jog, two faster, the run. On the
#: walk as built they step its cadence, 0.6-0.9 strides/s: the page's meter two cells down and
#: none up (the user, 2026-10-05). The law's walk is the page's own once it is a woman's
#: (docs/TODO.md item 28).
LEVELS = (-1.0, -0.6, -0.3, 0.0, 0.5, 0.7, 0.85, 1.0)

#: Z and X move her style SWAY_STEP of `style.SWAY`'s axis. On the one law M picks the next of
#: its manners (`gaits.MANNERS`) and Z and X move that one's amount MANNER_STEP, 0 to 1, each
#: keeping its own: a blend - leaning 0.5 and crouched 0.25, into a wind (the user, 2026-10-05).
SWAY_STEP, MANNER_STEP = 0.25, 0.25

#: The boards G and H glitch, in turn, and how long G's SOA lasts, s.
GLITCHED, SOA_S = ('left_knee', 'right_knee', 'left_hip', 'right_hip'), 0.5

#: What she trips on, by the digit that lays it (Ctrl letters: Ctrl+S paused the terminal,
#: 2026-09-28).
TRIPS = dict(zip('123456', ('hole', 'rug', 'sill', 'slip', 'lace', 'stairs')))

#: The gym's rigs she lands on, by the digit that lands her (`events.STANDING`).
RIGS = dict(zip('7890', ('brick', 'brick_on', 'board', 'rocker')))

#: Where R's recordings go, a CSV from a press to the next: every state the page hears from her,
#: 10-20 a second of her time (HEADER).
RECORDINGS = os.path.join(REPO, 'build', 'recordings')
HEADER = (['t', 'stage', 'yaw', 'speed', 'phase', 'left_load', 'right_load',
           'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'] + list(JOINTS)
          + ['%s_%s' % (seg[0], axis) for seg in SEGMENTS for axis in 'xyz']
          + ['set_' + j for j in JOINTS] + ['watts'])

#: What C steps her through: dressed, her shell, her mechanism (`coaxial.graphics.mechanism`),
#: her drives alone - the stick figure.
SKINS = ('dressed', 'shell', 'mechanism', 'actuators')


def _tripped(event):
    """1-6: `event` laid where her walk meets it - the walk as built's: on the law she is
    handed to that first."""
    def trip(state):
        if state['law']:
            _lawed(state)
        state['tripped'] = event
        state['body'].send(event=event)
    return trip


def _glitched(kind):
    def glitch(state):
        joint = GLITCHED[state['glitches'] % len(GLITCHED)]
        state['glitches'] += 1
        state['glitched'] = (joint, kind)
        state['body'].send(glitch=(joint, kind, SOA_S))
    return glitch


def _paced(step):
    """S and F: on the one law the row asked the next of LEVELS; on the walk as built, its
    cadence a step."""
    def pace(state):
        if state['law']:
            at = min(range(len(LEVELS)), key=lambda i: abs(LEVELS[i] - state['pace']))
            state['pace'] = LEVELS[max(0, min(len(LEVELS) - 1, at + (1 if step > 0.0 else -1)))]
            state['body'].send(pace=state['pace'])
            return
        state['cadence'] = max(CADENCE[0], min(CADENCE[1], state['cadence'] + step))
        state['body'].send(cadence=state['cadence'])
    return pace


def _lawed(state):
    """J: her going handed to the one law (`machine.pace`) at its walk, landed anew; again,
    back to the walk as built, her manners on the law let go first."""
    if state.get('manners'):
        state['manners'] = {}
        state['body'].send(manner=())
    state['law'], state['pace'] = not state['law'], 0.0
    state['body'].send(pace=state['pace'] if state['law'] else None)


def _next_knob(state):
    """K: the next style knob to trim."""
    state['knob'] = style.NAMES[(style.NAMES.index(state['knob']) + 1) % len(style.NAMES)]


def _trimmed(steps):
    """, and .: the knob picked a step down or up, the walk eased over to it (`machine.style`)."""
    return lambda state: state['body'].send(style=(state['knob'], steps))


def _picked(state):
    """M: the next of the one law's manners picked for Z and X; on the walk as built, her
    going handed to the law."""
    if not state['law']:
        return _lawed(state)
    names = tuple(gaits.MANNERS)
    state['manner'] = names[(names.index(state['manner']) + 1) % len(names)
                            if state.get('manner') in names else 0]


def _swayed(step):
    """Z and X: on the walk as built a step toward the catwalk or the swagger, every knob
    eased over to it; on the one law the manner picked a step less or more, the others as
    they are."""
    def sway(state):
        if state['law']:
            name = state.setdefault('manner', next(iter(gaits.MANNERS)))
            now = dict(state.get('manners', {}))
            now[name] = max(0.0, min(1.0, now.get(name, 0.0)
                                     + math.copysign(MANNER_STEP, step)))
            state['manners'] = {k: v for k, v in now.items() if v > 0.0}
            state['body'].send(manner=tuple(state['manners'].items()))
            return
        state['sway'] = max(-1.0, min(1.0, state['sway'] + step))
        state['body'].send(sway=state['sway'])
    return sway


def row(now, yaw):
    """A recording's row (HEADER) at the page's `yaw`."""
    placed = frames(now['angles'], now['where'], quat(*now['turn']))
    asked = now.get('set', {})
    return ([round(now['t'], 4), now['stage'], yaw, now['speed'], now['phase']]
            + list(now['loads']) + list(now['where']) + list(now['turn'])
            + [now['angles'].get(j, 0.0) for j in JOINTS]
            + [v for seg in SEGMENTS for v in placed[seg[0]][0]]
            + [asked.get(j, float('nan')) for j in JOINTS] + [now.get('watts', float('nan'))])


def _recorded(state):
    """R: recording from now; again: written to RECORDINGS, its name on the page."""
    if state['recording'] is None:
        state['recording'], state['recorded'] = [], None
        return
    rows, state['recording'] = state['recording'], None
    os.makedirs(RECORDINGS, exist_ok=True)
    path = os.path.join(RECORDINGS, time.strftime('humanoid_%Y%m%d_%H%M%S.csv'))
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)
        writer.writerows(rows)
    state['recorded'] = path


def _befell(event):
    """P, N, B: `event` (`events.befall`) toward her left and right in turn."""
    def befall(state):
        state['side'] = -state['side']
        state['body'].send(befall=(event, 0 if state['side'] > 0.0 else 1))
    return befall


def _rigged(key):
    """7-0: landed anew on the rig (RIGS), standing; the key again, the floor."""
    def rig(state):
        state['rig'] = None if state['rig'] == RIGS[key] else RIGS[key]
        if state['law']:
            # on the law she stands on it till F asks her on
            state['pace'] = -1.0
            state['body'].send(pace=-1.0, rig=state['rig'])
        else:
            state['body'].send(rig=state['rig'])
    return rig


def _skinned(state):
    """The next of SKINS."""
    state['skin'] = SKINS[(SKINS.index(state['skin']) + 1) % len(SKINS)]


def table(view, calling, shown):
    """{key: what it does to the state}: `view`'s keys and the page's own; L and T step through
    `calling` and `shown`."""
    return dict(
        list(view.items()) + [('[', _paced(-CADENCE_STEP)), (']', _paced(CADENCE_STEP))]
        + [(k, _paced(-CADENCE_STEP)) for k in 'sS'] + [(k, _paced(CADENCE_STEP)) for k in 'fF']
        + [('p', _befell('shove')), ('P', _befell('shove_on')), ('n', _befell('nudge')),
           ('N', _befell('nudge_on'))] + [(k, _befell('brick')) for k in 'bB']
        + [(k, _glitched('soa')) for k in 'gG'] + [(k, _glitched('hot')) for k in 'hH']
        + [(k, _tripped(event)) for k, event in TRIPS.items()]
        + [(k, _rigged(k)) for k in RIGS]
        + [(k, lambda state: state['body'].send(restart=True)) for k in 'aA']
        + [(k, lambda state: state.update(orbit=not state['orbit'])) for k in 'oO']
        + [(k, lambda state: state.update(called=calling[(calling.index(state['called']) + 1)
                                                         % len(calling)])) for k in 'lL']
        + [(k, _recorded) for k in 'rR']
        + [(k, _next_knob) for k in 'kK'] + [(',', _trimmed(-1)), ('.', _trimmed(1))]
        + [(k, _swayed(-SWAY_STEP)) for k in 'zZ'] + [(k, _swayed(SWAY_STEP)) for k in 'xX']
        + [(k, lambda state: state.update(shown=shown[(shown.index(state['shown']) + 1)
                                                      % len(shown)])) for k in 'tT']
        + [(k, _skinned) for k in 'cC']
        + [(k, lambda state: state.update(data=not state['data'])) for k in 'dD']
        + [(k, _lawed) for k in 'jJ'] + [(k, _picked) for k in 'mM'])


def act_on(keys, typed, state, wheel=0.0):
    """`typed` and the wheel on the view's `state`, by `keys` (`table`)."""
    if wheel:
        viewpoint.zoomed(state, max(0.5, 1.0 + wheel))
    state.setdefault('typed', []).extend(typed)
    for key in typed:
        if key in keys:
            keys[key](state)
