"""The HUMANOID page's viewpoint: a chase camera over her shoulder, turned by keys and the mouse.

    viewpoint.turned(state, dx, dy)           # a left drag's cells: round her and over her
    viewpoint.moved(state, dx, dy, w, h)      # a right drag's cells: what was under it stays
    viewpoint.lifted(state, 1)                # an arrow up: the view a step higher
    viewpoint.zoomed(state, 1.1)              # the zoom times k, within ZOOM
    viewpoint.chased(state, heading, going, now)   # eased behind her while she goes
    viewpoint.looks(state)                    # the heading it looks along, rad
    run_view(.., **viewpoint.mouse(state, size))   # the page's mouse
"""
import math
import time

from coaxial.graphics import engine, gynoid

#: Where it starts - straight behind her (`yaw` 180°), tipped down a little -, degrees a
#: key turns it, the orbit's degrees a second, the zoom's bounds: at 8 a knee's linkage fills it.
YAW, PITCH, TURN_DEG, ORBIT_DEG_S, ZOOM = 180.0, 8.0, 10.0, 12.0, (0.6, 8.0)
HOME = {'yaw': YAW, 'pitch': PITCH, 'zoom': 1.0, 'pan': (0.0, 0.0), 'shoulder': 1.0}

#: A left drag's degrees a cell across and a row down; the tip's bounds, deg; an arrow's step at
#: zoom 1, m.
DRAG_DEG, TIP_DEG, STEP_M = (2.0, 4.0), (-30.0, 85.0), 0.05

#: The chase: RECENTRE_S after the view was last turned, while she goes, its yaw eases behind
#: her heading, LAG_S its time constant; it looks past her SHOULDER_M aside, X the other side.
RECENTRE_S, LAG_S, SHOULDER_M = 1.5, 0.8, 0.22


def turned(state, dx, dy):
    state['yaw'] += dx * DRAG_DEG[0]
    state['pitch'] = max(TIP_DEG[0], min(TIP_DEG[1], state['pitch'] + dy * DRAG_DEG[1]))
    state['looked'] = time.monotonic()


def zoomed(state, k):
    state['zoom'] = max(ZOOM[0], min(ZOOM[1], state['zoom'] * k))


def lifted(state, steps):
    x, y = state['pan']
    state['pan'] = (x, y + steps * STEP_M / state['zoom'])


def moved(state, dx, dy, width, height):
    """The point looked at moved against a drag of (dx, dy) cells on a `width` x `height`
    drawing: a column DISTANCE / scale m there, a row twice it (`engine.project`'s aspect)."""
    cam = engine.camera(width, height, gynoid.REACH, distance=gynoid.DISTANCE, zoom=state['zoom'])
    m = gynoid.DISTANCE / cam['scale']
    x, y = state['pan']
    state['pan'] = (x - dx * m, y + 2.0 * dy * m)


def orbited(state, t):
    """Turned ORBIT_DEG_S a second of her time `t` while O orbits."""
    if state['orbit'] and state['last_t'] is not None:
        state['yaw'] += ORBIT_DEG_S * max(0.0, t - state['last_t'])
    state['last_t'] = t


def looks(state):
    """The heading the view looks along, rad: hers when it is straight behind her."""
    return math.radians(state['yaw'] - YAW)


def along(state):
    """The way the view looks over the floor, (x, y, z) a unit: a push from it."""
    return (-math.sin(math.radians(state['yaw'])), 0.0, -math.cos(math.radians(state['yaw'])))


def chased(state, heading, going, now):
    """Its yaw eased behind her `heading` (rad) while she is `going`, RECENTRE_S on from the
    view's last turn; `now` the wall clock, s."""
    then, state['chased'] = state.get('chased', now), now
    if not going or state.get('orbit') or now - state.get('looked', -RECENTRE_S) < RECENTRE_S:
        return
    off = (math.degrees(heading) + YAW - state['yaw'] + 180.0) % 360.0 - 180.0
    state['yaw'] += off * (1.0 - math.exp(-max(0.0, now - then) / LAG_S))


def shouldered(state):
    """X: the other shoulder."""
    state['shoulder'] = -state.get('shoulder', 1.0)


def panned(state):
    """Where the drawing is centred: the drags' pan, SHOULDER_M aside of her."""
    x, y = state['pan']
    return (x + SHOULDER_M * state.get('shoulder', 1.0), y)


def mouse(state, size):
    """`run_view`'s mouse: held throughout, F the page's pace; a left drag turns - across, the
    other way round her than the arrows -, a right drag moves, `size` () ->
    the drawing's (width, height); the wheel zooms through `on_input`."""
    return {'mouse': True, 'select': frozenset(), 'scroll_keys': False,
            'on_drag': lambda dx, dy: turned(state, -dx, dy),
            'on_pan': lambda dx, dy: moved(state, dx, dy, *size())}


def _turn(step):
    return lambda state: turned(state, step / DRAG_DEG[0], 0.0)


def _zoom(k):
    return lambda state: zoomed(state, k)


#: Its keys: the arrows round her and up and down, + and - nearer and farther, V home, X the
#: other shoulder.
KEYS = dict([('left', _turn(-TURN_DEG)), ('right', _turn(TURN_DEG)),
             ('up', lambda state: lifted(state, 1)), ('down', lambda state: lifted(state, -1))]
            + [(k, lambda state: state.update(HOME)) for k in 'vV']
            + [(k, shouldered) for k in 'xX']
            + [(k, _zoom(1.1)) for k in '+='] + [(k, _zoom(1.0 / 1.1)) for k in '-_'])
