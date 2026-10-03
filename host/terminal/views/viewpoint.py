"""The HUMANOID page's viewpoint: turned, tipped, moved and zoomed by keys and the mouse.

    viewpoint.turned(state, dx, dy)           # a left drag's cells: round her and over her
    viewpoint.moved(state, dx, dy, w, h)      # a right drag's cells: what was under it stays
    viewpoint.lifted(state, 1)                # an arrow up: the view a step higher
    viewpoint.zoomed(state, 1.1)              # the zoom times k, within ZOOM
    run_view(.., **viewpoint.mouse(state, size))   # the page's mouse
"""
from coaxial.graphics import engine, gynoid

#: Where it starts - three quarters round so a stride shows, tipped down a little -, degrees a
#: key turns it, the orbit's degrees a second, the zoom's bounds: at 8 a knee's linkage fills it.
YAW, PITCH, TURN_DEG, ORBIT_DEG_S, ZOOM = 60.0, 8.0, 10.0, 12.0, (0.6, 8.0)
HOME = {'yaw': YAW, 'pitch': PITCH, 'zoom': 1.0, 'pan': (0.0, 0.0)}

#: A left drag's degrees a cell across and a row down; the tip's bounds, deg; an arrow's step at
#: zoom 1, m.
DRAG_DEG, TIP_DEG, STEP_M = (2.0, 4.0), (-30.0, 85.0), 0.05


def turned(state, dx, dy):
    state['yaw'] += dx * DRAG_DEG[0]
    state['pitch'] = max(TIP_DEG[0], min(TIP_DEG[1], state['pitch'] + dy * DRAG_DEG[1]))


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


def mouse(state, size):
    """`run_view`'s mouse: held throughout, F the page's pace; a left drag turns - across, the
    other way round her than the arrows (the user, 2026-10-03) -, a right drag moves, `size` () ->
    the drawing's (width, height); the wheel zooms through `on_input`."""
    return {'mouse': True, 'select': frozenset(), 'scroll_keys': False,
            'on_drag': lambda dx, dy: turned(state, -dx, dy),
            'on_pan': lambda dx, dy: moved(state, dx, dy, *size())}


def _turn(step):
    return lambda state: turned(state, step / DRAG_DEG[0], 0.0)


def _zoom(k):
    return lambda state: zoomed(state, k)


#: Its keys: the arrows round her and up and down, + and - nearer and farther, V home.
KEYS = dict([('left', _turn(-TURN_DEG)), ('right', _turn(TURN_DEG)),
             ('up', lambda state: lifted(state, 1)), ('down', lambda state: lifted(state, -1))]
            + [(k, lambda state: state.update(HOME)) for k in 'vV']
            + [(k, _zoom(1.1)) for k in '+='] + [(k, _zoom(1.0 / 1.1)) for k in '-_'])
