"""The front page's stand: the board, her walking, the quad hovering, torn one into the next.

    view = stand.fresh(now)                       # the turntable's state
    stand.idle(view, now, dt)                     # a frame of its own motion
    stand.grab(view, keys, moved, now)            # the hand on it
    art = stand.turntable(view, width, height)    # what it shows now
    threading.Thread(target=stand.warm).start()   # its solids, off the frame loop
"""
import math
import time

from terminal.ui import glitch

#: Degrees of yaw per second for the idle tumble, with slower sways
#: about the other two axes riding on top - all three turn, none of them
#: fast enough to read as spinning.
TURN_DPS = 30.0


#: The turntable's zoom: it opens at SWELL_FROM, fills to SWELL_LO over
#: SWELL_IN seconds, then breathes between SWELL_LO and SWELL_HI while
#: it turns. Grabbed - a drag turns it, the wheel zooms it - it holds
#: still; released, the tumble and the breath resume from where it was
#: left: the pose is state, and the breath re-seats its phase on the
#: zoom it finds.
SWELL_FROM, SWELL_IN = 0.25, 2.5
SWELL_LO, SWELL_HI = 0.75, 1.20
SWELL_PERIOD = 11.0
#: render()'s zoom 1.0 fits the bounding sphere at any attitude, which in
#: the box is 56% of its width - measured, and 2.0 is the first zoom
#: that reaches every edge. The envelope is in shares of the box, so
#: it rides on this base: 1.2 lands at 2.16, past the box, so the
#: board clips into its frame; 0.75 at 1.35, four fifths of it.
SWELL_BASE = 1.8
#: Seconds after the last touch before the idle motion takes over again.
HOLD = 0.6


def seat(zoom):
    """The breath's phase whose zoom is nearest `zoom`, on the rising
    side, so it climbs first from wherever the hand let go."""
    mid = (SWELL_LO + SWELL_HI) / 2.0
    half = (SWELL_HI - SWELL_LO) / 2.0
    return math.asin(max(-1.0, min(1.0, (zoom - mid) / half)))


def _swell(view, now):
    """The fill, eased out: fast at first, settling at SWELL_LO."""
    t = (now - view['opened']) / SWELL_IN
    view['zoom'] = SWELL_FROM + (SWELL_LO - SWELL_FROM) * (
        1.0 - (1.0 - t) ** 3)


def _breathe(view, dt):
    """The breath: the zoom rides a sine between SWELL_LO and SWELL_HI,
    seated where the swell or the hand left it."""
    if view['phase'] is None:
        view['phase'] = seat(view['zoom'])
    view['phase'] += dt * 2.0 * math.pi / SWELL_PERIOD
    mid = (SWELL_LO + SWELL_HI) / 2.0
    half = (SWELL_HI - SWELL_LO) / 2.0
    want = mid + half * math.sin(view['phase'])
    # Toward the envelope rather than onto it: a zoom the wheel left outside
    # the band glides back instead of snapping.
    view['zoom'] += (want - view['zoom']) * min(1.0, 4.0 * dt)


def fresh(now):
    """The turntable's state, opened `now`: its pose and zoom persist across a grab, so the
    idle motion carries on from wherever the hand left it."""
    return {'pose': (0.0, 0.0, 0.0, 1.0), 'zoom': SWELL_FROM, 'phase': None, 'opened': now,
            'touched': -1e9, 'spun': 0.0, 'carry': (0.0, 0.0)}


def idle(view, now, dt):
    """One frame of the turntable on its own: tumble, and breathe."""
    from terminal.views.show_render import turn

    if now - view['touched'] < HOLD:
        return
    if view['phase'] is None and now - view['opened'] < SWELL_IN:
        _swell(view, now)
    else:
        _breathe(view, dt)
    wobble = view['spun'] = view['spun'] + dt
    turn(view, (0.35 * math.sin(wobble * 0.5),
                0.25 * math.sin(wobble * 0.83 + 1.3), 1.0),
         TURN_DPS * dt)


def grab(view, keys, moved, now):
    """The hand on the turntable: wheel zoom, and a left-drag turn
    (show_render's `carry`) that pauses the idle motion while it lasts."""
    from terminal.views.show_render import carry

    if moved:
        view['zoom'] = max(0.25, min(4.0, view['zoom'] * (1.0 + moved)))
        view['touched'], view['phase'] = now, None
    dx, dy = keys.dragged()
    if dx or dy:
        view['touched'], view['phase'] = now, None
    carry(view, dx, dy)


#: The stand shows the board BOARD_S, then her walking HER_S, in place as her plan has it
#: (`machine.gait`), then the quad QUAD_S, in a hover and once over its nose, each torn into
#: the next over TORN_S (`terminal.ui.glitch`) - where a card lights them: on the CPU a frame
#: of her took 280 ms, on the card 6 (2026-09-28).
BOARD_S, HER_S, QUAD_S, TORN_S = 14.0, 9.0, 9.0, 0.5

#: The quad on the stand: its lean, degrees, round once in LEAN_S; its roll, begun ROLL_AT s
#: into its act and made in ROLL_S.
LEAN_DEG, LEAN_S, ROLL_AT, ROLL_S = 10.0, 5.0, 3.5, 2.0
#: Looked down on this much, degrees, this reach of it framed, m; its cans and boards as warm
#: as a flight has them, C.
LOOK_DEG, REACH_M, CAN_C, BOARD_C = 32.0, 0.62, 55.0, 45.0


def turntable(view, width=52, height=18):
    """The board on the stand, at the pose and zoom the view holds - lit as the render demo
    lights it: camera straight down the axis, no horizon - and in turn her, walking, and the
    quad, hovering.
    """
    if not _STAGE['ready']:
        return ''
    elapsed = time.monotonic() - view['opened']
    acts, half = ((BOARD_S, _draw), (HER_S, _her), (QUAD_S, _quad)), TORN_S / 2.0
    if _STAGE.get('lit') is None or elapsed < half:
        return _draw(view, width, height)
    into, act = elapsed % sum(seconds for seconds, _draws in acts), 0
    while into >= acts[act][0]:
        into, act = into - acts[act][0], act + 1
    seconds, draws = acts[act]
    if half <= into < seconds - half:
        return draws(view, width, height)
    # torn in from the act before, or out to the one after
    old, new, k = ((acts[act - 1][1], draws, 0.5 + into / TORN_S) if into < half else
                   (draws, acts[(act + 1) % len(acts)][1], (into - seconds + half) / TORN_S))
    return '\n'.join(glitch.torn(old(view, width, height).split('\n'),
                                 new(view, width, height).split('\n'), k, int(elapsed * 30)))


def _her(view, width, height):
    """Her walking on the stand, in place, turning as the board turns."""
    from coaxial.graphics import gynoid
    from machine import gait
    from machine.figure import rz
    t = time.monotonic() - view['opened']
    lateral, roll, rise, _level = gait.sway(t)
    return '\n'.join(gynoid.render(gait.walk(t, glance=False), width, height,
                                    yaw=(view['spun'] * TURN_DPS) % 360.0, lit=_STAGE['lit'],
                                    root=((lateral, rise, 0.0), rz(math.radians(roll)))))


def _quad(view, width, height):
    """The quad on the stand, alone: in a hover, leant round as an orbit leans it, and once over
    its nose; seen turning as the board turns."""
    from coaxial.graphics import quadcopter
    from machine import aerobatics, quad
    from machine.figure import mul, rx, rz
    t = time.monotonic() - view['opened']
    into = t % (BOARD_S + HER_S + QUAD_S) - BOARD_S - HER_S
    lean, round_ = math.radians(LEAN_DEG), math.tau * t / LEAN_S
    roll = math.tau * aerobatics.ease((into - ROLL_AT) / ROLL_S)
    turn = mul(mul(rz(lean * math.cos(round_)), rx(lean * math.sin(round_))), rz(roll))
    hover = quad.speed_for(quad.MASS_KG * quad.GRAVITY / 4.0)
    return '\n'.join(quadcopter.render(
        {'turn': turn, 'at': (0.0, 0.0, 0.0)}, [(hover * t, hover, CAN_C, BOARD_C)] * 4, width,
        height, yaw=(view['spun'] * TURN_DPS) % 360.0, pitch=LOOK_DEG, reach=REACH_M,
        lit=_STAGE['lit'], world=False))


def _draw(view, width, height):
    from coaxial.graphics import wireframe

    return wireframe.render(view['pose'], width, height, zoom=view['zoom'],
                            horizon=False, tip=0.0, lift=0.5,
                            least=wireframe.CREW_LEAST, crew=_STAGE.get('crew'))


#: The turntable's solids build off the frame loop: the page is up in the
#: import's 0.3 s and the board arrives when the parse and two decimations
#: are done; drawn inline, the first frame waited 2.0 s, the parse twice.
_STAGE: dict = {'ready': False}


def warm():
    """The stand's solids built, and her and the quad once on the card where one answers."""
    try:
        _draw({'pose': (0.0, 0.0, 0.0, 1.0), 'zoom': SWELL_FROM}, 8, 4)
        # On the card where one answers: 64 ms a frame at 52x18 on the CPU (2026-09-25).
        from coaxial.graphics import gpu, shading
        _STAGE['crew'] = gpu.card_crew(art=shading._face())
        card = gpu.adapter()
        if card is not None:
            _STAGE['lit'] = gpu.LitRaster(found=card)
            _her({'opened': time.monotonic(), 'spun': 0.0}, 8, 4)
            _quad({'opened': time.monotonic(), 'spun': 0.0}, 8, 4)
    finally:
        _STAGE['ready'] = True
