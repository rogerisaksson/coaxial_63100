"""The robot's legs released at will from what bears it: the gate, the cascade, the leg held till free.

intentional_limb_emancipation.lisp on the robot's loop, a leg a lane (its soles share one manifold,
`ortho.PLANTAR`).

    emancipation.step(g, s, out, told, dt)   # a pass: g['emancipated'] each leg's release, its cascade laid
    eta, parts = emancipation.gate(xi, pts, room, soles, vcom, shares, loads, weight)  # the physics' leave, each leg
    held, rest = emancipation.held(g, s, bears, unfelt)  # the robot's capture point's barrier's support and its share

A leg is released as far as the robot's text asks it (`intent`: the robot's soles told nothing while something else
is, or the leg lifted) and its physics allows it (`gate`: the robot's centre within the text's Froude number,
what its text tells bears it besides the leg its least sum, the robot's capture point inside that by
the text's least margin), slewed over the text's 160 ms, back over its relock. The cascade (`unloading-cascade`) in
order: the robot's estimator lets the sole go as an anchor (`motion.seen`), its step lets the leg go and sheds
its foot's push to nothing, then the leg moves once its sole bears no more than its own leg. Asked, the robot's trunk
leans its capture point in over the rest (`counterweighted`), the leg's cantilever on it, and the robot's
barrier holds it there; asked while the robot goes, the gate holds the leg in its chain.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np
# np.clip's own ufunc, its Python layers left out: the same bits, 3.7 -> 1.3 us a call, 14 a law pass here
from numpy._core.umath import clip as CLIP  # pyright: ignore[reportAttributeAccessIssue]

from machine.senses import borne, field, observer, edge
from machine.texts import kernel, ortho
from machine.law import composer, latent, modal
from machine.moves import leap
from machine.moves.cascade import (_U, asked, HANG_RATE, LEG_JOINTS, LEGGED, leave, MARGIN_NM, OPEN, opening,
                                   PLANTAR, reached, released, SHED)
from machine.stack.wbc import turn_error

#: The robot's seat's manifolds facing up (`observer.UPRIGHT`): its rows, the segment their points lie on, the
#: kernel's CoP box for each (`anatomical-manifold-registry`'s cop-bounds, m: along its way, across).
_REGISTRY = kernel.read()['anatomical-manifold-registry']
SEAT_ROWS = [ortho.MANIFOLDS.index('ischial')]
SEAT_SEG = {r: next(seg for _sh, seg, _p, _n in observer.POINTS if _sh[0][0] == 'ischial-bilateral') for r in SEAT_ROWS}
SEAT_BOX = {r: tuple(1e-3 * np.array(_REGISTRY['ischial-bilateral']['cop-bounds-mm'][k]) for k in ('x', 'y')) for r in SEAT_ROWS}
#: Each of the robot's observer's points' room along the floor, m: its segment's capsule's radius round it, as the
#: robot's skin's contacts have it (`borne._room`). Points alone, the robot's seat's ischia read its capture
#: point 1 cm outside a seat it lay 35 mm inside.
ROOMS = np.array([float(borne._room(edge.SKIN.index(seg), np.asarray(p, float))[0]) if seg in edge.SKIN else 0.0
                  for _sh, seg, p, _n in observer.POINTS])


def margin(point, pts, room, mask):
    """(m (k,), its edge's outward normal (k, 2)): `point` (2,) inside the hull of the discs `pts` (n, 2) of radii
    `room` (n,) that `mask` (k, n) keeps in each lane - the least of its support's reach past it over DIRECTIONS
    (`_U`): inside, its nearest edge's distance; outside, less than 0 by its distance to the hull; with none kept,
    -inf."""
    reach = reached(point, pts, room, mask)
    return reach.min(axis=1), _U[reach.argmin(axis=1)]


def gate(xi, pts, room, soles, vcom, shares, loads, weight, left=None):
    """(eta (2,), (froude's, anchors' (2,), margin's (2,), margin N m (2,), its edge's normal (2, 2))): the physics'
    leave to release each leg - the robot's centre's Froude number under the text's lockout (`vcom` world, y up),
    what its text tells bears it besides the leg (`shares`, `observer.said`; its soles' share
    split by their `loads`) at the text's least sum or over - weighed as felt, its released heels resting on
    the floor shut it -, the robot's capture point `xi` (2,) inside what bears it besides it by the text's
    least margin of the robot's `weight`: the points `pts` (n, 2) of radii `room`, each leg's own sole's points
    (`soles` (2, n), 1 its own) not among them. The arithmetic `left`, `leave`'s C (`machine.law.motion.laid`), else
    numpy's; the edge its least reach's, picked here."""
    arithmetic = leave if left is None else left
    eta, by_froude, by_anchors, by_margin, nm, reach = arithmetic(xi, pts, room, soles, vcom, shares, loads, weight)
    return eta, (by_froude, by_anchors, by_margin, nm, _U[reach.argmin(axis=1)])


def told_on(obs, told, floor, on=None):
    """Which of the robot's observer's points (`observer.POINTS`) are of what its text tells bears the
    robot besides its soles (`told`, `latent.MANIFOLDED`, its rows' order; its toes are the
    robot's soles') within the prior's reach of the `floor` (`observer.REACH_M`); `on` its arithmetic's C
    (`machine.law.motion.laid`), else numpy's (`on_floor`)."""
    return (on_floor if on is None else on)(obs['soled'], obs['row'], obs['latch_of'], obs['points'], told, floor)


def on_floor(soled, row, latch_of, points, told, floor):
    """`told_on`'s arithmetic of the robot's observer's arrays alone, numpy's, so compiled (g['told_on']): 14 us a
    call in numpy, twice a law pass."""
    return np.logical_and(np.logical_and(np.logical_and(np.logical_not(soled), row != PLANTAR),
                                         latch_of[:, 1:] @ told[1:] > 0.0), points[:, 1] - floor <= observer.REACH_M)


def supports(s, obs, told, floor, legs, loads, on=None):
    """(the robot's floor points (n, 2), their radii (n,), each leg's own (2, n)): what bears it besides the
    robot's soles as its text tells it (`told`, `latent.MANIFOLDED`) - the robot's observer's points of each
    manifold told (`momentum.laid`) within its prior's reach of the `floor` - and as the robot's skin feels it - each
    contact but a let-go foot (`borne.LET`), its capsule's room round it -, and each sole's corners where it bears the
    robot (`leap.TOUCH_N`), as far as its leg is not asked released (`legs`, 2). The robot's skin's alone, the robot's
    seat unloaded as it leant back left it, and the robot's counterweight leant its further onto the
    robot's hands and over."""
    from machine.senses.borne import LET
    held = [c for c in s.get('contacts', ()) if c['patch'] < LET]
    top = max(floor, float(s.get('prop', (0.0, 0.0, (0.0, 0.0, 0.0)))[2][1]), float(s['com'][1]) - 0.35)
    said = told_on(obs, told, top, on)
    corners = [f['pts'] for f in s['feet']]
    pts = np.concatenate([np.array([c['p'][[0, 2]] for c in held]).reshape(-1, 2), obs['points'][said][:, [0, 2]]]
                         + [p[:, [0, 2]] for p in corners])
    room = np.concatenate([np.array([c['room'] for c in held]), ROOMS[said], np.zeros(sum(len(p) for p in corners))])
    stood = (loads >= leap.TOUCH_N) * (legs < 1.0)
    soles = np.zeros((2, len(pts)))
    at = len(pts) - sum(len(p) for p in corners)
    for k, p in enumerate(corners):
        soles[k, at:at + len(p)] = 1.0
        soles[1 - k, at:at + len(p)] = 1.0 - stood[k]
        at += len(p)
    return pts, room, soles


def step(g, s, out, told, dt):
    """Each leg's release `dt` s on (`g['emancipated']`, 0 to 1), slewed toward the robot's text's intent times the
    robot's physics' leave; its cascade laid: `g['a'][LET_GO]` the robot's step's let-go legs (the shed window's first
    edge; and as its text tells its soles nothing while something else bears it,
    `borne.latched`, or while what its text tells bears its body lies on the floor, `g['sat']`, held
    the robot's comparator's dwell past it: on the floor the robot's legs are anchors, never standing legs - stood
    there, its standing law launched it off its seat at 142-225 N), `g['unanchored']` the
    robot's estimator's share of each sole as an anchor, `g['let_cap']` each let-go foot's push, N, `g['freed']` how
    far each leg moves - the parts in `g['emancipation']`; its legs' modes in `out` held (`g['held_ref']`) as
    far as a leg asked released is not yet free; its centre of mass's ask leant to its counterweight
    after (`counterweighted`). The arithmetic in `asked` and `released`, the support in `supports` and `gate` between
    them; each its C where `machine.law.motion.laid` compiled it."""
    weight, floor, loads = float(g['fr'][modal.WEIGHT]), float(g['fr'][modal.FLOOR]), np.asarray(g['loads'], float)
    was = g.get('emancipated', np.zeros(2))
    g['sat'] = max(g['seat_on'], g['sat'] * g['seat_near'])
    c, v = np.asarray(s['com'], float), np.asarray(s['vcom'], float)
    kept, iota, om, xi = g.get('em_asked', asked)(was, g['sat'], told, out, c, v, floor)
    eta, parts, support = gated(g, s, told, floor, iota, loads, xi, v, weight)
    eta, gamma, let_go, unanchored, let_cap, freed, held, rate, acc = g.get('em_released', released)(
        was, iota, eta, kept, g['sat'], told, g['a'], g['y'], c, loads, g['fr'], g['held_ref'], out, dt)
    g['emancipated'], g['unanchored'], g['let_cap'], g['freed'] = gamma, unanchored, let_cap, freed
    g['a'][modal.LET_GO] = let_go
    g['held_ref'][...] = held
    out[latent.REF][LEGGED], out[latent.RATE][LEGGED], out[latent.ACC][LEGGED] = held, rate, acc
    g['emancipation'] = {'iota': iota, 'eta': eta, 'parts': parts, 'xi': xi, 'om': om, 'support': support}
    return gamma


# no leg asked released, the robot's cascade asks each iota times its leave: 0 times any leave is 0, and none else
# reads the leave, its parts or the robot's support - its counterweight's rest is this one's (`composer`)
@composer.capability(lambda _g, _s, _told, _floor, iota, *_a: bool(iota.any()), lambda *_a: (np.zeros(2), None, None))
def gated(g, s, told, floor, iota, loads, xi, v, weight):
    """(eta (2,), the gate's parts, the robot's support (points, radii, each leg's own)): the physics' leave to
    release each leg (`gate`) over what bears it besides it (`supports`), at work while the robot's text asks
    a leg released (`iota`, a leg kept released among them). Asked none - the robot's stand, its walk, the
    robot's seat with both soles told -, they were told each law pass for a leave its cascade multiplied by
    nothing: 72 us of it."""
    pts, room, soles = g.get('em_supports', supports)(s, g['observer'], told, floor, iota, loads, g.get('told_on'))
    eta, parts = gate(xi, pts, room, soles, v, observer.said(told), loads, weight, g.get('leave'))
    return eta, parts, (pts, room, soles)


def settled(acc, g, s):
    """`acc`, the robot's centre of mass's ask (world, y up), sinking at the text's free-air seek
    (`field.SEEK_M_S`, at its rate) as far as what the robot's text tells bears its body besides the
    robot's soles and hands lies within its observer's reach of the floor and bears less of the robot
    than told (`s['prop']`, felt): onto it, that its load goes where the robot's legs' and hands' is shed to;
    in free air only, till it touches (`g['sat']`). Held where the robot was, it sat on its hands and feet a
    millimetre over a seat told 0.92 of it and never felt; sinking on with its seat on the floor, its step
    lowered its centre of mass the one way left, its trunk back -10 -> -70 deg. The arithmetic `settling`'s,
    compiled where the robot's law laid it."""
    obs = g['observer']
    return g.get('settling', settling)(acc, obs['soled'], obs['row'], obs['latch_of'], obs['points'], g['told'],
                                       float(g['fr'][modal.FLOOR]), g['sat'], float(s['prop'][0]), s['vcom'])


def settling(acc, soled, row, latch_of, points, told, floor, sat, prop0, vcom):
    """`settled`'s arithmetic of the robot's law's arrays alone - its observer's, what its text tells bears it, its
    centre of mass's ask `acc` and speed `vcom`, the share of it what bears its felt `prop0` -, numpy's, so compiled
    (g['settling']): 26 us a law pass in numpy."""
    near = np.logical_and(on_floor(soled, row, latch_of, points, told, floor), row != ortho.PALMAR).any()
    said = observer.said(told)
    unfelt = np.clip(said.sum() - said[PLANTAR] - said[ortho.PALMAR] - prop0, 0.0, 1.0)
    rise = near * (1.0 - sat) * unfelt * field.RATE * (field.SEEK_M_S - vcom[1])
    return np.stack([acc[0], acc[1] + rise, acc[2]])


# no leg asked released, its lean is 0 iota times its cantilever's: at rest the robot's ask as it was
# (`composer`)
@composer.capability(lambda _acc, g: bool(g['emancipation']['iota'].any()), lambda acc, _g: np.array(acc, float), g=1)
def counterweighted(acc, g):
    """`acc`, the robot's centre of mass's ask (world, y up), with its capture point asked in of what bears
    the robot besides the leg its text most asks released, as far as it asks, to where the robot's gate is
    open wholly (twice the text's margin, of its weight): the leg's cantilever on the rest of the robot
    (`feedforward-cantilever-reaction-wrench`), in by its pendulum's omega^2, from no further than its edge;
    none with nothing else bearing the robot. Asked of the robot's pitch, its step's rows on the robot's
    centre of mass held it and the robot's trunk never leant."""
    e = g['emancipation']
    k = int(np.argmax(e['iota'] * (1.0 - 0.5 * g['freed'])))
    weight = float(g['fr'][modal.WEIGHT])
    m, edge = e['parts'][3][k] / weight, e['parts'][4][k]
    full = 2.0 * MARGIN_NM / weight
    short = CLIP(full - m, 0.0, full) * float(np.isfinite(m))
    out = np.array(acc, float)
    out[[0, 2]] -= e['iota'][k] * e['om'] ** 2 * short * edge
    return out


def held(g, s, bears, unfelt):
    """(the robot's support's points (k, 3), the share its capture point's barrier holds it in it,
    `barrier.barred`): each sole that bears the robot (`bears`) its text does not ask released, and what else
    bears it as its observer feels it (`observer.bearing`) - the robot's soles all where nothing else
    has three points -; held as far as what its rows lean on is unfelt (`unfelt`) or a leg is asked released,
    while it does not stride."""
    iota, others = g['emancipation']['iota'], observer.bearing(g['observer'])
    stood = bears & (iota < 1.0)
    kept = stood | ~stood.any() & (len(others) < 3)
    pts = np.concatenate([s['feet'][k]['pts'] for k in np.flatnonzero(kept)] + [others])
    return pts, (1.0 - float(g['a'][modal.STRIDES])) * max(float(bears.any()) * unfelt, float(iota.max()))


def opened(g, s, ask, dt):
    """`ask`'s posture of each let-go leg its open chain's (`open-chain`): its reference on its joints as they
    are while it bears; let go, its knee drawn to the robot's text's over the cascade's slew, its hip and
    ankle where that knee puts its sole on the floor from its hip's height over its ankle as it was let go
    (`modal.seat`'s chain, that height held) and the robot's pelvis on its pitch's aim
    (`posed.posed`) while the leg's sole is down - its weight, out ahead of the robot, holds it there; on
    the robot's pelvis as it was the robot's legs let it roll back 48 -> 61 deg and the robot's trunk -5 ->
    -31 deg, the floor sit fell at 7.1 s - and on its pelvis's pitch as sensed as the leg is lifted -
    on the aim its lifted legs' weight hauled its pelvis to the ask, 37 deg forward at the
    bed's handover where the scoot had rolled it back, and the robot's trunk with it off the robot's fists -,
    its yaw, roll and ankle's roll where they were; `ask['open']` the text's stiffness and damping. On the
    robot's posture's own - its chair's legs on its pelvis's height as sensed, the robot's
    crouch's - a let-go leg pressed its foot down as the robot rose and lifted its 0.16 -> 0.26 m;
    yielding to all its sole bore, its hip wound to -200 deg; drawn to the robot's text's knee, past where the
    robot's sit had it, the robot's released knees folded 146 -> 162 deg and its heels dragged the
    robot."""
    high, ref, posture, opening_, let_hold, hang = g.get('em_opening', opening)(
        g['a'][modal.LET_GO], s['q'], g['fr'], g['a'], g['y'], g['out'], g['high'], g['emancipation']['iota'],
        g['freed'], g['told'], g['sat'], ask['posture'], g['bend'], g['q_open'], g['posture'], g['q_lo'], g['q_hi'],
        g['emancipated'], dt)
    g['high'] = high
    g['q_open'][...] = ref
    ask['posture'][...] = posture
    ask['open'] = (OPEN['kp'], OPEN['kd'], opening_)
    ask['let_hold'] = let_hold
    ask['hang'], ask['hang_rate'] = hang, HANG_RATE
    hipped(g, s, ask)


def hipped(g, s, ask):
    """Each let-go leg turned at its hip's yaw toward the heading the robot's pelvis is asked (`ask['turns']`, the
    error about its pelvis's up, within the joint's range): swung up, its legs turn with the robot's
    trunk's turn to lie along the bed, the pelvis between them following; its right hip's axis is the robot's
    left's mirrored."""
    turns, let = ask.get('turns', ()), g['a'][modal.LET_GO]
    if not len(turns) or not (let > 0.5).any():
        return
    have = np.asarray(s['turns'][0]['quat'], float)
    w0, x0, y0, z0 = have
    up = np.array([2.0 * (x0 * y0 - w0 * z0), 1.0 - 2.0 * (x0 * x0 + z0 * z0), 2.0 * (y0 * z0 + w0 * x0)])
    yaw = float(turn_error(np.asarray(turns[0], float), have) @ up)
    for k, sign in ((0, 1.0), (1, -1.0)):
        if let[k] > 0.5:
            j = LEG_JOINTS[k][0]
            ask['posture'][j] = min(max(sign * yaw, float(g['q_lo'][j])), float(g['q_hi'][j]))


def seated(g, s):
    """`s['contacts']` with the seat the robot's text tells bears it as the kernel's cone for it
    (`anatomical-manifold-registry`, the manifold `pan-wrench-homotopy` pans the robot onto): while it lies on
    the floor - the lowest of its segment's capsules within the robot's sole's touch (`leap.TOUCH_M`), or felt
    within the prior's reach of it (felt on a chair 0.45 m up, the robot was given the floor's cone there and
    fell at 2.4 s), held while it stays within the prior's reach of the floor (`observer.REACH_M`; `g['sat']`,
    the pan's sync trigger) - its CoP box's four corners on the floor under the robot's observer's points of
    it, the robot's pelvis's heading its x, each a contact bearing what the robot's step put on it the pass
    before (`g['seat_f']`) or a quarter of what the robot's skin feels of it (its points taken out), its push
    grown from there (`rounds.GROW_N`): on the robot's skin's alone, a seat a millimetre under it could
    take 120 N; held by its observer's comparator, it dropped out each time the robot's hands took the
    robot and it rose off it. Two points across the robot's ischia held no pitch: it rolled back over
    them; felt alone, a seat a millimetre under it bore 0 N."""
    b, obs, told, floor = g['body'], g['observer'], g['told'], float(g['fr'][modal.FLOOR])
    rows = [r for r in SEAT_ROWS if told[r] > 0.0]
    on, near = 0.0, 0.0
    for r in rows:
        seg = SEAT_SEG[r]
        at = b.m.body(seg).id
        R, x = b.d.xmat[at].reshape(3, 3), b.d.xpos[at]
        ends = [x + R @ e for _r, e0, e1 in borne.CAPSULES[edge.SKIN.index(seg)] for e in (e0, e1)]
        radii = [rr for rr, _e0, _e1 in borne.CAPSULES[edge.SKIN.index(seg)] for _ in (0, 1)]
        low = min(e[1] - rr for e, rr in zip(ends, radii)) - floor
        felt = float(any(c.get('body') == at for c in s['contacts']))
        on = max(on, float(low <= leap.TOUCH_M), felt * float(low <= observer.REACH_M)) * float(
            g.get('emancipated', np.zeros(2)).max() <= SHED[0])
        near = max(near, float(low <= observer.REACH_M))
    g['seat_on'], g['seat_near'] = on, near
    if g['sat'] <= 0.5:
        return s
    jp, jr = b.jacs
    ahead = b.d.xmat[b.pelvis].reshape(3, 3)[:, 2] * np.array([1.0, 0.0, 1.0])
    ahead /= np.linalg.norm(ahead)
    across = np.cross(np.array([0.0, 1.0, 0.0]), ahead)
    for r in rows:
        seg = SEAT_SEG[r]
        at = b.m.body(seg).id
        felt = sum(float(c['f'][1]) for c in s['contacts'] if c.get('body') == at)
        give = max([float(c.get('give', 0.0)) for c in s['contacts'] if c.get('body') == at] + [0.0])
        s['contacts'] = [c for c in s['contacts'] if c.get('body') != at]
        mid = obs['points'][obs['row'] == r].mean(axis=0)
        mid[1] = floor
        for corner, (dx, dz) in enumerate((dx, dz) for dx in SEAT_BOX[r][0] for dz in SEAT_BOX[r][1]):
                p = mid + dx * ahead + dz * across
                bears = max(0.25 * felt, float(g['seat_f'][corner]))
                b.mj.mj_jac(b.m, b.d, jp, jr, p, at)
                J = jp[:, :b.n].copy()
                b.mj.mj_jacDot(b.m, b.d, jp, jr, p, at)
                s['contacts'].append({'p': p, 'f': np.array([0.0, bears, 0.0]), 'n': np.array([0.0, 1.0, 0.0]), 'J': J,
                                      'dJv': jp[:, :b.n] @ s['v'],
                                      'carpal': 0.0, 'side': 0, 'patch': 2, 'room': 0.0, 'body': at, 'corner': corner,
                                      'give': give})
    return s
