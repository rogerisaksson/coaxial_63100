"""Her legs released on purpose from what bears her: the gate, the cascade, the leg held till free.

intentional_limb_emancipation.lisp on her loop, a leg a lane (her soles share one manifold, `ortho.PLANTAR`).

    emancipation.step(g, s, out, told, dt)   # a pass: g['emancipated'] each leg's release, its cascade laid
    eta, parts = emancipation.gate(xi, pts, room, soles, vcom, shares, loads, weight)  # the physics' leave, each leg
    held, rest = emancipation.held(g, s, bears, unfelt)  # her capture point's barrier's support and its share

A leg is released as far as her text asks it (`intent`: her soles told nothing while something else is, or the leg
lifted) and her physics allows it (`gate`: her centre within the text's Froude number, what her text tells bears her
besides the leg its least sum, her capture point inside that by the text's least margin), slewed over the text's 160
ms, back over its relock. Its cascade (`unloading-cascade`) in order: her estimator lets the sole go as an anchor
(`motion.seen`), her step lets the leg go and sheds its foot's push to nothing, then the leg moves once its sole bears
no more than its own leg. Asked, her trunk leans her capture point in over the rest (`counterweighted`), the leg's
cantilever on it, and her barrier holds it there; asked while she goes, the gate holds the leg in her chain.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np
# np.clip's own ufunc, its Python layers left out: the same bits, 3.7 -> 1.3 us a call, 14 a law pass here
from numpy._core.umath import clip as CLIP

from machine.senses import borne, field, observer, edge
from machine.texts import kernel, ortho
from machine.law import latent, modal
from machine.moves import leap
from machine.moves.cascade import _U, asked, LEGGED, leave, MARGIN_NM, nothing, OPEN, opening, PLANTAR, reached, released, SHED

#: Her seat's manifolds facing up (`observer.UPRIGHT`): her rows, the segment their points lie on, the kernel's CoP box
#: for each (`anatomical-manifold-registry`'s cop-bounds, m: along her way, across).
_REGISTRY = kernel.read()['anatomical-manifold-registry']
SEAT_ROWS = [ortho.MANIFOLDS.index('ischial')]
SEAT_SEG = {r: next(seg for _sh, seg, _p, _n in observer.POINTS if _sh[0][0] == 'ischial-bilateral') for r in SEAT_ROWS}
SEAT_BOX = {r: tuple(1e-3 * np.array(_REGISTRY['ischial-bilateral']['cop-bounds-mm'][k]) for k in ('x', 'y')) for r in SEAT_ROWS}
#: Each of her observer's points' room along the floor, m: its segment's capsule's radius round it, as her skin's
#: contacts have it (`borne._room`). Points alone, her seat's ischia read her capture point 1 cm outside a seat it lay
#: 35 mm inside.
ROOMS = np.array([float(borne._room(edge.SKIN.index(seg), np.asarray(p, float))[0]) if seg in edge.SKIN else 0.0
                  for _sh, seg, p, _n in observer.POINTS])


def margin(point, pts, room, mask):
    """(m (k,), its edge's outward normal (k, 2)): `point` (2,) inside the hull of the discs `pts` (n, 2) of radii
    `room` (n,) that `mask` (k, n) keeps in each lane - the least of its support's reach past it over DIRECTIONS
    (`_U`): inside, its nearest edge's distance; outside, less than 0 by its distance to the hull; with none kept,
    -inf."""
    reach = reached(point, pts, room, mask)
    return reach.min(axis=1), _U[reach.argmin(axis=1)]


def told_nothing(told):
    """How far her text tells her soles nothing, 0 to 1: under her comparator's release share of what is told
    (`observer.said`; nothing told, her soles bear)."""
    return float(nothing(told))


def gate(xi, pts, room, soles, vcom, shares, loads, weight, left=None):
    """(eta (2,), (froude's, anchors' (2,), margin's (2,), margin N m (2,), its edge's normal (2, 2))): the physics'
    leave to release each leg - her centre's Froude number under the text's lockout (`vcom` world, y up), what her
    text tells bears her besides the leg (`shares`, `observer.said`; her soles' share split by their `loads`) at the
    text's least sum or over - weighed as felt, her released heels resting on the floor shut it -, her capture point
    `xi` (2,) inside what bears her besides it by the text's least margin of her `weight`: the points `pts` (n, 2) of
    radii `room`, each leg's own sole's points (`soles` (2, n), 1 its own) not among them. Its arithmetic `left`,
    `leave`'s C (`machine.law.motion.laid`), else numpy's; the edge its least reach's, picked here."""
    arithmetic = leave if left is None else left
    eta, by_froude, by_anchors, by_margin, nm, reach = arithmetic(xi, pts, room, soles, vcom, shares, loads, weight)
    return eta, (by_froude, by_anchors, by_margin, nm, _U[reach.argmin(axis=1)])


def told_on(obs, told, floor):
    """Which of her observer's points (`observer.POINTS`) are of what her text tells bears her besides her soles
    (`told`, `latent.MANIFOLDED`, her rows' order; her toes are her soles') within the prior's reach of the `floor`
    (`observer.REACH_M`)."""
    return (~obs['soled'] & (obs['row'] != PLANTAR) & (obs['latch_of'][:, 1:] @ told[1:] > 0.0)
            & (obs['points'][:, 1] - floor <= observer.REACH_M))


def supports(s, obs, told, floor, legs, loads):
    """(her floor points (n, 2), their radii (n,), each leg's own (2, n)): what bears her besides her soles as her
    text tells it (`told`, `latent.MANIFOLDED`) - her observer's points of each manifold told (`observer.laid`) within
    its prior's reach of the `floor` - and as her skin feels it - each contact but a let-go foot (`borne.LET`), its
    capsule's room round it -, and each sole's corners where it bears her (`leap.TOUCH_N`), as far as its leg is not
    asked released (`legs`, 2). Her skin's alone, her seat unloaded as she leant back left it, and her counterweight
    leant her further onto her hands and over."""
    from machine.senses.borne import LET
    held = [c for c in s.get('contacts', ()) if c['patch'] < LET]
    top = max(floor, float(s.get('prop', (0.0, 0.0, (0.0, 0.0, 0.0)))[2][1]), float(s['com'][1]) - 0.35)
    said = told_on(obs, told, top)
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
    """Each leg's release `dt` s on (`g['emancipated']`, 0 to 1), slewed toward her text's intent times her physics'
    leave; its cascade laid: `g['a'][LET_GO]` her step's let-go legs (the shed window's first edge; and as her text
    tells her soles nothing while something else bears her, `borne.latched`, or while what her text tells bears her
    body lies on the floor, `g['sat']`, held her comparator's dwell past it: on the floor her legs are anchors, never
    standing legs - stood there, her standing law launched her off her seat at 142-225 N), `g['unanchored']`
    her estimator's share of each sole as an anchor, `g['let_cap']` each let-go foot's push, N, `g['freed']` how far
    each leg moves - the parts in `g['emancipation']`; her legs' modes in `out` held (`g['held_ref']`) as far as a leg
    asked released is not yet free; her centre of mass's ask leant to her counterweight after (`counterweighted`).
    The arithmetic in `asked` and `released`, the support in `supports` and `gate` between them; each its C where
    `machine.law.motion.laid` compiled it."""
    weight, floor, loads = float(g['fr'][modal.WEIGHT]), float(g['fr'][modal.FLOOR]), np.asarray(g['loads'], float)
    was = g.get('emancipated', np.zeros(2))
    g['sat'] = max(g['seat_on'], g['sat'] * g['seat_near'])
    c, v = np.asarray(s['com'], float), np.asarray(s['vcom'], float)
    kept, iota, om, xi = g.get('em_asked', asked)(was, g['sat'], told, out, c, v, floor)
    pts, room, soles = supports(s, g['observer'], told, floor, iota, loads)
    eta, parts = gate(xi, pts, room, soles, v, observer.said(told), loads, weight, g.get('leave'))
    eta, gamma, let_go, unanchored, let_cap, freed, held, rate, acc = g.get('em_released', released)(
        was, iota, eta, kept, g['sat'], told, g['a'], g['y'], c, loads, g['fr'], g['held_ref'], out, dt)
    g['emancipated'], g['unanchored'], g['let_cap'], g['freed'] = gamma, unanchored, let_cap, freed
    g['a'][modal.LET_GO] = let_go
    g['held_ref'][...] = held
    out[latent.REF][LEGGED], out[latent.RATE][LEGGED], out[latent.ACC][LEGGED] = held, rate, acc
    g['emancipation'] = {'iota': iota, 'eta': eta, 'parts': parts, 'xi': xi, 'om': om, 'support': (pts, room, soles)}
    return gamma


def settled(acc, g, s):
    """`acc`, her centre of mass's ask (world, y up), sinking at the text's free-air seek (`field.SEEK_M_S`, at its rate) as
    far as what her text tells bears her body besides her soles and hands lies within her observer's reach of the floor
    and bears less of her than told (`s['prop']`, felt): onto it, that its load goes where her legs' and hands' is shed
    to; in free air only, till it touches (`g['sat']`). Held where she was, she sat on her hands and feet a millimetre
    over a seat told 0.92 of her and never felt; sinking on with her seat on the floor, her step lowered her centre of
    mass the one way left, her trunk back -10 -> -70 deg."""
    told, obs, floor = g['told'], g['observer'], float(g['fr'][modal.FLOOR])
    body = obs['row'] != ortho.PALMAR
    near = float((told_on(obs, told, floor) & body).any())
    said = observer.said(told)
    unfelt = CLIP(float(said.sum() - said[PLANTAR] - said[ortho.PALMAR]) - float(s['prop'][0]), 0.0, 1.0)
    out = np.array(acc, float)
    out[1] += near * (1.0 - g['sat']) * unfelt * field.RATE * (field.SEEK_M_S - float(s['vcom'][1]))
    return out


def counterweighted(acc, g):
    """`acc`, her centre of mass's ask (world, y up), with her capture point asked in of what bears her besides the
    leg her text most asks released, as far as it asks, to where her gate is open wholly (twice the text's margin, of
    her weight): the leg's cantilever on the rest of her (`feedforward-cantilever-reaction-wrench`), in by her
    pendulum's omega^2, from no further than its edge; none with nothing else bearing her. Asked of her pitch, her
    step's rows on her centre of mass held it and her trunk never leant."""
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
    """(her support's points (k, 3), the share her capture point's barrier holds her in it, `barrier.barred`): each
    sole that bears her (`bears`) her text does not ask released, and what else bears her as her observer feels it
    (`observer.bearing`) - her soles all where nothing else has three points -; held as far as what her rows lean on is
    unfelt (`unfelt`) or a leg is asked released, while she does not stride."""
    iota, others = g['emancipation']['iota'], observer.bearing(g['observer'])
    stood = bears & (iota < 1.0)
    kept = stood | ~stood.any() & (len(others) < 3)
    pts = np.concatenate([s['feet'][k]['pts'] for k in np.flatnonzero(kept)] + [others])
    return pts, (1.0 - float(g['a'][modal.STRIDES])) * max(float(bears.any()) * unfelt, float(iota.max()))


def opened(g, s, ask, dt):
    """`ask`'s posture of each let-go leg its open chain's (`open-chain`): its reference on its joints as they are
    while it bears; let go, its knee drawn to her text's over the cascade's slew, its hip and ankle where that
    knee puts its sole on the floor from its hip's height over its ankle as it was let go (`modal.seat`'s chain, that
    height held) and her pelvis on her pitch's aim (`impedance._poses`) - her legs' weight, out ahead of her, holds it
    there; on her pelvis as it was, her legs let it roll back 48 -> 61 deg and her trunk -5 -> -31 deg, on her pitch's
    want, a blend of that and the aim, her pelvis stayed at -50 deg and her spine's posture took her trunk down to
    it, -97 deg -, its yaw, roll and
    ankle's roll where they were; `ask['open']` the text's stiffness and damping. On her
    posture's own - her chair's legs on her pelvis's height as sensed, her crouch's - a let-go leg pressed its foot
    down as she rose and lifted her 0.16 -> 0.26 m; yielding to all its sole bore, its hip wound to -200 deg; drawn to
    her text's knee, past where her sit had it, her released knees folded 146 -> 162 deg and her heels dragged her."""
    high, ref, posture, opening_, let_hold = g.get('em_opening', opening)(
        g['a'][modal.LET_GO], s['q'], g['fr'], g['a'], g['y'], g['out'], g['high'], g['emancipation']['iota'],
        g['freed'], g['told'], g['sat'], ask['posture'], g['bend'], g['q_open'], g['posture'], g['q_lo'], g['q_hi'],
        g['emancipated'], dt)
    g['high'] = high
    g['q_open'][...] = ref
    ask['posture'][...] = posture
    ask['open'] = (OPEN['kp'], OPEN['kd'], opening_)
    ask['let_hold'] = let_hold


def seated(g, s):
    """`s['contacts']` with the seat her text tells bears her as the kernel's cone for it (`anatomical-manifold-registry`,
    the manifold `pan-wrench-homotopy` pans her onto): while it lies on the floor - the lowest of its segment's capsules
    within her sole's touch (`leap.TOUCH_M`), or felt within the prior's reach of it (felt on a chair 0.45 m up, she was
    given the floor's cone there and fell at 2.4 s), held while it stays within the prior's reach of the floor
    (`observer.REACH_M`; `g['sat']`, the pan's sync trigger) - its CoP box's four corners on the floor under her
    observer's points of it, her pelvis's heading its x, each a contact bearing what her step put on it the pass
    before (`g['seat_f']`) or a quarter of what her skin feels of it (its points taken out), its push grown from there
    (`rounds.GROW_N`): on her skin's alone, a seat a millimetre under her could take 120 N; held by her observer's
    comparator, it dropped out each time her hands took her and she rose off it. Two
    points across her ischia held no pitch: she rolled back over them; felt alone, a seat a millimetre under her bore
    0 N."""
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
        s['contacts'] = [c for c in s['contacts'] if c.get('body') != at]
        mid = obs['points'][obs['row'] == r].mean(axis=0)
        mid[1] = floor
        for corner, (dx, dz) in enumerate((dx, dz) for dx in SEAT_BOX[r][0] for dz in SEAT_BOX[r][1]):
                p = mid + dx * ahead + dz * across
                bears = max(0.25 * felt, float(g['seat_f'][corner]))
                b.mj.mj_jac(b.m, b.d, jp, jr, p, at)
                J = jp[:, :b.n].copy()
                b.mj.mj_jacDot(b.m, b.d, jp, jr, p, at)
                s['contacts'].append({'p': p, 'f': np.array([0.0, bears, 0.0]), 'J': J, 'dJv': jp[:, :b.n] @ s['v'],
                                      'carpal': 0.0, 'side': 0, 'patch': 2, 'room': 0.0, 'body': at, 'corner': corner})
    return s
