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

from machine import borne, edge, field, impedance, kernel, latent, leap, maneuver, melt, modal, observer, ortho, poise, rounds

#: The gate, the cascade as her text lays them (`xform-intentional-limb-emancipation`).
GATE = kernel.node('xform-intentional-limb-emancipation', 'release-gate')
OPEN = kernel.node('xform-intentional-limb-emancipation', 'open-chain')['open-chain-impedance']
CASCADE = kernel.node('xform-intentional-limb-emancipation', 'unloading-cascade')
#: Her Froude number's lockout; the least share of her weight what bears her besides a leg is told; the least margin
#: of her capture point inside it, N m (her weight times its distance in).
FROUDE, ANCHORS, MARGIN_NM = GATE['max-allowable-froude'], GATE['min-complement-weight-sum'], GATE['min-residual-cone-margin']
#: The cascade's slew to release, s, and back; its windows in a leg's release - her estimator's anchor let go, her
#: step's push shed -; the share of her weight a shed sole bears over its own leg's under which the leg is free to
#: move: a foot sliding on the floor bears its leg (her legs 0.38 of her); held to the text's 0.03 of her, lifted
#: foot's, her heels never let her knees go.
SLEW_S, RELOCK_S = 1e-3 * CASCADE['slew-duration-ms'], 1e-3 * CASCADE['relock-slew-ms']
UNANCHOR, SHED = CASCADE['observer-unanchor-window'], CASCADE['qp-load-shed-window']
FREED = CASCADE['kinematic-release-fz-floor']
#: A leg moves once shed, from the shed window's end.
FREE = (SHED[1], 1.0)
#: Her leg, m: what her Froude number is over.
LEG_M = maneuver._manifest()[1]['leg']
#: Her comparator's release share (`observer.RELEASE`): soles told less of what is told are told nothing.
TOLD_NOTHING = observer.RELEASE
PLANTAR = ortho.MANIFOLDS.index('plantar')
#: Her seat's manifolds facing up (`observer.UPRIGHT`): her rows, the segment their points lie on, the kernel's CoP box
#: for each (`anatomical-manifold-registry`'s cop-bounds, m: along her way, across).
_REGISTRY = kernel.read()['anatomical-manifold-registry']
SEAT_ROWS = [ortho.MANIFOLDS.index('ischial')]
SEAT_SEG = {r: next(seg for _sh, seg, _p, _n in observer.POINTS if _sh[0][0] == 'ischial-bilateral') for r in SEAT_ROWS}
SEAT_BOX = {r: tuple(1e-3 * np.array(_REGISTRY['ischial-bilateral']['cop-bounds-mm'][k]) for k in ('x', 'y')) for r in SEAT_ROWS}
#: Her legs' modes (`melt.LEGGED`): held where they were as a leg is asked released, till it is free - moved by her
#: text while its leg stood on the gate's leave, the sled's knees opened under 294 N on her soles and threw her back.
LEGGED = melt.LEGGED
#: Her legs' joints, a row a leg (`rounds.LEGS`).
LEG_JOINTS = np.array(rounds.LEGS)
#: The directions her support's margin is taken along: its error at most R (1 - cos(pi / DIRECTIONS)), 0.6 mm over
#: her 0.5 m reach.
DIRECTIONS = 64
#: Each of her observer's points' room along the floor, m: its segment's capsule's radius round it, as her skin's
#: contacts have it (`borne._room`). Points alone, her seat's ischia read her capture point 1 cm outside a seat it lay
#: 35 mm inside.
ROOMS = np.array([float(borne._room(edge.SKIN.index(seg), np.asarray(p, float))[0]) if seg in edge.SKIN else 0.0
                  for _sh, seg, p, _n in observer.POINTS])
_U = np.stack([np.cos(np.arange(DIRECTIONS) * 2.0 * np.pi / DIRECTIONS),
               np.sin(np.arange(DIRECTIONS) * 2.0 * np.pi / DIRECTIONS)], axis=1)


def smooth(x):
    """The smooth step of `x` over [0, 1]."""
    u = CLIP(x, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def window(gamma, span):
    """How far `gamma` is through the window `span` (lo, hi), 0 to 1."""
    return CLIP((gamma - span[0]) / (span[1] - span[0]), 0.0, 1.0)


def margin(point, pts, room, mask):
    """(m (k,), its edge's outward normal (k, 2)): `point` (2,) inside the hull of the discs `pts` (n, 2) of radii
    `room` (n,) that `mask` (k, n) keeps in each lane - the least of its support's reach past it over DIRECTIONS
    (`_U`): inside, its nearest edge's distance; outside, less than 0 by its distance to the hull; with none kept,
    -inf."""
    reach = np.where(mask[:, None, :] > 0.0, (_U @ pts.T + room)[None], -np.inf).max(axis=2) - (_U @ point)[None]
    return reach.min(axis=1), _U[reach.argmin(axis=1)]


def told_nothing(told):
    """How far her text tells her soles nothing, 0 to 1: under her comparator's release share of what is told
    (`observer.said`; nothing told, her soles bear)."""
    return float(CLIP(1.0 - observer.said(told)[PLANTAR] / TOLD_NOTHING, 0.0, 1.0))


def intent(told, out):
    """How far her text asks each leg released, 0 to 1 (2,): her soles told nothing (`told_nothing`), or the leg
    lifted past her poise's lift (`impedance.LIMBS`, its rhythm aside: her stride's clearance is no lift)."""
    ref = out[latent.REF][modal.LIFT] - out[latent.SWAY][modal.LIFT] * np.sin(out[latent.PSI][modal.LIFT])
    return np.maximum(told_nothing(told), window(impedance.LIMBS * ref, (0.0, poise.LIFT_EPS)))


def gate(xi, pts, room, soles, vcom, shares, loads, weight):
    """(eta (2,), (froude's, anchors' (2,), margin's (2,), margin N m (2,), its edge's normal (2, 2))): the physics'
    leave to release each leg - her centre's Froude number under the text's lockout (`vcom` world, y up), what her
    text tells bears her besides the leg (`shares`, `observer.said`; her soles' share split by their `loads`) at the
    text's least sum or over - weighed as felt, her released heels resting on the floor shut it -, her capture point
    `xi` (2,) inside what bears her besides it by the text's least margin of her `weight`: the points `pts` (n, 2) of
    radii `room`, each leg's own sole's points (`soles` (2, n), 1 its own) not among them."""
    froude = (vcom[0] * vcom[0] + vcom[2] * vcom[2]) / (modal.G * LEG_M)
    by_froude = np.exp(-(froude / FROUDE) ** 2)
    own = shares[PLANTAR] * loads / max(float(loads.sum()), observer.TINY)
    by_anchors = smooth((float(shares.sum()) - own - ANCHORS) / (1.0 - ANCHORS))
    m, out = margin(xi, pts, room, 1.0 - soles)
    by_margin = smooth((weight * m - MARGIN_NM) / MARGIN_NM)
    return by_froude * by_anchors * by_margin, (by_froude, by_anchors, by_margin, weight * m, out)


def told_on(obs, told, floor):
    """Which of her observer's points (`observer.POINTS`) are of what her text tells bears her besides her soles
    (`told`, `latent.MANIFOLDED`, her rows' order; her toes are her soles') within the prior's reach of the `floor`
    (`observer.REACH_M`)."""
    return (obs['skin'] & (obs['row'] != PLANTAR) & (told[np.maximum(obs['row'], 0)] > 0.0)
            & (obs['points'][:, 1] - floor <= observer.REACH_M))


def supports(s, obs, told, floor, legs, loads):
    """(her floor points (n, 2), their radii (n,), each leg's own (2, n)): what bears her besides her soles as her
    text tells it (`told`, `latent.MANIFOLDED`) - her observer's points of each manifold told (`observer.laid`) within
    its prior's reach of the `floor` - and as her skin feels it - each contact but a let-go foot (`borne.LET`), its
    capsule's room round it -, and each sole's corners where it bears her (`leap.TOUCH_N`), as far as its leg is not
    asked released (`legs`, 2). Her skin's alone, her seat unloaded as she leant back left it, and her counterweight
    leant her further onto her hands and over."""
    from machine.borne import LET
    held = [c for c in s.get('contacts', ()) if c['patch'] < LET]
    said = told_on(obs, told, floor)
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
    asked released is not yet free; her centre of mass's ask leant to her counterweight after (`counterweighted`)."""
    weight, floor, loads = float(g['fr'][modal.WEIGHT]), float(g['fr'][modal.FLOOR]), np.asarray(g['loads'], float)
    iota = intent(told, out)
    c, v = np.asarray(s['com'], float), np.asarray(s['vcom'], float)
    om = np.sqrt(modal.G / max(float(c[1]) - floor, impedance.LOW_M))
    xi = c[[0, 2]] + v[[0, 2]] / om
    pts, room, soles = supports(s, g['observer'], told, floor, iota, loads)
    eta, parts = gate(xi, pts, room, soles, v, observer.said(told), loads, weight)
    was = g.get('emancipated', np.zeros(2))
    gamma = was + CLIP(iota * eta - was, -dt / RELOCK_S, dt / SLEW_S)
    g['emancipated'] = gamma
    g['sat'] = max(g['seat_on'], g['sat'] * g['seat_near'])
    g['a'][modal.LET_GO] = np.maximum(window(gamma, (0.0, SHED[0])),
                                      max(told_nothing(told) * float(g['a'][modal.PROPPED]), g['sat']))
    g['unanchored'] = 1.0 - window(gamma, UNANCHOR)
    g['let_cap'] = (1.0 - window(gamma, SHED)) * weight
    leg = weight * (1.0 - 1.0 / float(g['fr'][modal.HIPS_BY]))
    g['freed'] = window(gamma, FREE) * CLIP(1.0 - (loads - leg) / (FREED * weight), 0.0, 1.0)
    hold, free = float(np.max(iota * (1.0 - g['freed']))), float(np.max(g['freed']))
    held = g['held_ref']
    held += (1.0 - hold) * (out[latent.REF][LEGGED] - held)
    out[latent.REF][LEGGED] = held
    out[latent.RATE][LEGGED] *= 1.0 - hold
    out[latent.ACC][LEGGED] *= 1.0 - hold
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
    k = int(np.argmax(e['iota']))
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
    let, q, fr, a, y = g['a'][modal.LET_GO], s['q'], g['fr'], g['a'], g['y']
    rot = fr[modal.ROT].reshape(3, 3)
    leg = modal._legs(y, rot)[0]
    g['high'] = let * g['high'] + (1.0 - let) * np.sqrt(leg[:, modal.UP] ** 2 + leg[:, modal.LEFT] ** 2)
    cosine = np.cos(0.5 * fr[modal.KNEE]) - modal.AXIAL_M * g['out'][latent.REF][modal.AXIAL] * fr[modal.HIPS_BY] / modal.LEG_M
    told = 2.0 * np.arccos(CLIP(cosine, -1.0, 1.0)) - fr[modal.KNEE] - a[modal.BEND]
    said = ask['posture'][modal.CHAIN[:, 1]] + g['bend'][modal.CHAIN[:, 1]] * told
    ref = g['q_open']
    ref[modal.CHAIN[:, 1]] += dt * let * (said - ref[modal.CHAIN[:, 1]]) / SLEW_S
    knee = modal.FLEX[1] * ref[modal.CHAIN[:, 1]]
    high = g['high']
    behind = np.sqrt(np.maximum(modal.SHANK ** 2 + modal.THIGH ** 2 + 2.0 * modal.SHANK * modal.THIGH * np.cos(knee)
                                - high ** 2, 0.0))
    lean = np.arctan2(-behind, high) + np.arctan2(modal.THIGH * np.sin(knee), modal.SHANK + modal.THIGH * np.cos(knee))
    aim = fr[modal.PITCH0][0] + modal.PITCH_RAD * g['out'][latent.REF][modal.PITCH]
    chain = np.stack([modal.FLEX[0] * (knee - lean + aim), modal.FLEX[1] * knee, modal.FLEX[2] * lean], axis=1)
    # both legs at once, each element's ops as a leg's alone (their joints apart): 20 numpy calls where 10
    joints, on = LEG_JOINTS, let[:, None]
    target = ref[joints]
    target[:, [2, 4]] = chain[:, [0, 2]]
    ref[joints] += dt * on * (target - ref[joints]) / SLEW_S
    ref[joints] = CLIP(on * ref[joints] + (1.0 - on) * q[joints], g['q_lo'][joints], g['q_hi'][joints])
    ask['posture'][joints] = on * ref[joints] + (1.0 - on) * ask['posture'][joints]
    ask['open'] = (OPEN['kp'], OPEN['kd'], g['freed'].copy())
    ask['let_hold'] = 1.0 - window(g['emancipated'], SHED)


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
        on = max(on, float(low <= leap.TOUCH_M), felt * float(low <= observer.REACH_M))
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
