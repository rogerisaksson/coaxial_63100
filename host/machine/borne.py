"""What bears her off her soles as her skin feels it: her law's support, her step's contacts.

    prop, contacts, palms = borne.read(b, s, y, weight, hands)   # `wbc.sense`'s s, `machine.edge`'s y

Each segment above her soles pushed up (`edge.TOUCH`) at its touch's centre (`edge.AT`), placed
by her own model as her estimate has her - a seat, a bed, the floor under a knee or her hands -:
prop, the share of her weight the furniture bears and the box what bears her spans, her hands' points too,
world (share, low (3,), high (3,)); contacts, each such point, its force now (3,), its Jacobian (3, n) and that's
drift (3,), whose force the step finds (`machine.wrench`). Its sensed force fed to the step
instead, a stiff seat's a pass late, the step cancelled the seat's stiffness a pass after it
pushed - 695, 250, 88, 539, 131 N on it 30 ms apart, her soles' loads swinging 0 to 286 N.
Her hands bear as the furniture does, a patch of their own (CARPAL), out of prop's share: palms, each one's.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import edge, figure, supervisor, touch
from machine.modal import PALMS, PROP, PROP_HI, PROP_LO, PROPPED, PROPPED_BELOW, PROPPED_BOX, PROPPED_SINCE

#: A segment bears that is pushed up past this, N: her skin's touch.
BEARS_N = edge.TOUCHED_N
#: Her arms' segments, upper arm to fingers, indices into her skin - her left's first: a hand braced, or a
#: forearm down on the bed as she lies back onto her elbows.
HANDS = np.array(touch.ARMS[0] + touch.ARMS[1])
#: The segments above her soles (`touch.ABOVE`) and her arms', indices into her skin.
ABOVE = np.array(sorted(set(touch.ABOVE) | set(HANDS.tolist())))
#: What bears her bounded where nothing does: its low and high corners, m, the bounds a touch widens.
NOWHERE = np.full(3, np.inf), np.full(3, -np.inf)
#: Her pelvis among them: its skin's one touch is her two ischia's (`figure.SEAT_X` either side), split by
#: where it lies between them - the force-weighted point of two lines gives their shares exactly - and laid
#: on them as far as it pushes up her pelvis (sat; lying on her back or side the two meet at the touch).
#: One point, a seat held her roll by nothing: sat on her pelvis alone she rolled off at 2e-4 rad.
PELVIS = edge.SKIN.index('pelvis')
#: Her thighs and shanks: sat on a bed's edge with her hands asked behind her (`hands > 0`) or a leg released
#: (`let_go`), her pelvis holds the bed; holding her legs still on the mattress (`wrench.W_STILL`) locked her
#: hip and knee joints against her recline and leg swing.
THIGHS = tuple(edge.SKIN.index(s + p) for s in ('left_', 'right_') for p in ('thigh', 'shank'))
#: What bears her is a surface under her skin: a bearing point's room is its segment's capsule's radius round
#: it along the floor (CAPSULES, `figure.BODY` by its name without its side; MuJoCo gives a capsule one
#: contact point). Points alone, sat on her pelvis its box was the ischia's line and her centre of mass was
#: asked back to it at 1.5 m/s^2.
CAPSULES = [[(r, np.asarray(a, float), np.asarray(b, float)) for seg, r, a, b in figure.BODY
             if seg == (name.split('_', 1)[1] if name.startswith(('left_', 'right_')) else name)] for name in edge.SKIN]
ALONG = np.array([1.0, 0.0, 1.0])


#: Her support's comparator, the master insert's (`supervisor`): latched from PROP_ENGAGE of her
#: weight, let go under PROP_RELEASE held PROP_DWELL_S, never within PROP_BLANK_S of its strike.
_CMP = supervisor.insert()['contact-comparators']
PROP_ENGAGE, PROP_RELEASE = _CMP['schmitt-high-engage-fz'], _CMP['schmitt-low-release-fz']
PROP_DWELL_S, PROP_BLANK_S = 1e-3 * _CMP['min-state-dwell-ms'], 1e-3 * _CMP['impact-blanking-window-ms']


def _room(k, local):
    """(3,) m, skin segment `k`'s room round a point `local` of it (its frame): along the floor the radius of its
    capsule nearest the point, none a point."""
    caps = CAPSULES[k]
    if not caps:
        return np.zeros(3)
    gaps = []
    for r, a, b in caps:
        ab = b - a
        t = float(np.clip((local - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0))
        gaps.append((float(np.linalg.norm(local - (a + t * ab))) - r, r))
    return min(gaps)[1] * ALONG
#: The segments one surface bears are one patch of her step: a push's way, the world's axis it is most along and
#: its sign - friction leans a push by atan MU, under 45 deg, so a seat's, a bed's or a floor's pushes are all
#: up, a backrest's or a wall's all one way across (the lodge's surfaces stand square; a yawed wall's split). Each
#: hand a patch of its own past every other's (CARPAL on, a side's apart): split as felt between them, one hand's
#: share held what it was and the asymmetry grew till it let go. A let-go foot's past both hands' (LET on): held
#: against the floor alone - held still along it too, her legs sliding out as she sat braked them 102 N and threw
#: her back over her seat. Each let-go foot a patch of its own (LET on, a side's apart), its push its own cap
#: (`machine.emancipation`): shed one by one.
CARPAL = 6
LET = 3 * CARPAL


def _points(k, push, at, pivot=False):
    """((share, point in the segment's frame), ..) of skin segment `k`'s touch `push` at `at`: one, her pelvis's
    two ischia's (PELVIS) unless pivoting on the bed (`pivot`)."""
    if k != PELVIS or pivot:
        return ((1.0, at),)
    sat = float(np.clip(push[1] / max(float(np.linalg.norm(push)), 1e-9), 0.0, 1.0))
    left = float(np.clip((at[0] + figure.SEAT_X) / (2.0 * figure.SEAT_X), 0.0, 1.0))
    to_left = np.array([sat * (figure.SEAT_X - at[0]), 0.0, 0.0])
    to_right = np.array([sat * (-figure.SEAT_X - at[0]), 0.0, 0.0])
    return ((left, at + to_left), (1.0 - left, at + to_right))


def read(b, s, y, weight, hands=0.0, let_go=(0.0, 0.0)):
    """(prop, contacts, palms) of `y`'s touch above her soles: her model `b`'s data posed as `wbc.sense`
    left it for `s`, `weight` hers, N; her hands among them where her text tells them to bear (`hands`, their
    share told, `ortho.PALMAR`) - untold, an arm swinging over a seat holds no hand still on it -, a let-go leg's
    foot too, not its toes (`let_go`, `machine.emancipation`): no contact, the step took that leg for hanging on the
    floor; its toes held still, their passive joint was asked 500-835 rad/s^2 and the whole ask shrank to nothing."""
    m, d, n = b.m, b.d, b.n
    ids = getattr(b, 'skin', None)
    if ids is None:
        ids = b.skin = np.array([m.body(name).id for name in edge.SKIN])
        b.jacs = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    pushes, at = y[edge.TOUCH].reshape(-1, 3), y[edge.AT].reshape(-1, 3)
    up, low, high, contacts, palms = 0.0, NOWHERE[0], NOWHERE[1], [], np.zeros(2)
    soled = [touch.SOLES[k][0] for k in (0, 1) if let_go[k] > 0.5]
    foot = {touch.SOLES[k][0]: k for k in (0, 1)}
    skin = np.concatenate([ABOVE, soled]).astype(int)
    pushed = skin[pushes[skin].any(axis=1) & ((hands > 0.0) | ~np.isin(skin, HANDS))]
    pivot = bool(soled) and (d.xmat[ids[PELVIS]].reshape(3, 3) @ pushes[PELVIS])[1] > BEARS_N
    ischial = pivot or (hands > 0.0 and (d.xmat[ids[PELVIS]].reshape(3, 3) @ pushes[PELVIS])[1] > BEARS_N)
    jp, jr = b.jacs
    for k in pushed:
        R = d.xmat[ids[k]].reshape(3, 3)
        f = R @ pushes[k]
        if f[1] <= BEARS_N:
            continue
        # a let-go foot is a contact, not what bears her off her soles: in her props' box her centre was asked out over
        # her feet at the floor's grip, 5.9 m/s^2 each way, and a hand flew 0.7 m up
        prop = float(k not in soled)
        up += f[1] * float(k not in HANDS) * prop
        let = int(k in soled)
        side = int(k in HANDS[len(touch.ARMS[0]):]) + let * foot.get(k, 0)
        carpal = float(k in HANDS)
        palms[side] += f[1] * carpal
        axis = int(np.argmax(np.abs(f)))
        patch = 2 * axis + int(f[axis] < 0.0) + CARPAL * (1 + side) * int(carpal) + LET * (1 + side) * let
        for share, local in _points(k, pushes[k], at[k], pivot):
            p, room = d.xpos[ids[k]] + R @ local, _room(k, local)
            low, high = np.where(prop, np.minimum(low, p - room), low), np.where(prop, np.maximum(high, p + room), high)
            if ischial and k in THIGHS:
                continue
            b.mj.mj_jac(m, d, jp, jr, p, ids[k])
            J = jp[:, :n].copy()
            b.mj.mj_jacDot(m, d, jp, jr, p, ids[k])
            contacts.append({'p': p, 'f': share * f, 'J': J, 'dJv': jp[:, :n] @ s['v'], 'carpal': carpal,
                             'side': side, 'patch': patch, 'room': float(room[0]), 'body': int(ids[k])})
    held = np.isfinite(low[0])
    return (up / weight, np.where(held, low, 0.0), np.where(held, high, 0.0)), contacts, palms / weight


def latched(y, rot, a, dt):
    """(her support off her soles, 0 or 1; (its box's low and high corners along and across in her
    way's frame, that support)): what bears her - a seat, a bed, a knee's floor - on her support's
    comparator, kept in `a` and stepped `dt` s on; its box held while what bears her is under its
    release; her hands' share with the furniture's (`borne`): the furniture's alone, a hitch on her
    hands let it go under the release and her soles stood under her again. Its share ramped to
    PROP_ON, a brush of the bed's frame at the dock, 0.07 of her,
    counted 0.07, her parry stepped her back into it and over the edge; reclining on the bed's edge
    it went 0.66, 0, 0.17, 0.71 in a second, each zero dropped the bed from her support, her soles
    lifted in turn and she slid onto the mattress."""
    share, latched = y[PROP] + y[PALMS].sum(), a[PROPPED]
    on = (1.0 - latched) * float(share >= PROP_ENGAGE)
    under = latched * float(share < PROP_RELEASE)
    a[PROPPED_BELOW] = under * (a[PROPPED_BELOW] + dt)
    a[PROPPED_SINCE] = (1.0 - on) * (a[PROPPED_SINCE] + dt)
    off = under * float(a[PROPPED_SINCE] >= PROP_BLANK_S) * float(a[PROPPED_BELOW] >= PROP_DWELL_S)
    a[PROPPED] = latched + on - off
    xs, zs = np.array([y[PROP_LO][0], y[PROP_HI][0]]), np.array([y[PROP_LO][2], y[PROP_HI][2]])
    way = np.stack([np.repeat(xs, 2), np.zeros(4), np.tile(zs, 2)], axis=1) @ rot.T
    box = np.concatenate([way[:, :2].min(axis=0), way[:, :2].max(axis=0)])
    a[PROPPED_BOX] = np.where(share >= PROP_RELEASE, box, a[PROPPED_BOX])
    return a[PROPPED], (a[PROPPED_BOX][:2].copy(), a[PROPPED_BOX][2:].copy(), a[PROPPED])
