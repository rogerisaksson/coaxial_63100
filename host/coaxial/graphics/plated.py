"""The robot bare of its clothes: its mechanism in its livery, bolted, its guards on it; and dressed.

The robot is its mechanism (`mechanism.parts`: carbon tubes, each drive's drum, rods on ball
joints, gimbals), every part of it a hard shell's in its livery's paints (`framed`: black
carbon tubes, magenta drives, steel) and each drive bolted - a ring of bolts on either face
of its motor's can -; on it its guards (`guards.laid`: honeycomb pads in carbon frames, stood
off its tubes), its seat (`seat.plate`: a black honeycomb net a side over its hip's roll drive, turning with it), its
hands and its head - a body's and a figure's where a likeness is worn, else as built - and its
shoes. No shell closes it: its tubes, its drives and the air between
them are what is seen. Dressed (`dressed`), it wears the likeness's clothes over that: what a
garment girds is not drawn, what is left of a triangle across its end a layer further
(`lit.UNDER`), and nothing of it stands out of them (`tools/sim/shown.py --out`).

    parts = plated.parts(meshes, extra, liked)   # [(name, parent, joints, offset, rest, mesh)]
    parts = plated.dressed(parts, liked)         # in the likeness's tee and jeans
"""
import math
from typing import Any

from coaxial.graphics import drums, guards, mechanism, seat
from coaxial.graphics.lit import PAINTED, UNDER, WOVEN, paint
from coaxial.graphics.livery import BOARD, BOLT, FRAME, GEAR, SERVO, STEEL, TUBE
from coaxial.graphics.panels import SHELL
from coaxial.graphics.shapes import turn_about
from machine.body import figure
from machine.body.guards import NEAR_M

#: A likeness's part, the segment its corners bend into, the local y where the bend begins and
#: ends (`drape.BLENDS`' columns): the trunk's and the limbs' joints. A shoulder's by the share
#: each corner carries (`likeness.fitted`), whichever of the two parts it is cut into. A hem at
#: the ankle bends with the foot over the last HEM_M of it, all of it at the joint: with no bend
#: there a shin leant over its foot - a squat, a kneel - and the shoe's instep stood 72 to 77 mm
#: out of the jeans' front, the instep up to 47 mm over the ankle.
HEM_M = 0.06
ANKLE = next(seg[3][1] for seg in figure.SEGMENTS if seg[0] == 'left_foot')
BLENDS = (('like_torso', 'pelvis', 0.03, -0.035), ('like_pelvis', '{side}_thigh', -0.02, -0.11),
          ('like_{side}_shank', '{side}_thigh', -0.01, 0.035), ('like_{side}_shank', '{side}_foot', ANKLE + HEM_M, ANKLE),
          ('like_{side}_upper_arm', 'torso', None, None), ('like_torso', '{side}_upper_arm', None, None),
          ('like_{side}_forearm', '{side}_upper_arm', -0.03, 0.03),
          ('like_{side}_hand', '{side}_forearm', -0.015, 0.015),
          ('like_neck', 'torso', -0.02, 0.02), ('like_head', 'neck', -0.02, 0.02))
#: What of a likeness is worn in the place of the robot's own - its head, its hands -, the
#: segments whose bones those hands stand for, and what of the robot's own head is kept where
#: no figure's is worn: by its name, or its name's end.
WORN = ('like_head', 'like_hair', 'like_fall', 'like_left_hand', 'like_right_hand')
HANDS = ('left_hand', 'left_fingers', 'right_hand', 'right_fingers')
KEPT, ENDS = ('jaw', 'nose', 'lips'), ('_ear', '_eye')
#: A drive is bolted: BOLTS heads a face of its motor's can, on a circle CIRCLE of its radius, a
#: hexagon BOLT_R round stood BOLT_M proud.
BOLTS, CIRCLE, BOLT_R, BOLT_M = 6, 0.72, 0.005, 0.004
#: A guard clears what stands by its segment at rest - a drive's can and its gearbox, a tube
#: (CLEARED, a part's name's start) - within NEAR_M of the segment's axis (the build's,
#: `machine.body.guards`) and DRUM_M of its ends.
CLEARED, DRUM_M = ('drive_', 'inv_', 'gear_', 'hung'), 0.06
#: Under its clothes: about a segment's bone - a limb's from its joint to the next's, the
#: trunk's the upright - cloth within REACH_M of it in UNDER_OF of SIDES ways round, told in
#: slabs BIN_M along it from SPAN_M before its joint, a slab with its two neighbours. What a
#: garment girds is not drawn: a triangle all in it, and one across its end with a corner
#: outside its cloth - a guard wider than its sleeve, a drum wider than its collar; what is
#: left across its end is a garment's layer further (`lit.UNDER`). Drawn a layer further 40 mm
#: within a garment's end, a guard stood through its sleeve at a shoulder: no layer holds what
#: stands 30 mm past the cloth. Told by the cloth before a corner, a shin's plate stood
#: through jeans whose leg lay 30 mm behind the shank: how far out a corner stands is nothing
#: to whether its limb is in a garment.
SIDES, UNDER_OF, REACH_M, BIN_M, SPAN_M, WITHIN_M = 8, 6, 0.25, 0.02, 1.0, 0.004
#: A garment's corner under another's (`lit.UNDER`) is tucked TUCK_M toward its segment's bone:
#: jeans under a tee's hem, as wide as the hem, stood out of it at a hip bent.
TUCK_M = 0.012
_SEGMENTS = frozenset(seg[0] for seg in figure.SEGMENTS)


def faces(axis, corners, triangles, spots):
    """(over, under): how far along `axis` the can (`corners`, `triangles`) stands over and under each
    of `spots` (n, 3) seen down its axis - its face there either way, a turned motor's slope or hub
    (`bells`) as a drum's flat; -inf over and inf under where none of it is."""
    from coaxial.model.blocks import numpy as np
    e1 = np.cross(axis, (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (1.0, 0.0, 0.0))
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(axis, e1)
    flat = np.stack([corners @ e1, corners @ e2], 1)[triangles]            # (t, 3, 2)
    a, b, c = flat[:, 0], flat[:, 1], flat[:, 2]
    q = np.stack([spots @ e1, spots @ e2], 1)[:, None]                     # (n, 1, 2)
    den = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
    den = np.where(np.abs(den) < 1e-15, np.nan, den)
    u = ((b[:, 1] - c[:, 1]) * (q[..., 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (q[..., 1] - c[:, 1])) / den
    v = ((c[:, 1] - a[:, 1]) * (q[..., 0] - c[:, 0]) + (a[:, 0] - c[:, 0]) * (q[..., 1] - c[:, 1])) / den
    w = 1.0 - u - v
    height = (corners @ axis)[triangles]
    at = u * height[:, 0] + v * height[:, 1] + w * height[:, 2]
    inside = (u >= -1e-9) & (v >= -1e-9) & (w >= -1e-9)
    return np.where(inside, at, -np.inf).max(axis=1), np.where(inside, at, np.inf).min(axis=1)


def _bolts(axis, corners, triangles):
    """A drive's bolts' mesh: BOLTS heads on either face of the can (`corners`, `triangles`, its
    part's frame) along its `axis` on a circle CIRCLE of its radius, each on its face where it
    stands (`faces`), a hexagon's prism each, steel."""
    from coaxial.model.blocks import numpy as np
    along = corners @ axis
    radius = float(np.linalg.norm(corners - np.outer(along, axis), axis=1).max())
    e1 = np.cross(axis, (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (1.0, 0.0, 0.0))
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(axis, e1)
    turn = np.arange(6) * (math.pi / 3.0)
    head = BOLT_R * (np.outer(np.cos(turn), e1) + np.outer(np.sin(turn), e2))
    sides = [row for i in range(6) for row in ((i, (i + 1) % 6, i + 6), ((i + 1) % 6, (i + 1) % 6 + 6, i + 6))]
    one = np.array(sides + [(6, 7, 8), (6, 8, 9), (6, 9, 11), (9, 10, 11)])
    ring = 2.0 * math.pi * np.arange(BOLTS) / BOLTS
    spots = CIRCLE * radius * (np.outer(np.cos(ring), e1) + np.outer(np.sin(ring), e2))
    over, under = faces(axis, corners, triangles, spots)
    cs, ts = [], []
    for face, way in ((over, 1.0), (under, -1.0)):
        for k in range(BOLTS):
            centre = axis * float(face[k]) + spots[k]
            ts.append(one + 12 * len(cs))
            cs.append(np.vstack([centre + head, centre + head + axis * (way * BOLT_M)]))
    out = np.vstack(cs)
    return out, np.vstack(ts), np.zeros((len(out), 2)), np.full(len(out), BOLT)


def framed(gone=()):
    """[(name, parent, joints, offset, rest, mesh)]: the mechanism's parts (`mechanism.parts`), a
    hard shell's each (`lit.HARD`: a servo drawn solid reads as one) in the livery's paints - a
    tube's black carbon, a board's black, a drive's motor and its board anodised and its
    gearbox darker, the rest steel -, after each drive's motor its bolts ('bolts_' and
    its joint); those named in `gone` kept empty."""
    from coaxial.model.blocks import numpy as np
    carbon, laminate, out = paint(mechanism.CARBON), paint(mechanism.PCB), []
    for *head, (corners, triangles, uv, materials) in mechanism.parts():
        name = head[0]
        ink = SERVO if name.startswith(('drive_', 'inv_')) else GEAR if name.startswith('gear_') else np.where(
            materials == carbon, TUBE, np.where(materials == laminate, BOARD, STEEL))
        mesh = (corners, triangles, uv, np.where(materials >= PAINTED, ink, materials))
        out.append((*head, tuple(each[:0] for each in mesh) if name in gone else mesh))
        if name.startswith('drive_'):
            joint = name[len('drive_'):]
            out.append(('bolts_' + joint, head[1], (), head[3], 0.0,
                        _bolts(np.array(drums.AXES[joint][0]), corners, triangles)))
    return out


def _own(meshes, extra):
    """{segment: corners}: the robot's own shell where its guards are laid with no body beside
    its figure - its limbs' and its trunk's lofts, its busts on that."""
    from coaxial.model.blocks import numpy as np
    out = {seg: mesh[0] for seg, mesh in meshes.items() if seg == 'torso' or seg.partition('_')[2] in guards.LIMB}
    busts = [rest[1][0] + np.asarray(rest[0], float) for name, _parent, *rest in extra
             if len(rest) == 2 and name.endswith('_bust') and not name.startswith('cloth_')]
    out['torso'] = np.vstack([out['torso'], *busts])
    return out


def parts(meshes, extra, liked: Any = None):
    """[(name, parent, joints, offset, rest, mesh)]: the robot bare of its clothes - its
    mechanism (`framed`), its head, its hands, its shoes and its guards. `meshes` {segment:
    mesh} and `extra` [(name, parent, offset, mesh)] are its shell as built: its head's parts
    (KEPT, ENDS) and its shoes are worn of it, carbon, and its guards are laid on its limbs'
    and its trunk's (`_own`). `liked` a figure's likeness's parts (`likeness.worn`'s, a body's
    with them: `form.worn`'s, 'form_' and their segments): its head, its face and its hair in
    place of the head as built; its hands, carbon - the body's where there is one - in place
    of the bones of the robot's own; its guards laid on the body's surface."""
    from coaxial.model.blocks import numpy as np
    liked = list(liked or ())
    body = {part[1]: part for part in liked if part[0].startswith('form_')}
    worn = [part for part in liked if part[0] in WORN and not (body and part[0].endswith('_hand'))]
    worn += [part for seg, part in body.items() if seg.endswith('_hand')]
    figured, z0 = any(part[0] == 'like_head' for part in worn), (0.0, 0.0, 0.0)
    out = framed(HANDS if any(part[0].endswith('_hand') for part in worn) else ())
    for name, parent, *rest in worn:
        joints, offset = (rest[0], rest[1]) if len(rest) == 3 else ((), rest[0])
        out.append((name, parent, joints, offset, 0.0, (*rest[-1][:3], np.where(rest[-1][3] == SHELL, TUBE, rest[-1][3]))))
    for seg, (corners, triangles, uv, materials) in meshes.items():
        if seg.endswith(('_foot', '_toes')):
            out.append(('shell_' + seg, seg, (), z0, 0.0, (corners, triangles, uv, np.full(len(corners), TUBE))))
        elif seg == 'head' and not figured:
            out.append(('shell_head', seg, (), z0, 0.0, (corners, triangles, uv, np.where(materials == SHELL, FRAME, materials))))
    for name, parent, *rest in extra:
        if len(rest) == 2 and parent == 'head' and not figured and (name in KEPT or name.endswith(ENDS)):
            out.append(('shell_' + name, parent, (), rest[0], 0.0, rest[1]))
    placed, frames = _rested(out)
    rested = [(part[0], part[1], part[5][0] @ frame[0].T + frame[1], part[5][1]) for part, frame in zip(out, frames)
              if frame is not None and len(part[5][0])]
    pelvis = [(c, t) for _n, parent, c, t in rested if parent == 'pelvis']
    for name, side in (('left', 1.0), ('right', -1.0)):
        joint = placed[name + '_hip_yaw'][1]
        drive = ([(n, c, t) for n, parent, c, t in rested if parent == name + '_hip_yaw'], joint)
        out.append((name + '_seat', name + '_hip_yaw', (), tuple(-joint), 0.0, seat.plate(side, drive, pelvis)))
    return out +[(name, parent, (), offset, 0.0, mesh) for name, parent, offset, mesh in guards.laid(cleared(out, body, meshes, extra))]


def cleared(parts_, body, meshes, extra):
    """{segment: (corners, the mechanism's by it), its frame}: the surfaces the robot's guards
    are laid on - a body's parts' (`body` {segment: its 'form_' part}) or, with none, its own
    shell's (`_own`) -, and with each what of the mechanism `parts_` stands by its segment at
    rest (CLEARED): a guard clears the drum it lies over, a knee's cup its knee's."""
    from coaxial.model.blocks import numpy as np
    surfaces = {seg: part[-1][0] for seg, part in body.items()} or _own(meshes, extra)
    placed, frames = _rested(parts_)
    hard = [part[5][0] @ frame[0].T + frame[1] for part, frame in zip(parts_, frames)
            if frame is not None and len(part[5][0]) and part[0].startswith(CLEARED)]
    for seg, corners in surfaces.items():
        turn, spot = placed[seg]
        low, high = float(corners[:, 1].min()) - DRUM_M, float(corners[:, 1].max()) + DRUM_M
        by = [at[(at[:, 1] > low) & (at[:, 1] < high) & (np.hypot(at[:, 0], at[:, 2]) < NEAR_M)]
              for at in ((world - spot) @ turn for world in hard)]
        surfaces[seg] = (corners, np.vstack([np.zeros((0, 3)), *by]))
    return surfaces


def _frame(part):
    """Whether `part` is one another hangs on or is posed by - a segment, a stage, a crank, a
    rod, a rod's end -: kept, drawn or not."""
    return part[0] in _SEGMENTS or part[0] in figure.STAGES or part[1] == '*' or part[0].startswith('end_')


def _rested(parts_):
    """({name: (turn, spot)}, [(turn, spot)] a part's each) of `parts_` at rest, the first at the
    origin: each on its parent by its offset and its rest turn, those posed a frame at a time
    ('*') as they are laid at rest (`mechanism.posed`)."""
    from coaxial.model.blocks import numpy as np
    placed = {}
    for name, parent, _joints, offset, rest, _mesh in parts_:
        if parent == '*':
            continue
        above, at = placed[parent] if parent else (np.eye(3), np.zeros(3))
        placed[name] = (above @ turn_about('z', rest) if rest else above, at + above @ np.asarray(offset, float))
    frames = [placed.get(part[0]) if part[1] != '*' else None for part in parts_]
    mechanism.posed(parts_, placed, {}, frames)
    return placed, frames


def _bones(placed):
    """{segment: (its joint, its bone's way)} at rest, `placed` {name: (turn, spot)}: a limb's
    bone from its joint to the next's; a hand's, a foot's and what hangs on them their limb's
    on; the trunk's the upright."""
    from coaxial.model.blocks import numpy as np
    kids, out = {}, {}
    for name, parent, *_rest in figure.SEGMENTS:
        kids.setdefault(parent, []).append(name)
    for name, parent, *_rest in figure.SEGMENTS:
        on = [kid for kid in kids.get(name, ()) if kids.get(kid)]
        if not name.startswith(('left_', 'right_')):
            way = np.array([0.0, 1.0, 0.0])
        elif on:
            way = placed[on[0]][1] - placed[name][1]
            way = way / np.linalg.norm(way)
        else:
            way = out[parent][1]
        out[name] = (placed[name][1], way)
    return out


def _girt(cover, bones, beside):
    """{segment: told}: about each bone of `bones` (`_bones`), slabs BIN_M thick from SPAN_M
    before its joint, girt where the cover (`cover`, its points (m, 3)) stands within REACH_M
    of the bone in UNDER_OF of SIDES ways round - a slab with its two neighbours where
    `beside`. `told(points)` is, a point's each, (whether its slab is girt, how far inside the
    cover's least reach its way it stands, m)."""
    from coaxial.model.blocks import numpy as np
    out, n = {}, int(2.0 * SPAN_M / BIN_M)
    for seg, (joint, way) in bones.items():
        aside = np.cross(way, (0.0, 0.0, 1.0) if abs(way[2]) < 0.9 else (1.0, 0.0, 0.0))
        aside /= np.linalg.norm(aside)

        def laid(points, joint=joint, way=way, aside=aside, ahead=np.cross(way, aside)):
            off = points - joint
            t, x, z = off @ way, off @ aside, off @ ahead
            return (np.clip((t + SPAN_M) / BIN_M, 0, n - 1).astype(int),
                    np.floor(np.arctan2(z, x) / (2.0 * math.pi) * SIDES).astype(int) % SIDES, np.hypot(x, z), np.abs(t) < SPAN_M)
        slab, side, reach, on = laid(cover)
        on &= (reach < REACH_M) & (reach > WITHIN_M)
        least = np.full((n, SIDES), np.inf)
        np.minimum.at(least, (slab[on], side[on]), reach[on])
        seen = np.pad(np.isfinite(least), ((1, 1), (0, 0)))
        girt = (seen[1:-1] | (seen[:-2] | seen[2:]) & bool(beside)).sum(axis=1) >= UNDER_OF

        def told(points, laid=laid, girt=girt, least=least):
            slab, side, reach, _on = laid(points)
            return girt[slab], least[slab, side] - reach
        out[seg] = told
    return out


def _under(parts_, cloth):
    """`parts_` with what of them is under the garments `cloth` (parts) seen to: at rest a corner
    is under where cloth girds its segment's bone there (`_girt`) - a triangle all under is
    not drawn, nor one with a corner under and outside the cloth; what is left of a triangle
    across a garment's end is a garment's layer further (`lit.UNDER`). A part with nothing
    left to draw is gone, but a frame another hangs on or
    is posed by - a segment, a stage, a crank, a rod, a rod's end. What the robot wears of a
    likeness (WORN) is over all."""
    from coaxial.model.blocks import numpy as np
    segs = {seg[0]: seg[1] for seg in figure.SEGMENTS}
    turned = {joint: seg[0] for seg in figure.SEGMENTS for joint, *_rest in seg[2]}
    placed, frames = _rested(parts_ + cloth)

    def stood(k, part):
        turn, spot = frames[k]
        return part[5][0] @ turn.T + spot
    # A garment's points: its corners, its edges' middles and its triangles' - closer than a slab.
    points = np.vstack([np.vstack([at[tri].mean(axis=1), *(at[tri[:, [i, j]]].mean(axis=1) for i, j in ((0, 1), (1, 2), (2, 0))), at])
                        for k, part in enumerate(cloth, len(parts_)) for at, tri in [(stood(k, part), part[5][1])]])
    up, girt, out = {part[0]: part[1] for part in parts_}, _girt(points, _bones(placed), True), []
    for k, part in enumerate(parts_):
        corners, triangles, uv, worn = part[5]
        seg = part[0]
        if part[1] == '*':                     # a crank, a pin, a rod: along the segment over the one its joint turns
            seg = segs.get(turned.get(seg.split('_', 1)[1]))
        else:
            while seg not in segs:             # up its stages to the segment it hangs on
                seg = up[seg]
        if not len(triangles) or seg not in girt or part[0] in WORN:
            out += [part] if len(triangles) or _frame(part) else []
            continue
        under, inside = girt[seg](stood(k, part))
        gone = under[triangles].all(axis=1) | (under & (inside < 0.0))[triangles].any(axis=1)
        used, index = np.unique(triangles[~gone], return_inverse=True)
        if len(used) or _frame(part):
            flagged = np.where(under & (worn >= PAINTED), worn | UNDER, worn)
            out.append((*part[:5], (corners[used], index.reshape(-1, 3), uv[used], flagged[used])))
    return out


def dressed(parts_, liked):
    """`parts_` (`parts`'s) in a likeness's clothes, `liked` its parts (`likeness.worn`'s): of each
    its cloth alone - a part 'like_' and its segment, bent as the likeness's is
    (BLENDS), what of it lies under another garment tucked (TUCK_M) - and of the
    robot what its clothes leave to see (`_under`): an arm past its sleeve's cuff, its neck
    over the collar, a foot under a hem. No cloth is stood through."""
    from coaxial.model.blocks import numpy as np
    cloth = []
    for name, parent, *rest in liked:
        corners, triangles, uv, worn = rest[-1]
        t = triangles[((worn & WOVEN) > 0)[triangles].all(axis=1)]
        if name not in WORN and len(t):
            used, index = np.unique(t, return_inverse=True)
            at, tucked = corners[used].copy(), (worn[used] & UNDER) > 0
            reach = np.hypot(at[tucked, 0], at[tucked, 2])
            at[tucked] *= np.stack([1.0 - TUCK_M / np.maximum(reach, 2.0 * TUCK_M), np.ones(len(reach))], 1)[:, [0, 1, 0]]
            cloth.append((name, parent, (), rest[0], 0.0, (at, index.reshape(-1, 3), uv[used], worn[used])))
    return _under(list(parts_), cloth) + cloth
