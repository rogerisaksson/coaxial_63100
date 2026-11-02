"""Humanoid G-code: a general rig's joint angles from a mocap, converted to hers, read as her program.

    rows = gcode.rig(joints)                        # (n, len(WORDS)) of a take's joints (`fbx.take`), the general rig
    text = gcode.text(header, times, WORDS, rows)   # a program's text; `parse` reads it back
    hers = gcode.hers(header, times, rows)          # (header, times, her joints, her degrees), her limits kept
    beats = gcode.program(path)                     # her program's beats as `joints` plays them

The dialect: lines of `; key value` head a program; a move is `G1 T<s> <WORD><value> ...`, a word letters and
underscores, its value a number; the general rig's words (WORDS) are the pelvis's place and turn (X Y Z m, A B C
roll pitch yaw deg), the spine's flexion, lateral bend and yaw (SPF SPL SPY), the neck's (NKF NKL) and a side's
(L or R) hip flexion, abduction and rotation (HF HA HR), knee (KF), ankle dorsiflexion and roll (AF AR), shoulder
flexion, abduction and rotation (SF SA SR), elbow (EF) and wrist (WF), all degrees, 0 the anatomical neutral,
flexion forward, abduction out and rotation external positive. Her program's words are her joints (`figure.JOINTS`).

The rig's angles come from twenty joints' places alone (JOINTS: a take's HumanIK names, hers by `SEGMENTS`): the
pelvis frame from the hips and the spine's base, the trunk's from the spine and the shoulders, each bone's
direction in its parent's frame; a hip's or shoulder's rotation from the plane its knee or elbow bends in, read
once the bend passes BENT. Converting to her: the same extraction on her own figure (`figure.frames`) at a stand
and with each driven joint moved CALIBRATE deg tells which word a joint moves, its sign and gain (`calibrated`;
words no joint moves are dropped), the take's times scaled by dynamic similarity (the rig's leg to hers), each
joint clamped to its drive's range and each move slowed to its drives' speeds (`defrobot`), the report in her
program's header.
"""
import math
import os

from machine.body import defrobot, figure, mechatronics
from machine.errors import MachineError

#: Her programs' home, the host's gcode/: `<name>.gcode` beside `<name>.general.gcode`.
DIR = os.path.join(os.path.dirname(mechatronics.HERE), 'gcode')
#: The general rig's joints (HumanIK's names) and her segment each is the origin of.
SEGMENTS = {'Hips': 'pelvis', 'Spine': 'torso', 'Neck': 'neck', 'Head': 'head'}
SEGMENTS.update({side + n: side.lower() + '_' + seg for side in ('Left', 'Right')
                 for n, seg in (('UpLeg', 'thigh'), ('Leg', 'shank'), ('Foot', 'foot'), ('ToeBase', 'toes'),
                                ('Arm', 'upper_arm'), ('ForeArm', 'forearm'), ('Hand', 'hand'), ('HandMiddle1', 'fingers'))})
JOINTS = tuple(SEGMENTS)
SIDES = (('L', 'Left', 1.0), ('R', 'Right', -1.0))
#: The general rig's words, in a move's order.
WORDS = ('X', 'Y', 'Z', 'A', 'B', 'C', 'SPF', 'SPL', 'SPY', 'NKF', 'NKL') + tuple(
    s + w for s, _side, _sign in SIDES for w in ('HF', 'HA', 'HR', 'KF', 'AF', 'AR', 'SF', 'SA', 'SR', 'EF', 'WF'))
#: A bend, deg, past which a joint's rotation reads off its child's bending plane; a joint moved this far, deg,
#: on her figure calibrates its word; a word moved less than this per degree is no joint's; the bends her figure
#: is calibrated at, deg, so every rotation reads: her stand's knees and elbows are straighter than BENT.
BENT, CALIBRATE, GAIN = 10.0, 5.0, 0.3
BASE = {'left_hip': -20.0, 'right_hip': -20.0, 'left_knee': 30.0, 'right_knee': 30.0, 'left_elbow': 30.0, 'right_elbow': 30.0}


def _unit(v):
    import numpy as np
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-9)


def _dot(a, b):
    import numpy as np
    return np.sum(a * b, axis=-1)


def _angle(u, v):
    import numpy as np
    return np.degrees(np.arccos(np.clip(_dot(_unit(u), _unit(v)), -1.0, 1.0)))


def _asin(x):
    import numpy as np
    return np.degrees(np.arcsin(np.clip(x, -1.0, 1.0)))


def _frame(up, left):
    """(forward, up, left) unit rows of a frame whose up is `up` and whose left is `left` squared to it."""
    import numpy as np
    up = _unit(up)
    left = _unit(left - _dot(left, up)[:, None] * up)
    return np.cross(left, up), up, left


def _bend(child, bone, forward, left, sign):
    """A bone's rotation, deg, from the plane its `child` bends in: the bend's direction off `forward`, toward
    `left` positive (`sign` mirrors the right side), 0 within BENT of straight."""
    import numpy as np
    b = child - _dot(child, bone)[:, None] * bone
    n = np.linalg.norm(b, axis=-1)
    b = b / np.maximum(n, 1e-9)[:, None]
    return np.degrees(np.arctan2(sign * _dot(b, left), _dot(b, forward))) * (n > math.sin(math.radians(BENT)))


def rig(joints):
    """(n, len(WORDS)) the general rig's words of `joints` {name: (n, 3) places, y up}, a row a frame."""
    import numpy as np
    J = {k: np.asarray(joints[k], float) for k in JOINTS}
    f_p, u_p, l_p = _frame(J['Spine'] - J['Hips'], J['LeftUpLeg'] - J['RightUpLeg'])
    f_t, u_t, l_t = _frame(J['Neck'] - J['Spine'], J['LeftArm'] - J['RightArm'])
    head = _unit(J['Head'] - J['Neck'])
    out = {'X': J['Hips'][:, 0], 'Y': J['Hips'][:, 1], 'Z': J['Hips'][:, 2],
           'A': np.degrees(np.arctan2(l_p[:, 1], u_p[:, 1])), 'B': _asin(f_p[:, 1]),
           'C': np.degrees(np.arctan2(f_p[:, 0], f_p[:, 2])),
           'SPF': np.degrees(np.arctan2(_dot(u_t, f_p), _dot(u_t, u_p))), 'SPL': _asin(_dot(u_t, l_p)),
           'SPY': np.degrees(np.arctan2(-_dot(l_t, f_p), _dot(l_t, l_p))),
           'NKF': np.degrees(np.arctan2(_dot(head, f_t), _dot(head, u_t))), 'NKL': _asin(_dot(head, l_t))}
    for s, side, sign in SIDES:
        d, sh = _unit(J[side + 'Leg'] - J[side + 'UpLeg']), _unit(J[side + 'Foot'] - J[side + 'Leg'])
        toe = _unit(J[side + 'ToeBase'] - J[side + 'Foot'])
        hf = np.arctan2(_dot(d, f_p), -_dot(d, u_p))
        f_thigh = np.cos(hf)[:, None] * f_p + np.sin(hf)[:, None] * u_p
        out[s + 'HF'], out[s + 'HA'] = np.degrees(hf), _asin(sign * _dot(d, l_p))
        out[s + 'HR'] = _bend(sh, d, -f_thigh, -l_p, sign)
        out[s + 'KF'], out[s + 'AF'] = _angle(d, sh), _angle(sh, toe) - 90.0
        knee = np.cross(d, sh)                                        # the knee's bending plane's normal, toward her left
        knee = _unit(knee * np.sign(_dot(knee, l_p) + 1e-12)[:, None])
        out[s + 'AR'] = _asin(sign * _dot(toe, knee)) * (_angle(d, sh) > BENT)
        a, r = _unit(J[side + 'ForeArm'] - J[side + 'Arm']), _unit(J[side + 'Hand'] - J[side + 'ForeArm'])
        h = _unit(J[side + 'HandMiddle1'] - J[side + 'Hand'])
        sf = np.arctan2(_dot(a, f_t), -_dot(a, u_t))
        f_arm = np.cos(sf)[:, None] * f_t + np.sin(sf)[:, None] * u_t
        out[s + 'SF'], out[s + 'SA'] = np.degrees(sf), _asin(sign * _dot(a, l_t))
        out[s + 'SR'], out[s + 'EF'] = _bend(r, a, f_arm, l_t, sign), _angle(a, r)
        c = r - _dot(r, a)[:, None] * a
        out[s + 'WF'] = _angle(r, h) * np.sign(_dot(h, c) + 1e-12)
    return _held(np.stack([out[w] for w in WORDS], axis=1))


def _held(values):
    """`values` with each rotation word (HR, SR) unwrapped over the frames and held where its bend (KF, EF) is
    within BENT of straight, the ankle roll held likewise: read only past BENT, they would step otherwise."""
    import numpy as np
    for word, bend in (('HR', 'KF'), ('SR', 'EF'), ('AR', 'KF')):
        for s, _side, _sign in SIDES:
            k, b = WORDS.index(s + word), WORDS.index(s + bend)
            readable = values[:, b] > BENT
            v = values[:, k].copy()
            if word != 'AR':
                v[readable] = np.degrees(np.unwrap(np.radians(v[readable])))
            last = 0.0
            for i in range(len(v)):
                last = v[i] if readable[i] else last
                v[i] = last
            values[:, k] = v
    return values


def leg(joints):
    """m, the rig's leg: its left thigh and shank at the first frame."""
    import numpy as np
    J = {k: np.asarray(joints[k], float)[0] for k in ('LeftUpLeg', 'LeftLeg', 'LeftFoot')}
    return float(np.linalg.norm(J['LeftLeg'] - J['LeftUpLeg']) + np.linalg.norm(J['LeftFoot'] - J['LeftLeg']))


def _manifest():
    """Her manifest (`defrobot.read`): her name and her drives."""
    r = defrobot.read()
    if r is None:
        raise MachineError('no (defrobot ..) in her manifest')
    return r


def figured(degrees):
    """{rig joint: (1, 3) place} of her figure at {joint: degrees} (`figure.frames`), her pelvis 1 m up."""
    import numpy as np
    frames = figure.frames(degrees, (0.0, 1.0, 0.0), figure.IDENT)
    return {name: np.array([frames[seg][0]], float) for name, seg in SEGMENTS.items()}


def text(header, times, words, values):
    """A program's text: `header` {key: value} lines, then a move a row of `values` at `times` s."""
    lines = ['; %s %s' % kv for kv in header.items()]
    for t, row in zip(times, values):
        lines.append('G1 T%.3f %s' % (t, ' '.join('%s%.3f' % (w, v) for w, v in zip(words, row))))
    return '\n'.join(lines) + '\n'


def _word(token):
    """(word, value) of a move's token: its letters and underscores, then its number."""
    k = len(token.rstrip('-0123456789.'))
    return token[:k], float(token[k:])


def parse(source):
    """(header, times (n,), words, values (n, w)) of a program's text."""
    import numpy as np
    header, times, rows, words = {}, [], [], ()
    for line in source.splitlines():
        if line.startswith(';'):
            key, _sp, value = line[1:].strip().partition(' ')
            header[key] = value
        elif line.startswith('G1 T'):
            tokens = line.split()
            named = [_word(tok) for tok in tokens[2:]]
            times.append(float(tokens[1][1:]))
            words = words or tuple(w for w, _v in named)
            rows.append([v for _w, v in named])
    return header, np.array(times, float), words, np.array(rows, float).reshape(len(rows), -1)


def read(path):
    """`parse` of the program at `path`."""
    with open(path, encoding='utf-8') as f:
        return parse(f.read())


def calibrated():
    """({word: (her joint, deg a word's degree)}, the rig's words at her base pose, that pose): the word each of
    her driven joints moves most on her figure (`figured`, `rig`) from her stand bent BASE and its gain; a word two
    joints move is the stronger's."""
    import numpy as np
    base = dict(figure.stand(), **BASE)
    at = rig(figured(base))[0]
    driven = [j for j, d in _manifest()['drives'].items() if d.get('speed') is not None and j in figure.JOINTS]
    table = {}
    for j in driven:
        moved = dict(base)
        moved[j] = base[j] + CALIBRATE
        gain = (rig(figured(moved))[0] - at) / CALIBRATE
        k = int(np.argmax(np.abs(gain[6:]))) + 6                     # a joint turns no pelvis place or turn
        if abs(gain[k]) >= GAIN and abs(gain[k]) > abs(table.get(WORDS[k], (None, 0.0))[1]):
            table[WORDS[k]] = (j, float(gain[k]))
    return table, at, base


def limits():
    """({joint: (lo, hi) deg or None}, {joint: deg/s}) of her drives (`defrobot`): each's range and the speed
    its motor's rpm through its ratio gives."""
    drives = _manifest()['drives']
    return ({j: tuple(d['range']) if d.get('range') else None for j, d in drives.items()},
            {j: 6.0 * float(d['speed']) / float(d['ratio']) for j, d in drives.items() if d.get('speed') is not None})


def hers(header, times, values):
    """(header, times s, her joints, (n, k) degrees) of the general rig's `values` (WORDS) at `times` converted
    to her: each word's joint by `calibrated`, the times by dynamic similarity, the degrees within her ranges,
    the moves within her speeds; the header carries the rig, her leg, what was dropped, clamped, the worst move's
    slowing and the whole's stretch."""
    import numpy as np
    table, at, base = calibrated()
    ranges, speeds = limits()
    mine = leg(figured(base))
    scale = math.sqrt(mine / float(header['leg'])) if header.get('leg') else 1.0
    t = np.asarray(times, float) * scale
    words = [w for w in WORDS if w in table]
    joints = tuple(table[w][0] for w in words)
    deg = np.stack([base[table[w][0]] + (values[:, WORDS.index(w)] - at[WORDS.index(w)]) / table[w][1]
                    for w in words], axis=1)
    clamped = {}
    for k, j in enumerate(joints):
        rng = ranges.get(j)
        if rng:
            lo, hi = rng
            over = max(float(np.max(lo - deg[:, k])), float(np.max(deg[:, k] - hi)), 0.0)
            if over > 0.0:
                clamped[j] = over
            deg[:, k] = np.clip(deg[:, k], lo, hi)
    slow = np.ones(len(t))
    if len(t) > 1:
        dt = np.maximum(np.diff(t), 1e-9)
        for k, j in enumerate(joints):
            slow[1:] = np.maximum(slow[1:], np.abs(np.diff(deg[:, k])) / speeds[j] / dt)
        t = np.concatenate([[t[0]], t[0] + np.cumsum(np.diff(t) * slow[1:])])
    out = dict(header, rig=_manifest().get('name', 'hers'), general=header.get('rig', 'general'), leg='%.3f' % mine,
               similar='%.3f' % scale, dropped=' '.join(w for w in WORDS[6:] if w not in table) or 'none',
               clamped=' '.join('%s:%.0f' % kv for kv in clamped.items()) or 'none', slowed='%.2f' % float(slow.max()),
               stretched='%.2f' % (float(t[-1]) / float(max(times[-1] * scale, 1e-9)) if len(t) > 1 else 1.0))
    return out, t, joints, deg


def beats(times, joints, degrees):
    """[(name, s, {joint: deg})] a move a beat, as `machine.process.joints` plays them: the first over its own
    time from wherever she is."""
    out, last = [], 0.0
    for k, (t, row) in enumerate(zip(times, degrees)):
        out.append(('g%d' % k, max(float(t) - last, 1e-3), {j: float(v) for j, v in zip(joints, row)}))
        last = float(t)
    return out


def program(path):
    """Her program at `path` as beats (`beats`): its words her joints."""
    _header, times, words, values = read(path)
    return beats(times, words, values)
