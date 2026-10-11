"""Her leg's reach over its foot as static arrays: where its stops leave the hip, by its height.

    python tools/cores/reach.py --write     # machine/reach_rows.py
    laid = reach.rows()                     # what the file holds, laid anew from her figure

A hip's place in its foot's frame - across to the limb's own side, up, along the toes - is the
ankle's roll, the ankle's pitch and the knee, whatever the pelvis's turn: the hip's own three
joints take that, its yaw alone stopped. The roll tilts the leg's plane about the foot's length;
in that plane the hip stands `u` m up and `z` m along. Laid: for each `u` the run of `z` the
ankle's pitch's stroke (`linkage.RODS`) and the knee's stop (the compiled figure's) leave, BACK
to FORE - free, the leg's length alone bounding it, at -+(THIGH + SHANK); the roll's stroke
(`linkage.ROLLED`) as tangents; the hip's yaw's stops; the sole. `machine.reach` reads them,
tests/test_reach.py holds them to MuJoCo.

A run is the longest along its `u`: ahead of an island the ankle's stroke forbids (the hip
0.50-0.63 m up) the strip left under the leg's length is not in it. A row is the least of itself
and its neighbours: what is read between rows is reached.
"""
import importlib
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from machine import figure, linkage  # noqa: E402
from machine.gait import BALL, HEEL, HIP_DROP, HIP_HALF, SHANK, THIGH, TOE_M  # noqa: E402
from tools import REPO  # noqa: E402
from tools.cores import written  # noqa: E402

PATH = os.path.join(REPO, 'host', 'machine', 'reach_rows.py')
NOTICE = "Her leg's reach over its foot as static arrays: written by host/tools/cores/reach.py, not by hand."

#: The rows: the hip from U0 m up in its leg's plane, DU apart; a row scanned SCAN_M apart, its
#: run's ends bisected.
U0, DU, SCAN_M = 0.10, 0.005, 0.001
LENGTH = THIGH + SHANK


def stops():
    """{kind: (low, high) rad} of a leg's stopped joints: the hip's yaw and the knee as the figure
    compiles them, her two legs alike; the ankle's pitch and roll, their rods' strokes. A hip
    stopped in roll or pitch is not the foot's frame's to hold: it raises."""
    mujoco = importlib.import_module('mujoco')
    from machine import mjcf
    m = mujoco.MjModel.from_xml_string(mjcf.mjcf())
    out = {}
    for name in figure.LEG:
        each = [tuple(float(v) for v in m.joint(side + name).range)
                if m.jnt_limited[m.joint(side + name).id] else None for side in ('left', 'right')]
        if each[0] != each[1]:
            raise ValueError("%s: her legs' stops differ, %r" % (name[1:], each))
        if each[0]:
            out[name[1:]] = each[0]
    if 'hip_roll' in out or 'hip' in out:
        raise ValueError("a hip stopped in roll or pitch: its foot's frame does not hold it")
    roll = linkage.ROLLED['ankle']
    for kind, stroke in (('ankle', linkage.RODS['ankle'][1]), ('ankle_roll', (-roll, roll))):
        low, high = (math.radians(v) for v in stroke)
        out[kind] = (max(low, out[kind][0]), min(high, out[kind][1])) if kind in out else (low, high)
    return out


def bend(r):
    """(knee, ankle, ankle_roll) rad of her left leg, its hip at `r` in its foot's frame."""
    return figure.leg(1.0, (r[0] - HIP_HALF, r[1] + HIP_DROP, r[2]), figure.IDENT, (0.0, 0.0, 0.0),
                      figure.IDENT)[3:]


def held(r, limits):
    """Whether her left leg, its hip at `r` in its foot's frame, is within its length and
    `limits` (`stops`)."""
    return math.hypot(*r) <= LENGTH + 1e-12 and all(
        limits[k][0] <= q <= limits[k][1] for k, q in zip(('knee', 'ankle', 'ankle_roll'), bend(r)))


def run(u, limits):
    """(back, fore) m along the toes: the longest run her hip stands in, `u` m up in its leg's
    plane; an end the leg's length alone bounds, -+LENGTH."""
    edge = math.sqrt(LENGTH * LENGTH - u * u)
    n = int(edge / SCAN_M)
    ok = [held((0.0, u, k * SCAN_M), limits) for k in range(-n, n + 1)]
    best, k = (0, 0), 0
    while k < len(ok):
        j = k
        while j < len(ok) and ok[j]:
            j += 1
        best, k = max(best, (j - k, k)), j + 1
    count, first = best
    if not count:
        raise ValueError('nothing reached %g m up' % u)

    def end(k, step):
        # straight, the leg leans as its ankle is bent: past the ankle's stroke a strip
        # under the leg's length is out, micrometres to 3 mm deep - finer than the scan
        inside, outside = k * SCAN_M, (k + step) * SCAN_M if abs(k + step) <= n else step * edge
        if abs(k) == n and held((0.0, u, step * edge), limits):
            return math.copysign(LENGTH, step)
        for _ in range(10):
            mid = 0.5 * (inside + outside)
            inside, outside = (mid, outside) if held((0.0, u, mid), limits) else (inside, mid)
        return inside

    return end(first - n, -1), end(first + count - 1 - n, 1)


def sole(toes):
    """A sole's corners (across, along) m about its ankle, counter-clockwise: heel to ball, or
    on to its toes' tips; alike either side of its length."""
    _seg, _shape, (half, _hy, _hz), (x, _y, _z) = figure.CONTACTS[0]
    _seg, _shape, (toe_half, _ty, _tz), (toe_x, _y, _z) = figure.CONTACTS[1]
    if x or toe_x or (toes and toe_half != half):
        raise ValueError('a sole not alike either side: its mirror is not in the rows')
    front = BALL + (TOE_M if toes else 0.0)
    return ((-half, -HEEL), (half, -HEEL), (half, front), (-half, front))


def rows():
    """What machine/reach_rows.py holds, {name: value}."""
    limits = stops()
    count = int(round((LENGTH - U0) / DU))
    true = [run(U0 + DU * k, limits) for k in range(count)]
    # a free end beside a bounded one counts as the length's own circle there: a bound that
    # runs into the circle is read short of it between its last row and the first free
    edge = [math.sqrt(LENGTH * LENGTH - (U0 + DU * k) ** 2) for k in range(count)]
    near = [range(max(0, k - 1), min(count, k + 2)) for k in range(count)]
    back = [-LENGTH if all(true[j][0] <= -LENGTH for j in near[k]) else
            math.ceil(max(max(true[j][0], -edge[j]) for j in near[k]) * 1e4) / 1e4
            for k in range(count)]
    fore = [LENGTH if all(true[j][1] >= LENGTH for j in near[k]) else
            math.floor(min(min(true[j][1], edge[j]) for j in near[k]) * 1e4) / 1e4
            for k in range(count)]
    free = [b <= -LENGTH and f >= LENGTH for b, f in zip(back, fore)]
    top = free.index(True)
    if not all(free[top:]):
        raise ValueError("a bound over the rows' top, %g m up" % (U0 + DU * top))
    yaw, roll = limits['hip_yaw'], limits['ankle_roll']
    return {'LIMBS': (1.0, -1.0), 'THIGH': THIGH, 'SHANK': SHANK,
            'YAW': tuple(math.floor(abs(v) * 1e6) / 1e6 for v in yaw),
            'ACROSS': tuple(math.floor(math.tan(abs(v)) * 1e6) / 1e6 for v in roll),
            'U0': U0, 'DU': DU, 'BACK': tuple(back[:top + 1]), 'FORE': tuple(fore[:top + 1]),
            'SOLE': sole(False), 'FOOT': sole(True)}


def render(laid):
    """The module's text."""
    def flat(xs):
        return '(' + ', '.join('%g' % x for x in xs) + ')'

    def pairs(ps):
        return '(' + ', '.join('(%g, %g)' % p for p in ps) + ')'

    def wrapped(name, xs):
        out, line = [], '%s = (' % name
        for x in xs:
            if len(line) + len('%g, ' % x) > 99:
                out.append(line.rstrip())
                line = '    '
            line += '%g, ' % x
        return out + [line.rstrip(', ') + ')']

    return '\n'.join([
        '"""%s"""' % NOTICE, '',
        "#: Each limb's mirror, her left's and her right's: across is to a limb's own side.",
        'LIMBS = (%s)' % ', '.join('%.1f' % v for v in laid['LIMBS']),
        '#: Thigh and shank, m.',
        'THIGH, SHANK = %g, %g' % (laid['THIGH'], laid['SHANK']),
        "#: The hip's yaw's stops, rad: the foot turned in and out of the pelvis's heading.",
        'YAW = %s' % flat(laid['YAW']),
        "#: The ankle's roll's stroke as tangents: the hip across over its height, to the other",
        '#: side and to its own.',
        'ACROSS = %s' % flat(laid['ACROSS']),
        "#: In the leg's plane, the hip U0 + k DU m up: BACK[k] to FORE[k] m along the toes its",
        "#: ankle's pitch's stroke and its knee's stop leave; -+(THIGH + SHANK), its length alone.",
        'U0, DU = %g, %g' % (laid['U0'], laid['DU'])]
        + wrapped('BACK', laid['BACK']) + wrapped('FORE', laid['FORE']) + [
        "#: A sole heel to ball, a foot heel to its toes' tips: corners (across, along) m about",
        '#: the ankle, counter-clockwise.',
        'SOLE = %s' % pairs(laid['SOLE']),
        'FOOT = %s' % pairs(laid['FOOT'])]) + '\n'


if __name__ == '__main__':
    if '--write' not in sys.argv[1:]:
        sys.exit(__doc__)
    done = rows()
    written(PATH, render(done))
    print('%d rows written, the hip %.2f to %.2f m up' % (
        len(done['FORE']), done['U0'], done['U0'] + done['DU'] * (len(done['FORE']) - 1)))
