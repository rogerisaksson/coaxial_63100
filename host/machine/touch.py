"""What her skin and her soles feel: a touch that gates her going, a shove walked out, an edge.

    touch.step(g, out, dt)                  # a pass of her motion, `g` and `out` as `machine.motion`'s
    touch.stepped(y_in, vel, y, fr, a, st, out, rest, state, soled, asked, twist, tw_on, heading, dt)
    touch.soles(b, touch, ids, tips, y, fr, stance)   # its arithmetic of her soles
    touch.against(g, twist)                 # a twist less its part toward what she felt

Her law's own reading of the edge's touch (`machine.edge`): no body is named, nothing but forces
on her segments in their own frames, turned by her own model. The filter's memory is one array
(`laid`, STATE): what she felt and where, her hold, her brake, her stand, her drive, each sole's
clearance.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import edge, figure

#: Her segments under her knees, a side's: what bears on the floor, or trips over what is on it.
SOLES = ((edge.SKIN.index('left_foot'), edge.SKIN.index('left_toes')),
         (edge.SKIN.index('right_foot'), edge.SKIN.index('right_toes')))
ARMS = (tuple(edge.SKIN.index(n) for n in ('left_upper_arm', 'left_forearm', 'left_hand', 'left_fingers')),
        tuple(edge.SKIN.index(n) for n in ('right_upper_arm', 'right_forearm', 'right_hand', 'right_fingers')))
CORE = [edge.SKIN.index(n) for n in ('pelvis', 'torso', 'neck', 'head')]
ABOVE = sorted(set(range(len(edge.SKIN))) - set(SOLES[0] + SOLES[1] + ARMS[0] + ARMS[1]))


#: A touch above her soles not along her going - a wall, a tree, a hand that shoves her back or
#: aside - gates it: she brakes and stands, steps where her capture point asks, and goes on
#: as she was told RESUME_S after it is gone; one within ALONG_COS of her going she walks out.
#: Walking at 0.69 m/s an 80 N shove felled her 5 ways of 8 going on; stopped 0.1 s after it, 3 -
#: those along her way among them.
RESUME_S, ALONG_COS = 1.5, math.cos(math.radians(50.0))
#: What she felt stays a while: her going toward where it pushed her from is cut for FELT_S after.
FELT_S = 3.0

#: A sole meets an edge EDGE_M and more over the floor her bearing soles stand on: walking a flat
#: floor her swinging soles were touched along it 7.9 mm over it at most - a landing's friction
#: 544 N -, the 35 mm sill's face 14 to 38 mm up. Its lowest point is its toes' tip as much
#: as its corners (TIPS, in the toes' frame): turning, toes scuffing the floor at 0.6 to 6.6 mm
#: read 17 to 35 mm by the corners alone, tripped her on a flat floor.
EDGE_M = 0.01
TIPS = tuple((x, -figure.TOE_RY, figure.TOE_M) for x in (-figure.SOLE_HALF, figure.SOLE_HALF))

#: The filter's state: what she felt, along and across (m of FELT_S's reach left); her hold's
#: seconds; her brake, her stand and her drive, 0 to 1; each sole's clearance.
FELT, HOLD_S, U_BRAKE, U_STAND, U_DRV, CLEAR = slice(0, 2), 2, 3, 4, 5, slice(6, 8)
STATE = CLEAR.stop
#: At rest: nothing felt, no hold, no brake, standing, her drive whole, nothing to clear.
REST = (0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def laid():
    """The filter's state at rest, (STATE,)."""
    return np.array(REST)


def step(g, out, dt):
    """Her skin's and her soles' contact filter on `out` and `g['a']` this pass, her twist told
    back: `stepped` in C on her arrays (`g['touched_step']`, `machine.motion.laid`)."""
    g['twist'] = g['touched_step'](g.get('asked', (0.0, 0.0, 0.0)), g['twist'], float(g.get('_twist_active', False)),
                                   float(g['heading']), dt, g['est']['vel'])


def stepped(y_in, vel, y, fr, a, st, out, rest, state, soled, asked, twist, tw_on, heading, dt):
    """A pass of the filter on her edge's array `y_in` and her estimate's velocity `vel`, modal's
    `y`, `fr` and `a`, her latent `st` and its `out`, her `rest`: `out`, `a` and `state` (`laid`)
    stepped `dt` s in place, her twist returned - `asked` the twist told, `twist` hers now, `tw_on`
    whether one is told, `heading` hers; `soled` her soles as `soles` has them. In C bit for bit
    (`step`)."""
    from machine import latent, modal, ortho
    touch = y_in[edge.TOUCH].reshape(-1, 3)
    pushed, sizes, lowest, floors, _frames = soled
    f = np.asarray(pushed_core(soled, touch)).sum(axis=0)
    c_max = float(np.linalg.norm(touch[CORE], axis=1).max())
    af = min(1.0, max(0.0, (c_max - edge.TOUCHED_N) / edge.TOUCHED_N))
    fx, fz = float(f[0]), float(f[2])
    ox, oz = state[FELT]
    rem = math.hypot(ox, oz)
    rem_d = max(0.0, rem - dt)
    ox_d, oz_d = ox * (rem_d / max(rem, 1e-9)), oz * (rem_d / max(rem, 1e-9))
    fn, vn = math.hypot(fx, fz), math.hypot(float(vel[0]), float(vel[2]))
    nx, nz = fx / max(fn, 1e-9), fz / max(fn, 1e-9)
    state[FELT] = ((1.0 - af) * ox_d + af * FELT_S * nx, (1.0 - af) * oz_d + af * FELT_S * nz)
    cv = (fx * float(vel[0]) + fz * float(vel[2])) / max(fn * vn, 1e-9)
    drv0 = abs(float(out[latent.SWAY][modal.DRIVE]))
    w_opp = (af * min(1.0, max(0.0, (ALONG_COS - cv) / 0.15))
             * min(1.0, max(0.0, (drv0 - 0.02) / 0.04)) * (1.0 - float(a[modal.BORNE])))
    a[modal.GOING] *= 1.0 - w_opp
    tw = _against(tuple(state[FELT]), heading, asked)
    twist = tuple(tw_on * tw[i] + (1.0 - tw_on) * twist[i] for i in range(3))
    blocked = tw_on * min(1.0, max(0.0, (math.hypot(asked[0], asked[1]) - math.hypot(tw[0], tw[1])) / 0.05))
    gain_on = min(1.0, max(0.0, (max(float(st[latent.NOW][latent.GAIN]), float(st[latent.TO][latent.GAIN])) - 0.02) / 0.02))
    hold = gain_on * max(max(0.0, float(state[HOLD_S]) - dt), blocked * 0.5, w_opp * RESUME_S)
    state[HOLD_S] = hold
    u_act = min(1.0, hold / 0.01)
    u_b = min(1.0, max(0.0, float(state[U_BRAKE]) + (2.0 * u_act - 1.0) * (dt / (0.50 * u_act + 0.65 * (1.0 - u_act)))))
    need = float(a[modal.NEED]) + float(a[modal.SWINGS].max())
    u_set = u_act * min(1.0, max(0.0, (u_b - 0.95) / 0.05)) * min(1.0, max(0.0, 1.0 - need / 0.05))
    u_s = min(1.0, max(0.0, float(state[U_STAND]) + u_set * (dt / 0.40) - (1.0 - u_act) * (dt / 0.20)))
    state[U_BRAKE], state[U_STAND] = u_b, u_s
    sway, turns, psi = out[latent.SWAY].copy(), float(out[latent.TURNS]), out[latent.PSI]
    z0 = out[latent.REF] - sway * np.sin(psi)
    zb = np.zeros(ortho.N)
    zb[modal.AXIAL] = 0.04
    z_new = (1.0 - u_b) * z0 + u_b * (1.0 - u_s) * zb + u_b * u_s * rest[latent.Z]
    sway_b = np.zeros(ortho.N)
    sway_b[modal.LATERAL] = 0.12 * (1.0 - u_s)
    sway_b[modal.LIFT] = float(sway[modal.LIFT]) * (1.0 - u_s)
    sway_new = (1.0 - u_b) * sway + u_b * sway_b
    u_drv = min(1.0, max(0.0, float(state[U_DRV]) * (1.0 - w_opp) + (1.0 - u_act) * (dt / 0.65) - u_act * (dt / 0.50)))
    state[U_DRV] = u_drv
    sway_new[modal.DRIVE] = u_drv * float(sway[modal.DRIVE]) + (1.0 - u_drv) * min(float(sway[modal.DRIVE]), 0.04) * (1.0 - u_b)
    sine_new = sway_new * np.sin(psi)
    out[latent.REF] = z_new + sine_new
    out[latent.RATE] = (1.0 - u_b) * (out[latent.RATE] - sway * turns * np.cos(psi)) + sway_new * turns * np.cos(psi)
    out[latent.ACC] = -sine_new * turns * turns
    out[latent.SWAY] = sway_new
    kb, db = out[latent.KS].copy(), out[latent.DS].copy()
    kb[modal.AXIAL], kb[modal.LATERAL], kb[modal.SAGITTAL], kb[modal.ROCK] = 3000.0, 3000.0, 0.0, 2400.0
    db[modal.AXIAL], db[modal.LATERAL], db[modal.SAGITTAL], db[modal.ROCK] = 800.0, 600.0, 260.0, 240.0
    out[latent.KS] = (1.0 - u_b) * out[latent.KS] + u_b * (1.0 - u_s) * kb + u_b * u_s * rest[latent.K]
    out[latent.DS] = (1.0 - u_b) * out[latent.DS] + u_b * (1.0 - u_s) * db + u_b * u_s * rest[latent.D]
    aloft = np.clip(1.0 - a[modal.STANCE], 0.0, 1.0)
    clear = state[CLEAR].copy()
    up = np.minimum(y[modal.LOWEST], lowest) - floors
    w_up = np.clip((up - EDGE_M) / 0.004, 0.0, 1.0) * np.maximum(up, 0.0)
    p_arr, s_arr = np.asarray(pushed), np.asarray(sizes)
    w_edge = (np.clip((s_arr - edge.TOUCHED_N) / edge.TOUCHED_N, 0.0, 1.0)
              * np.clip((np.hypot(p_arr[:, :, 0], p_arr[:, :, 2]) - np.abs(p_arr[:, :, 1])) / edge.TOUCHED_N, 0.0, 1.0)).max(axis=1)
    a[modal.RAISE] = max(float(a[modal.RAISE]), float((clear * w_edge * w_up).max()))
    unfelt = np.clip(1.0 - s_arr.max(axis=1) / edge.TOUCHED_N, 0.0, 1.0)
    state[CLEAR] = aloft * np.maximum(clear, unfelt)
    return twist


def pushed_core(soled, touch):
    """Her core's touch turned out of its segments by the frames `soled` carries: `soles`'s
    fifth, (4, 3)."""
    return (soled[4].reshape(-1, 3, 3) @ touch[CORE][..., None]).squeeze(-1)


def soles(b, touch, ids, tips, y, fr, stance):
    """`stepped`'s arithmetic of her soles and her core on her law's body `b` as `wbc.sense` left
    it: (each sole's pushes, world - its foot's and its toes' -, their sizes, N; its toes' lowest
    tip and the floor under it, m; her core's segments' frames (4, 9)) - `touch` the edge's (SKIN,
    3), `ids` her segments' bodies, `tips` TIPS."""
    from machine import modal
    d = b.d
    pushed = [[d.xmat[ids[k]].reshape(3, 3) @ touch[k] for k in segs] for segs in SOLES]
    sizes = [[np.linalg.norm(p) for p in side] for side in pushed]
    lowest = [(d.xpos[ids[segs[1]]] + tips @ d.xmat[ids[segs[1]]].reshape(3, 3).T)[:, 1].min() for segs in SOLES]
    return pushed, sizes, lowest, modal.under(y, fr, stance)[0], d.xmat[ids[CORE]].copy()


def against(g, tw):
    """`tw` (body frame) less any axis aimed into what she felt (`g['touched']`): a wall walked
    into, a hand that shoved her - she does not walk on into it."""
    return _against(tuple(g['touched'][FELT]), float(g['heading']), tw)


def _against(felt, heading, tw):
    ox, oz = felt
    rem = math.hypot(ox, oz)
    w = min(1.0, rem / 0.10)
    nx, nz, sh, ch = ox / max(rem, 1e-9), oz / max(rem, 1e-9), math.sin(heading), math.cos(heading)
    cx, cy = sh * nx + ch * nz, ch * nx - sh * nz
    kx = 1.0 - w * (1.0 - min(1.0, max(0.0, (math.copysign(cx, tw[0]) + ALONG_COS) / (1.0 - ALONG_COS))))
    ky = 1.0 - w * (1.0 - min(1.0, max(0.0, (math.copysign(cy, tw[1]) + ALONG_COS) / (1.0 - ALONG_COS))))
    return (kx * tw[0], ky * tw[1], tw[2])
