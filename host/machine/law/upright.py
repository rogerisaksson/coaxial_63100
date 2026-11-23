"""control's weighing between its parts: the robot's seat sat upright, its arms' hold, its weights.

    seat, hold, w_neck, w_base, w_hb, w_hb2, pelvic = upright.weighs(g, s, told, ask, sw, w_hop)

`machine.law.motion.control` calls it as `g['upright']`, in C bit for bit (`tools.cores.upright`, wbc_upright.c).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.law import impedance, latent, modal, stride
from machine.moves import emancipation, leap
from machine.senses import observer
from machine.texts import ortho

#: The robot's arms' modes, its hold's (`ask['hold']`).
ARMED = np.array([modal.ARMS, modal.REACH, modal.ELBOWS])


def weighs(g, s, told, ask, sw, w_hop):
    """control's weighing between its parts on the robot's session `g`, its state `s`, what it is `told` and its
    `ask`: its seat sat upright, its arms' hold, its neck's, its base's and its hop's weights, and whether its
    pelvis bears."""
    st, out, pelvis = g['st'], g['out'], g['body'].pelvis
    bears = np.array(g['loads']) >= leap.TOUCH_N
    isch, pal = emancipation.SEAT_ROWS[0], latent.MANIFOLD.start + ortho.PALMAR
    pelvis_up = sum(float(c['f'][1]) * float(int(c.get('body', -1)) == pelvis) for c in s.get('contacts', ()))
    isch_w = min(1.0, float(told[isch]) / max(0.5 * float(told.sum()), observer.TINY))
    borne_w = min(1.0, pelvis_up / max(impedance.PROP_ON * float(g['fr'][modal.WEIGHT]), observer.TINY))
    pal_due = max(float(told[ortho.PALMAR]), float(st[latent.TO][pal]), float(g['tab'][:, pal].max(initial=0.0)))
    up = float((~bool(bears.all())) & (pal_due <= 0.0) & (float(ask.get('reclined', 0.0)) <= 0.5)
               & (float(st[latent.TO][latent.ZMP]) < 0.5) & (abs(float(out[latent.SLID])) <= 1e-6))
    hold = max(1.0, float((out[latent.KS][ARMED] / g['rest'][latent.K][ARMED]).max()))
    w_gait = min(1.0, max(0.0, (0.05 - float(st[latent.NOW][latent.GAIN])) / 0.05))
    w_drive = min(1.0, abs(float(st[latent.TO][latent.AMP][modal.DRIVE])) / stride.STIRS)
    foot = min(1.0, max(0.0, float(out[latent.HELD]) - 1.0))
    # a sole up while borne is no stride: counted one, lying back the robot's neck let go and flexed at 1034
    # deg/s
    rest = (1.0 - float(g['a'][modal.STRIDES]) * (1.0 - float(g['a'][modal.BORNE]))) * w_gait
    w_base = sw * max(w_hop, rest)
    # the robot's neck holds pitch through a brake and its blend into stand (`NOW[GAIN]`), and when seated
    g_on = max(float(st[latent.TO][latent.GAIN]), float(st[latent.NOW][latent.GAIN]))
    brake = foot * (1.0 - w_drive) * g_on
    w_neck = sw * max(w_hop, max(rest, brake))
    pelvic = any(int(c.get('body', -1)) == pelvis for c in s.get('contacts', ()))
    return float(isch_w * borne_w * up), hold, w_neck, w_base, w_hop * w_base, w_hop * w_base, pelvic
