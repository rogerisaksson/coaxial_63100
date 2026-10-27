"""A bearing patch's rows on her whole-body step: each contact held still, its split as felt.

Where it presses, up alone where it slides (`machine.wrench`); wbc_patch.c is it in C.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.borne import CARPAL
from machine.rounds import NU
from machine.wbc import CONTACT_K, SLIDE_M_S

#: A contact's point held still, m/s^2, while it presses - one the step would have pull is let go, its
#: point free: held still at nothing, a seat she rose from asked 5134 N of a sole; its push no more than
#: GROW_N over what it bears: let to plan 311 N where a mattress's soft edge bore 107, the push behind
#: her centre of mass never came and she rolled back onto the bed.
W_STILL = 1e4

_EYE3, _UP = np.eye(3), np.array([0.0, 1.0, 0.0])


def patch(cs, idx, nu, full, full0, v, slide, pin):
    """A bearing patch's rows, one (S, r, r0) a contact `cs` of it (its unknowns `idx` among the contacts): its
    point held still, up alone where it slides - its slip its velocity along the floor's plane (the force a patch
    feels leans into its push, 30 deg in a lean: no normal to measure across), past SLIDE_M_S and along the slide
    its text says it may (`slide`, world; none said, none slides: a hand walked into a wall slid by this measure
    and the wall threw her back 1.2 m; a strike's skid as she stood up flipped the answer for 6 ms): the regime
    the step feels, never assumes (given a drag the stuck seat never
    gave, a rise's lean planned -128 N along her thighs and felt +148) -; and the patch's normal force split
    as it is felt - each contact's, along the patch's normal, its share of the forces felt along it (returned,
    N) -, `pin` (W_PIN in shares of her weight) the row's, as a clipped force is held. Free, two thighs' split on
    one seat was the rounds' clips' to chatter over (63/65 -> 146/0 -> 4/4 N a pass): the step planned a roll
    the seat never gave and threw her off a chair laid 2e-4 rad off. Returns (rows, felt, each contact's
    (sliding, its slip along the world's x and z) for `rounds.pushed`)."""
    n = sum(c['f'] for c in cs)
    n = n / max(float(np.sqrt(n @ n)), 1e-9)
    felt = np.array([max(float(c['f'] @ n), 0.0) for c in cs])
    share = felt / felt.sum() if felt.sum() > 0.0 else np.full(len(cs), 1.0 / len(cs))
    rows, slips = [], []
    for m, (w, c) in enumerate(zip(share, cs)):
        split = np.zeros(nu)
        for k, j in enumerate(idx):
            split[NU + 3 * j:NU + 3 * j + 3] = n * (float(k == m) - w)
        jv = c['J'] @ v
        way = jv[[0, 2]]
        sliding = float((np.hypot(way[0], way[1]) > SLIDE_M_S) & (way @ slide[[0, 2]] > 0.0))
        mix = (1.0 - sliding) * _EYE3 + sliding * np.outer(_UP, _UP)
        left = -c['dJv'] - c['J'] @ full0
        rows.append((np.vstack([W_STILL * (mix @ (c['J'] @ full)), pin * split[None, :]]),
                     np.append(W_STILL * (mix @ (left - CONTACT_K * jv)), 0.0), np.append(W_STILL * (mix @ left), 0.0)))
        slips.append((sliding, way[0], way[1]))
    return rows, felt, slips


def patches(contacts, nu, full, full0, v, ask, pin):
    """(each contact's rows, its cap, N - its felt share of `fz_cap` over every patch, of `slid_cap` where it
    slides -, its slip (k, 3)) in the contacts' order
    (`patch`), a patch the contacts one surface bears (`borne`); a cap none, none. Capped a patch at a time, a
    thigh on the seat's front face let the seat plan 297 N under 214; capped sliding alone, the seat bore all of
    her through a rise's lean; the governor's cap on a stuck seat struck it. `pin` is the row's while a sole of
    hers stands (`wrench.step`): alone on what bears her the split is the step's to plan - pinned as felt it ran away,
    128/74/64 N on pelvis and thighs to 697 on her pelvis in 0.4 s, and threw her.
    Her hands' patches (`borne.CARPAL`) apart: `hand_cap`, their told share, over both as felt - each hand's alone,
    a hitch's hands took 0.84 of her and the seat let her go. `ask`'s `slide` the slide her contacts may take."""
    slide, cap = np.asarray(ask.get('slide', (0.0, 0.0, 0.0)), float), float(ask.get('fz_cap', np.inf))
    slid, hand, held = float(ask.get('slid_cap', np.inf)), float(ask.get('hand_cap', 0.0)), {}
    for idx in {c.get('patch', 0): [j for j, d in enumerate(contacts) if d.get('patch', 0) == c.get('patch', 0)] for c in contacts}.values():
        rows, felt, slips = patch([contacts[j] for j in idx], idx, nu, full, full0, v, slide, pin)
        held.update(zip(idx, zip(rows, felt, slips)))
    hands = np.array([float(c.get('patch', 0) >= CARPAL) for c in contacts])
    felt = np.array([held[j][1] for j in range(len(contacts))])
    shares = np.zeros(len(contacts))
    part = felt * (1.0 - hands)
    shares += part / part.sum() if part.sum() > 0.0 else (1.0 - hands) / max(float((1.0 - hands).sum()), 1.0)
    palm = felt * hands
    shares += palm / palm.sum() if palm.sum() > 0.0 else hands / max(float(hands.sum()), 1.0)
    slips = np.array([held[j][2] for j in range(len(contacts))]).reshape(len(contacts), 3)
    limit = np.where(hands > 0.0, hand, np.where(slips[:, 0] > 0.0, np.minimum(cap, slid), cap))
    caps = np.where(np.isfinite(limit), limit * shares, np.inf)
    return [held[j][0] for j in range(len(contacts))], caps, slips
