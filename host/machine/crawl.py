"""Her gaits off her feet: a twist's coronal index and turnout, a sled's share, its anchors' hold.

    kappa = crawl.coronal(twist)                  # 1: |vy| / |(vx, vy)| of floor twists [vx vy wz] (..., 3)
    turn = crawl.turnout(twist, most)             # rad: a hip's or shoulder's turnout for them, `most` sideways
    share = crawl.sled(phase, gamma)              # 1: her weight on the sled at an axial phase, the rest on anchors
    use = crawl.slip(phase, gamma, mu_a, mu_s)    # 1: the anchors' friction the sled's drag takes, 1 at its limit
    held = crawl.held(gamma, mu_a, mu_s)          # 1: of the vault's half turn, where the anchors hold
    Mbar, A, b = crawl.partitioned(M, h, free)    # her drives less those in High-Z: q''_f = A q''_a + b
    up = crawl.lifted(across, roll)               # m: the far hip a roll onto one buttock lifts
    gain = crawl.boost(hands, heel)               # 1: the dead leg's side's arm over the other's, no yaw

The kernel's terrestrial texts (machine/ortho_kernel/terrestrial_*.lisp) and its degraded scoot and crawl
(degraded_scoot_and_crawl_*.lisp); their 5-torus clock is `fours.clock`, n five. Flat arrays and arithmetic: no branch, no loop, no dict.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np


def coronal(twist):
    """1: floor twists' (..., 3) [vx vy wz] coronal index |vy| / |(vx, vy)| - 0 ahead, 1 sideways, 0 a spin
    (`align-hinge-joints-to-se2-vector`'s kappa_lat)."""
    twist = np.asarray(twist, float)
    return np.abs(twist[..., 1]) / np.maximum(np.hypot(twist[..., 0], twist[..., 1]), 1e-12)


def turnout(twist, most):
    """rad: a hip's or shoulder's turnout for floor twists `twist`, `most` at the coronal index 1."""
    return most * coronal(twist)


def sled(phase, gamma):
    """1: her weight on the sled at axial `phase` rad, 1 - gamma sin+ phase (`modulate-stick-slip-sled-ratchet`'s
    F_z,sled over m g); the anchors bear the rest."""
    return 1.0 - gamma * np.clip(np.sin(phase), 0.0, None)


def slip(phase, gamma, mu_anchor, mu_sled):
    """1: the anchors' static friction the sled's kinetic drag takes at axial `phase`, mu_sled share over mu_anchor
    (1 - share): over 1 the anchors slide."""
    share = sled(phase, gamma)
    return mu_sled * share / np.maximum(mu_anchor * (1.0 - share), 1e-300)


def held(gamma, mu_anchor, mu_sled):
    """1: of the vault's half turn, where the anchors hold the sled's drag (`slip` at most 1): sin phase at least
    mu_sled / (gamma (mu_anchor + mu_sled)), none where that passes 1."""
    least = mu_sled / (gamma * (mu_anchor + mu_sled))
    return 1.0 - 2.0 / np.pi * np.arcsin(np.clip(least, 0.0, 1.0))


def partitioned(M, h, free):
    """(her mass matrix's active block less what her freewheeling joints take, M_aa - M_af M_ff^-1 M_fa; the map
    and offset of their accelerations, q''_f = A q''_a + b, unforced) of her mass matrix `M` and bias `h` - a
    freewheeling drive's backdrive in it -, `free` 1 a joint in High-Z (`partition-fault-tolerant-dynamics`)."""
    free = np.asarray(free, bool)
    act = ~free
    taken = np.linalg.solve(M[np.ix_(free, free)], np.column_stack([M[np.ix_(free, act)], h[free]]))
    return M[np.ix_(act, act)] - M[np.ix_(act, free)] @ taken[:, :-1], -taken[:, :-1], -taken[:, -1]


def lifted(across, roll):
    """m: a hip `across` m from the buttock she rolls onto, lifted by `roll` rad (`escort-passive-towed-limb`)."""
    return across * np.sin(roll)


def boost(hands, heel):
    """1: the push of the arm on her dead leg's side over the other's that leaves her no yaw, her hands `hands` and
    her working heel `heel` m off her midline, the heel pushing as an arm (`nullify-asymmetric-propulsion-yaw`):
    a f_L - a f_R - b f_h = 0, f_h = f_R."""
    return 1.0 + heel / hands
