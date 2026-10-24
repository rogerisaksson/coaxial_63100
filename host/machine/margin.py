"""The insert's phase margin (`monitor-discrete-phase-margin`): her loop's return ratio, locked in.

    probe = margin.laid()                    # the text's probe frequencies and window, at rest
    d = margin.injected(probe, t, amps)      # the probe's sine at `t`, `amps` N m: its frequency's turn
    margin.step(probe, u, c, t, dt)          # a pass: the torque her drive was given (u), her loop's own (c)
    got = margin.read(probe)                 # {hz, gain_db, phase_deg, margin_deg}: L = -C / U at each
    kd, gamma = margin.derated(got)          # the text's rollback of K_d and the barrier's gamma

The loop broken at her drive's torque, the probe's sine injected there (u = c + d). Its frequencies
in turn, each DWELL_S; a quadrature lock-in on the one whose turn it is: u and c each times cos and
sin of 2 pi f t, each product through a first-order leak of the text's window (`demod-window-ms`),
taken as its turn ends - the four at once beat 7-8 Hz apart through that leak, 9 % of each in the
others'. L = -C / U the ratio of their phasors. Where |L| is past -3 dB - the loop at or past its
crossover - and 180 + arg L under the text's margin, K_d and gamma scaled down by the margin's share
of it, no further than their floors.
"""
import math

from machine import kernel

_PROBE = kernel.node('monitor-discrete-phase-margin', 'phase-margin')
#: The probe's frequencies, Hz; its demodulation's window, s; the margin it keeps, deg.
HZ = tuple(_PROBE['probe-frequencies-hz'])
WINDOW_S = 1e-3 * _PROBE['demod-window-ms']
MARGIN_DEG = _PROBE['min-phase-margin-deg']
#: A frequency's turn, s: its leak settled to e^-6 of a step.
DWELL_S = 6.0 * WINDOW_S
#: The floors K_d and gamma are rolled back to.
KD_FLOOR = _PROBE['on-margin-deficit']['min-kd-scale-floor']
GAMMA_FLOOR = _PROBE['on-margin-deficit']['min-gamma-scale-floor']
#: The gain past which the loop is at its crossover, dB.
NEAR_DB = -3.0


def laid():
    """A probe's state: the turning frequency's phasors of u and c (re, im each), each frequency's
    taken at its turn's end."""
    return {'u': [0.0, 0.0], 'c': [0.0, 0.0], 'turn': 0, 'taken': [None] * len(HZ)}


def _turn(t):
    return int(t // DWELL_S) % len(HZ)


def injected(probe, t, amps):
    """The probe's sine at `t` s, `amps` N m, at the frequency whose turn it is."""
    return amps * math.sin(2.0 * math.pi * HZ[_turn(t)] * t)


def step(probe, u, c, t, dt):
    """`probe` a pass of `dt` s on: `u` the torque her drive was given at `t`, `c` her loop's own; a
    turn ended, its phasors taken and the next's begun at rest."""
    k = _turn(t)
    if k != probe['turn']:
        probe['taken'][probe['turn']] = (complex(*probe['u']), complex(*probe['c']))
        probe['u'], probe['c'], probe['turn'] = [0.0, 0.0], [0.0, 0.0], k
    leak, w = min(1.0, dt / WINDOW_S), 2.0 * math.pi * HZ[k] * t
    co, si = math.cos(w), -math.sin(w)
    for p, x in ((probe['u'], u), (probe['c'], c)):
        p[0] += leak * (x * co - p[0])
        p[1] += leak * (x * si - p[1])


def read(probe):
    """[{hz, gain_db, phase_deg, margin_deg}] of each frequency taken: L = -C / U."""
    out = []
    for f, got in zip(HZ, probe['taken']):
        L = -got[1] / got[0] if got and abs(got[0]) > 0.0 else complex(0.0)
        phase = math.degrees(math.atan2(L.imag, L.real))
        out.append({'hz': f, 'gain_db': 20.0 * math.log10(max(abs(L), 1e-300)), 'phase_deg': phase,
                    'margin_deg': 180.0 + phase if phase <= 0.0 else phase - 180.0})
    return out


def derated(got):
    """(K_d's scale, gamma's): each the least over the probes at or past crossover of their margin's
    share of the text's, no lower than its floor; 1 where every margin holds."""
    share = min([max(0.0, g['margin_deg']) / MARGIN_DEG for g in got if g['gain_db'] > NEAR_DB] + [1.0])
    return max(KD_FLOOR, min(1.0, share)), max(GAMMA_FLOOR, min(1.0, share))
