"""The rotor observer's heat: winding, watts, headroom, SOA bars, the policy word."""
import math
import time

from rich.text import Text

from coaxial.devices.thermal import THROTTLE_AT
from coaxial.draw import cross_section
from coaxial.draw.gauges import (margin_class as soa_class, temp_share,
                                 thermometer_class as ntc_class)
from coaxial.model import thermal as _thermal
from coaxial.simulated.thermal.observer import SimulatedThermal
from motor import pmsm
from terminal.views.rotor.layout import BAR_CELLS, BAR_GLYPH, BOARD_NODES, SOA_NODES
from terminal.views.rotor.motions import LOAD_PEAK_A


#: The power face: (W / WATTS_SCALE) ^ p, p set so WATTS_MID is half the bar:
#: 20 W a tenth, 100 W 22 %, 500 W half, 2 kW full, red past it. Linear hid
#: the page's 97 W peak in two cells; logarithmic put it at 60 %. The figure
#: beside the bar is the watts, undistorted.
WATTS_SCALE = 2000.0

WATTS_MID = 500.0

#: Where the headroom gauge stops being green. The scale's own, not the
#: board's - see `headroom_class`.
HEADROOM_AMBER = 0.5

TRACK_GLYPH = chr(0x2812)

#: The identification's states as the foot says them (the bench's
#: abbreviations), in the margin's inks.
POLICY_WORD = {'STABLE': 'STABLE', 'CONVERGING': 'CONV',
               'UNCERTAIN': 'UNCR'}

POLICY_SHORT = {'STABLE': 'STBL'}

POLICY_INK = {'STABLE': cross_section.SOA_OK, 'CONVERGING': cross_section.SOA_WARN,
              'UNCERTAIN': cross_section.SOA_TRIP}


def _policy(view):
    """`(label, word, ink)` for the foot: TH OBS and the state with the
    margin the board acts on now - `UNCR 80%`, `CONV 93%`, `STBL 97%`,
    `STABLE` once whole (continuous since 2026-09-06) - or a grey dash
    before op 10 answers.
    """
    ident = view.get('ident')
    state = ident['state'] if ident else None
    if state in POLICY_INK:
        word, ink = _policy_word(ident, state)
        return 'TH OBS', word, ink
    return 'TH OBS', '-', cross_section.LEADER_GREY


def _policy_word(ident, state):
    """The state's word with the margin's percent, or the trip's."""
    margin = ident.get('margin', 1.0)
    percent = int(round(100.0 * margin))
    cap = ident.get('trip_cap', 1.0)
    floor = ident.get('margin_floor', _thermal.IDENT_MARGIN_FLOOR)
    if cap < 1.0 and abs(margin - cap) < 1e-6 and margin < floor - 1e-6:
        return 'TRIP %d%%' % percent, cross_section.INK[cross_section.SOA_TRIP]
    word = POLICY_WORD[state]
    if percent < 100:
        word = '%s %d%%' % (POLICY_SHORT.get(state, word), percent)
    return word, cross_section.INK[POLICY_INK[state]]


def winding(view):
    """The winding's temperature, estimated, degrees C."""

    budget = view.get('budget') or {}
    if 'winding_c' in budget:
        view['winding'] = budget['winding_c']
        view['winding_at'] = time.monotonic()
        return view['winding']
    now = time.monotonic()
    was, view['winding_at'] = view.get('winding_at'), now
    params = view['params']
    r_phase = params.get('motor_r') or 0.0
    k = params.get('winding_k_per_w') or pmsm.WINDING_K_PER_W
    heat = params.get('winding_j_per_k') or pmsm.WINDING_J_PER_K
    s = view['state']
    amps_rms = math.hypot(s['id'], s['iq']) / math.sqrt(2.0)
    target = _thermal.AMBIENT + 3.0 * amps_rms * amps_rms * r_phase * k
    if was is None:
        view['winding'] = _thermal.AMBIENT
        return view['winding']
    # The stand-in's haste, and only there: the real constant is ~7 min.
    tau = max(1e-3, k * heat) / (SimulatedThermal.HASTE
                                if view['simulated'] else 1.0)
    view['winding'] += (target - view['winding']) * min(1.0, (now - was) / tau)
    return view['winding']


def watts(view):
    """What the stage is putting into the motor, electrical, watts."""
    s = view['state']
    return 1.5 * (s['vd'] * s['id'] + s['vq'] * s['iq'])


def watts_bar(view):
    """The power as a fifth bar past the board's four, `(share, class)`."""
    return watts_share(watts(view))


def watts_share(w):
    """Watts to `(share, class)` on the power face - `WATTS_SCALE` has the
    shape and why.
    """
    power = math.log(0.5) / math.log(WATTS_MID / WATTS_SCALE)
    share = (abs(w) / WATTS_SCALE) ** power
    if share > 1.0:
        return 1.0, cross_section.SOA_TRIP
    return share, cross_section.WATTS


def headroom(view):
    """What is left of the whole board's thermal budget, 0 to 1."""
    worst = (view.get('budget') or {}).get('worst')
    return 1.0 - min(1.0, max(0.0, worst)) if worst is not None else 1.0


def envelope_acting(view):
    """Whether the board is holding the stage back - throttling, or tripped."""
    budget = view.get('budget') or {}
    return bool(budget.get('throttling') or budget.get('tripped'))


#: The margin tube's pulse while the envelope acts, Hz; only the colour
#: pulses, the level stays readable. 3 read as an emergency.
FLASH_HZ = 1.5


def flashing(view):
    """Whether this frame takes the bright half of the alarm pulse."""
    if not envelope_acting(view):
        return False
    return (time.monotonic() * FLASH_HZ * 2.0) % 2.0 < 1.0


def ntc_bar(view):
    """The thermistor as a tube, on the same scale as every other."""
    seen = (view.get('thermal') or {}).get('ntc')
    if seen is None:
        return []
    return [(temp_share(seen), ntc_class(seen))]


def switch_headroom(view):
    """What is left of the switches' budget, 0 to 1: the worst of the six
    nodes a duty cycle drives (SOA_NODES), each against its own ceiling.
    """
    used = (view.get('budget') or {}).get('used') or {}
    shares = [used[n] for n in SOA_NODES if n in used]
    if not shares:
        return headroom(view)
    return 1.0 - min(1.0, max(0.0, max(shares)))


def headrooms(view):
    """The two margins as gutter tubes: the switches', then the motor's."""
    switch = switch_headroom(view)
    motor = motor_headroom(view)
    budget = view.get('budget') or {}
    # The level is what is spent, not what is left.
    margin = policy_margin(view)
    motor_spent = 1.0 - motor
    if 'winding_used' in budget:
        motor_spent *= margin       # the board's winding, under its policy
    return [((1.0 - switch) * margin, cross_section.SOA_FLASH if flashing(view)
             else headroom_class(switch)),
            (motor_spent, cross_section.SOA_FLASH if motor_flashing(view)
             else headroom_class(motor))]


def policy_margin(view):
    """What the identification's state leaves of every ceiling's span -
    the board's number off op 10, one when it has not answered."""
    return float((view.get('ident') or {}).get('margin', 1.0))


def motor_flashing(view):
    """The motor's pulse: the board holding the stage back for the winding -
    its own factor under one, or its ceiling reached - since MINOR 12
    made it a node the envelope acts on.
    """
    budget = view.get('budget') or {}
    acting = (budget.get('winding_derate', 1.0) < 1.0
              or budget.get('winding_used', 0.0) >= 1.0)
    if not acting:
        return False
    return (time.monotonic() * FLASH_HZ * 2.0) % 2.0 < 1.0


def motor_headroom(view):
    """What is left of the winding's scale, 0 to 1."""
    budget = view.get('budget') or {}
    if 'winding_used' in budget:
        return max(0.0, 1.0 - budget['winding_used'])
    return motor_headroom_of(winding(view))


def motor_headroom_of(celsius):
    """The same margin from a temperature alone."""
    return 1.0 - temp_share(celsius)


def headroom_class(left):
    """The headroom gauge's colour: green, then amber, then red."""
    if left <= 1.0 - THROTTLE_AT:
        return cross_section.SOA_TRIP
    return cross_section.SOA_WARN if left <= HEADROOM_AMBER else cross_section.SOA_OK


def soa_bars(view, names):
    """`(fraction, class)` per node: height is heat, colour is margin."""
    budget = view.get('budget') or {}
    used = budget.get('used') or {}
    seen = view.get('thermal') or {}
    nodes = seen.get('nodes') or {}
    tripped = bool(budget.get('tripped'))
    out = []
    for name in names:
        if name not in used or nodes.get(name) is None:
            continue
        share = temp_share(nodes[name])
        out.append((max(0.0, min(1.0, share)),
                    soa_class(used[name], tripped)))
    return out


def soa_bar(share, tripped=False):
    """One node's margin as a bar, in the same ink the gutters use."""
    share = max(0.0, min(1.0, share))
    ink = cross_section.INK[soa_class(share, tripped)]
    bar = Text()
    bar.append(BAR_GLYPH * max(1, int(share * BAR_CELLS + 0.5)),
               style='color(%d)' % ink)
    # The rest of the tube.
    bar.append(TRACK_GLYPH * (BAR_CELLS - len(bar.plain)),
               style='color(%d)' % cross_section.INK[cross_section.TRACK])
    return bar


def thermal_rows(view):
    """The six nodes that carry the current, as bars against their ceilings."""

    th, budget = view.get('thermal'), view.get('budget')
    if not th:
        return ['  (not read yet)']
    used = (budget or {}).get('used') or {}
    degrees = th.get('nodes') or {}
    tripped = bool((budget or {}).get('tripped'))
    rows = []
    for node in SOA_NODES:
        if node not in used:
            continue
        bar = soa_bar(used[node], tripped)
        bar.append('%3.0f%% %5.1fC' % (100.0 * used[node],
                                       degrees.get(node, float('nan'))))
        rows.append((_thermal.pretty(node), bar))
    for node in BOARD_NODES:
        if node in used:
            rows.append((node, soa_bar(used[node], tripped).append(
                '%3.0f%% %5.1fC' % (100.0 * used[node],
                                    degrees.get(node, float('nan'))))
                or None))
    rows.append(('headroom', '%9.0f %% left, worst %s'
                 % (100.0 * headroom(view),
                    (budget or {}).get('worst_node', '?'))))
    factor = (budget or {}).get('derate')
    if factor is not None:
        rows.append(('throttle', Text(' %3.0f %% of the clamp ' % (100 * factor),
                                      style='chip.live' if factor > 0.99
                                      else 'chip.sim' if factor > 0.0
                                      else 'alarm')))
    soak = (budget or {}).get('soak_j') or {}
    worst_node = (budget or {}).get('worst_node')
    if worst_node in soak:
        rows.append(('soak', '%9.1f J left in %s' % (soak[worst_node],
                                                     worst_node)))
    # The two gauges along the foot, named in the order they lie there.
    rows.append(('winding', '%9.1f C %s, upper foot bar'
                 % (winding(view),
                    'board' if 'winding_c' in (view.get('budget') or {})
                    else 'est')))
    rows.append(('power', '%9.1f W of %.0f log, lower' % (watts(view),
                                                          WATTS_SCALE)))
    if view['load']:
        rows.append(('load loop', '%9.1f A of %.0f, %s'
                     % (view['load_amps'], LOAD_PEAK_A,
                        'rising' if view['load_rising'] else 'falling')))
    rows.append(('NTC', '%7.1f C' % th['ntc'] if th.get('ntc') is not None
                 else '%7s' % 'unread'))
    # The room as identified (the board has no sensor for it), and on the
    # stand-in the truth's, so the tour reads off this page too.
    ident = view.get('ident') or {}
    if ident.get('ambient') is not None:
        truth = ident.get('truth') or {}
        rows.append(('room', '%7.1f C identified%s'
                     % (ident['ambient'],
                        '  sim %s %.0f' % (truth['situation'], truth['ambient'])
                        if truth else '')))
    if budget:
        left = budget.get('seconds_to_limit')
        rows.append(('worst', '%-11s %3.0f%%%s'
                     % (budget.get('worst_node', '?'),
                        100.0 * budget['worst'],
                        '  %.0f s' % left if left is not None else '')))
    return rows
