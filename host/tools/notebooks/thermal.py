"""Thermal: the node network, the envelope, the identification, a trip."""
from .parts import code, md, section

TITLE = 'Thermal'
SUMMARY = 'Twenty nodes on one thermometer: the fit, the board\'s estimate and budget, the envelope, the room identified, a trip.'

SECTIONS = [
    section(
        'The network, and how it was fitted',
        md('`coaxial.model.thermal` is the network `thermal/src/thermal.c` integrates: '
           'twenty nodes along the copper. Measured: `board_to_ambient` and '
           '`board_capacity` only.'),
        code('''from coaxial.model import thermal

print('ambient %.1f C assumed; board_to_ambient %.2f K/W at a %.0f K rise, '
      'board_capacity %.0f J/K, tau %.1f min'
      % (thermal.AMBIENT, thermal.CFG['board_to_ambient'], thermal.BOARD_CAL_RISE_K,
         thermal.CFG['board_capacity'], thermal.tau_minutes()))
print('%d nodes on %d edges: %d sources, %d of laminate, the hot swap, %d of motor'
      % (len(thermal.ALL_NODES), len(thermal.EDGES), len(thermal.NODES),
         len(thermal.LAMINATE), len(thermal.MOTOR)))
for node in thermal.NODES:
    print('%-12s to_board %5.1f K/W  capacity %.3f J/K'
          % (node, thermal.CFG['to_board'][node], thermal.CFG['capacity'][node]))'''),
        md('Camera, four states, each adding one power term, 25 min each (3.7 constants), '
           "dead soldermask as reference (2026-08-28). The NTC: 0.30 of the V leg's rise "
           '(geometry).'),
        code('''print(thermal.MEASURED)
for state in thermal.STATES:
    print('%-8s %s' % (state, thermal.STATE_IS[state]))
watts = sum(thermal.POWER_SWITCHING[n] for n in thermal.DRIVERS)
print('NTC_OFFSET        %.2f K over the board, passive' % thermal.NTC_OFFSET)
print('NTC_SEES_DRIVERS  %.2f of the V leg\\'s rise; the element lags %.0f s'
      % (thermal.NTC_SEES_DRIVERS, thermal.NTC_TAU_S))
print('drivers switching %.2f W over three legs = %.2f W a leg x %.0f K/W = %.1f K over the patch'
      % (watts, thermal.DRIVER_SWITCH_WATT, thermal.LEG_TO_BOARD, thermal.DRIVER_RISE_SWITCHING))
print('NTC while switching %.1f - %.1f - %.1f = %.1f K over the offset; the element accounts for '
      '%.2f x %.1f = %.2f K; residual %.2f K, a thermistor against a camera'
      % (thermal.MEASURED['switching']['ntc'], thermal.MEASURED['switching']['board'],
         thermal.NTC_OFFSET, thermal.MEASURED['switching']['ntc']
         - thermal.MEASURED['switching']['board'] - thermal.NTC_OFFSET,
         thermal.NTC_SEES_DRIVERS, thermal.DRIVER_RISE_SWITCHING,
         thermal.NTC_SEES_DRIVERS * thermal.DRIVER_RISE_SWITCHING,
         thermal.NTC_CAMPAIGN_RESIDUAL_K))'''),
        md('`calibrate`: `to_board = (T_zone - T_reference) / P_zone`, one division a node.'),
        code('''import math

passive_power = {'mcu': thermal.POWER_SWITCHING['mcu'],
                 'regulators': thermal.POWER_SWITCHING['regulators'] - watts}
camera_passive = {'mcu': 30.0 + 15.0, 'regulators': 30.0 + 8.0}    # state 1, dead soldermask 30.0 C
fit = thermal.calibrate(camera_passive, board_c=30.0, power=passive_power)
for node, k_per_w in sorted(fit.items()):
    print('%-12s %5.1f K/W from the camera, %5.1f in the graph'
          % (node, k_per_w, thermal.CFG['to_board'][node]))
bridge_lumped = (10.1 - 1.0) / watts                # the bridge zone, state 4 less state 1
camera_leg = 3.0 * bridge_lumped
sheet_leg = 25.9 - 0.69 - thermal.CFG['board_to_ambient']
print('bridge       %5.1f K/W lumped for three legs, %.1f a leg in parallel'
      % (bridge_lumped, camera_leg))
print('a leg        %.1f K/W off the camera against %.1f off the datasheet coupon; '
      'geometric mean %.1f, carried as %.0f'
      % (camera_leg, sheet_leg, math.sqrt(camera_leg * sheet_leg), thermal.LEG_TO_BOARD))'''),
        md('Switching dry, relaxed; drawn as the THERMAL OBSERVER page draws it, fixed '
           'scale.'),
        code('''from coaxial.draw import ansi, thermalmap

steady = thermal.steady(thermal.POWER_SWITCHING)
print('power while switching, dry: %.2f W' % sum(thermal.POWER_SWITCHING.values()))
for node in thermal.ALL_NODES:
    print('%-12s %6.2f C' % (node, steady[node]))
ntc_steady = thermal.expected_ntc(steady['board'], steady[thermal.NTC_PATCH] - steady['board'])
print('NTC expected %.2f C between the centre and the V patch' % ntc_steady)
print('settled: ' + ', '.join('%d min %.0f %%' % (m, 100 * thermal.settled_fraction(m))
                              for m in (1, 5, 10, 25)))
ansi.image(thermalmap.render({n: steady[n] for n in thermal.NODES}, steady['board'],
                             cells=60, colour=True, title='steady state, switching dry'))'''),
    ),
    section(
        'The board\'s own estimate, and its budget',
        md('`state()`: nodes in C and `error` (expected NTC - measured). `ntc` None while '
           "AFE_ON is low. `budget()`: `used`, the fraction of each node's span to its "
           'record ceiling.'),
        code('''observer = device.thermal
st = observer.state()
print('NTC %s C  ambient %.1f C (estimated)  expected NTC %.2f C  error %s  settled %s'
      % (st['ntc'], st['ambient'], st['expected_ntc'], st['error'], st['settled']))
print('other dies: afe %s C, mcu %s C, seen %.1f s ago; %d integration steps'
      % (st['afe'], st['mcu'], st['seen_s_ago'], st['steps']))
cal = device.calibration.read()
budget = observer.budget()
limits = dict(zip(thermal.ALL_NODES, cal['soa_limit_c']))
print('ceilings written in the record: %d, throttle at %.2f; the defaults in force: '
      'silicon %.0f C, laminate %.0f, motor %.0f'
      % (len(limits), cal['soa_throttle_at'], thermal.CEILING_DEFAULT_C,
         thermal.CEILING_C['board'], thermal.CEILING_C['winding']))
for node in thermal.ALL_NODES:
    print('%-12s %6.2f C  used %5.1f %%  ceiling %s'
          % (node, st['nodes'][node], 100.0 * budget['used'][node],
             '%.1f C' % limits[node] if node in limits else 'none written'))
print('worst %s at %.1f %% of its span; throttling %s, tripped %s, trips %d, derate %.2f'
      % (budget['worst_node'], 100.0 * budget['worst'], budget['throttling'],
         budget['tripped'], budget['trips'], budget['derate']))
print('sample every 30 s:', observer.configure(sample_every_s=30.0, sample_settle_s=0.5),
      observer.state()['sample_every_s'])'''),
        md('Defaults with no ceiling written: 125 C silicon, 105 laminate, 120 motor.'),
    ),
    section(
        'The envelope',
        md('Quiet, switching dry, under current, cooling. Throttle at 90 % of the span: 115 '
           'C for silicon in a 25 C room. Conduction split FET 1.8 mohm / shunt 3.5 mohm.'),
        code('''from coaxial.model import inverter, motor
from coaxial.devices.thermal_device import THROTTLE_AT

R_PHASE = inverter.RDS_ON + inverter.SHUNT
THROTTLE_C = thermal.AMBIENT + THROTTLE_AT * (thermal.CEILING_DEFAULT_C - thermal.AMBIENT)
STATES = (('quiet, no PWM', 0.0, False),
          ('switching, no current', 0.0, True),
          ('switching, 10 A of iq', 10.0, True),
          ('switching, 20 A of iq', 20.0, True),
          ('switching, 60 A of iq', 60.0, True))
print('%-24s %6s %8s %10s %11s   %s' % ('', 'W', 'board C', 'worst C', 'which', 'holdable'))
holdable = {}
for name, iq, on in STATES:
    power = thermal.phase_power(iq / math.sqrt(2.0), R_PHASE, switching=on)
    at = thermal.steady(power)
    worst = max(thermal.NODES, key=lambda n: at[n])
    holdable[name] = (at, at[worst] <= THROTTLE_C)
    print('%-24s %6.2f %8.1f %10.1f %11s   %s'
          % (name, sum(power.values()), at['board'], at[worst], thermal.pretty(worst),
             'yes' if holdable[name][1] else 'NO - a burst, timed below'))
rms = thermal.continuous_amps(R_PHASE, THROTTLE_C)
iq_cont = rms * math.sqrt(2.0)
at = thermal.steady(thermal.phase_power(rms, R_PHASE))
print('continuous: %.1f A rms a phase = %.1f A of iq = %.2f N.m on the bench motor; '
      'worst node %.1f C against the %.1f C throttle point, board %.1f C'
      % (rms, iq_cont, motor.KT_NM_PER_AMP * iq_cont,
         max(at[n] for n in thermal.NODES), THROTTLE_C, at['board']))'''),
        md('A burst climbs at `P / capacity`; capacities unmeasured, so the seconds are a '
           'band, up to 3x shorter.'),
        code('''tau_board = thermal.tau_minutes()
print('board       %.0f J/K over %.2f K/W = %.1f min'
      % (thermal.CFG['board_capacity'], thermal.CFG['board_to_ambient'], tau_board))
tau_node = {}
for node in ('driver_u', 'phase_u', 'mcu'):
    tau_node[node] = thermal.CFG['capacity'][node] * thermal.CFG['to_board'][node]
    print('%-11s %.2f J/K over %.1f K/W = %.1f s'
          % (thermal.pretty(node), thermal.CFG['capacity'][node],
             thermal.CFG['to_board'][node], tau_node[node]))
print()
print('  iq A   W on the FET node   K/s   W on the shunt   K/s   s to the throttle point from ambient   from a warm board (60 C)')
burst = {}
for iq in (20.0, 40.0, 60.0, 100.0):
    power = thermal.phase_power(iq / math.sqrt(2.0), R_PHASE)   # the FET's conduction and switching, the shunt's conduction
    fet, shunt = power['driver_u'], power['phase_u']
    slope = max(fet / thermal.CFG['capacity']['driver_u'], shunt / thermal.CFG['capacity']['phase_u'])
    burst[iq] = (fet, shunt, slope, (THROTTLE_C - thermal.AMBIENT) / slope, (THROTTLE_C - 60.0) / slope)
    print('%7.0f %17.1f %5.1f %16.1f %5.1f %38.2f %26.2f'
          % (iq, fet, fet / thermal.CFG['capacity']['driver_u'], shunt,
             shunt / thermal.CFG['capacity']['phase_u'], burst[iq][3], burst[iq][4]))'''),
        md('Rise over the board `P * to_board` on `capacity * to_board`; the board on 6.8 '
           'min.'),
        code('''def rise_after(node, watts, seconds):
    tau = thermal.CFG['capacity'][node] * thermal.CFG['to_board'][node]
    return watts * thermal.CFG['to_board'][node] * (1.0 - math.exp(-seconds / tau))

def seconds_to(node, watts, now_c, ceiling_c):
    tau = thermal.CFG['capacity'][node] * thermal.CFG['to_board'][node]
    top = now_c + watts * thermal.CFG['to_board'][node]
    if top <= ceiling_c:
        return None
    return -tau * math.log(1.0 - (ceiling_c - now_c) / (top - now_c))

def cools_in(rise_k, tau_s, to_k=1.0):
    return tau_s * math.log(rise_k / to_k) if rise_k > to_k else 0.0

for node in ('phase_u', 'driver_u', 'mcu'):
    for watts in (5.0, 15.0, 35.0):
        t = seconds_to(node, watts, st['nodes'][node], THROTTLE_C)
        print('%-9s %5.1f W: +%5.1f K after 100 ms, +%6.1f K steady; the %.0f C throttle point from %.1f C: %s'
              % (node, watts, rise_after(node, watts, 0.1), watts * thermal.CFG['to_board'][node],
                 THROTTLE_C, st['nodes'][node], 'never' if t is None else '%.2f s' % t))
print()
cooling = {}
for node in ('driver_u', 'phase_u'):
    cooling[node] = [cools_in(THROTTLE_C - thermal.AMBIENT, tau_node[node], target) for target in (20.0, 5.0, 1.0)]
    print('%-9s after a burst to the throttle point: to +20 K over the board in %.1f s, +5 K in %.1f, +1 K in %.1f'
          % (node, cooling[node][0], cooling[node][1], cooling[node][2]))
board_cools = {}
for name, iq, on in STATES:
    at, holds = holdable[name]
    if not holds:
        continue
    board_cools[name] = (at['board'] - thermal.AMBIENT, cools_in(at['board'] - thermal.AMBIENT, tau_board * 60.0) / 60.0)
    print('board after %-24s +%5.1f K, back to +1 K in %.0f min' % (name + ':', board_cools[name][0], board_cools[name][1]))'''),
    ),
    section(
        'The tour: the room, the air path and the capacity identified',
        md("The stand-in's ground truth: a room, an air-path scale, a capacity scale, "
           'identified by `coaxial/kalman/thermal_ident.py` (mirrors `thermal_ident.c`). '
           'Rooms 20, -25, 45 C; load 2 min at 30 A, 4 idle; 150 model minutes.'),
        code('''truth = observer.situation('tour')
print('the board, switched on at %.0f C, carried into %s at %.0f C' % (thermal.AMBIENT, truth['situation'], truth['ambient']))
print('rooms on the tour: ' + ', '.join('%s %.0f C' % (name, observer.SITUATIONS[name]['ambient'])
                                        for name in observer.TOUR))
print('the load cycle:', observer.load_cycle(on_s=120.0, off_s=240.0))
rows, faces = [], {}
last = truth['situation']
for minute in range(1, 151):
    observer.fast_forward(60.0, live=True)
    now, got, truth = observer.state(), observer.identification(), observer.truth()
    if truth['load_a']:
        faces[truth['situation']] = (minute, got['ambient'], dict(now['nodes']))   # the last loaded minute of each leg
    rows.append({'minute': minute, 'situation': truth['situation'], 'state': got['state'],
                 'margin': got['margin'], 'room': got['ambient'], 'room_sigma': got['ambient_sigma'],
                 'room_truth': truth['ambient'], 'air': got['scales']['air'],
                 'air_sigma': got['sigma']['air'], 'air_truth': truth['air'],
                 'capacity': got['scales']['capacity'], 'capacity_truth': truth['capacity'],
                 'innovation': got['innovation_k'], 'driver_u': now['nodes']['driver_u'],
                 'load_a': truth['load_a'] or 0.0})
    if truth['situation'] != last:
        print('minute %3d: %s -> %s' % (minute, last, truth['situation']))
        last = truth['situation']
stable = [r for r in rows if r['state'] == 'STABLE']
print('%d minutes STABLE of %d; final margin %.2f, room %.1f C for %.0f, air %.2f for %.2f, capacity %.2f for %.2f'
      % (len(stable), len(rows), rows[-1]['margin'], rows[-1]['room'], rows[-1]['room_truth'],
         rows[-1]['air'], rows[-1]['air_truth'], rows[-1]['capacity'], rows[-1]['capacity_truth']))'''),
        md('Margin: 0.80 while the model is doubted, 1.0 when not. A room step charges the '
           'room, not the air path.'),
        code('''from IPython.display import display

for room in observer.TOUR:
    if room not in faces:
        continue
    minute, found, nodes = faces[room]
    display(ansi.image(thermalmap.render({n: nodes[n] for n in thermal.NODES}, nodes['board'],
                                         cells=48, colour=True,
                                         title='%s, minute %d: room %.0f C, identified %.0f C, driver U %.0f C'
                                               % (room, minute, observer.SITUATIONS[room]['ambient'],
                                                  found, nodes['driver_u']))))'''),
        code('''from coaxial.draw.figures import figure, show

t = [r['minute'] for r in rows]
fig, (top, room, air, load) = figure(rows=4, sharex=True)
top.plot(t, [r['margin'] for r in rows], label='margin the envelope keeps')
top.step(t, [{'UNCERTAIN': 0.8, 'CONVERGING': 0.9, 'STABLE': 1.0}[r['state']] for r in rows],
         where='post', alpha=0.4, label='state as a word (0.8 / 0.9 / 1.0)')
top.set_ylabel('margin')
top.legend(loc='lower right')
room.plot(t, [r['room'] for r in rows], label='room identified')
room.fill_between(t, [r['room'] - r['room_sigma'] for r in rows],
                  [r['room'] + r['room_sigma'] for r in rows], alpha=0.2)
room.step(t, [r['room_truth'] for r in rows], where='post', label='room, the truth')
room.set_ylabel('C')
room.legend(loc='lower right')
air.plot(t, [r['air'] for r in rows], label='air scale identified')
air.fill_between(t, [r['air'] - r['air_sigma'] for r in rows],
                 [r['air'] + r['air_sigma'] for r in rows], alpha=0.2)
air.step(t, [r['air_truth'] for r in rows], where='post', label='air scale, the truth')
air.set_ylabel('x default')
air.legend(loc='lower right')
load.plot(t, [r['driver_u'] for r in rows], label='driver U, estimated')
load.plot(t, [r['load_a'] for r in rows], alpha=0.5, label='load, A')
load.set_ylabel('C / A')
load.set_xlabel('model minute')
load.legend(loc='upper right')
show(fig)'''),
    ),
    section(
        'A trip',
        md('A trip drops MOE; the margin is capped at 0.70 and returns 1 %/min. Armed on '
           'the stand-in, cold room, 200 A three phases for 2 min, then 38 idle.'),
        code('''print('load cycle off:', observer.load_cycle(0))
truth = observer.situation('cold')
print('carried into %s at %.0f C' % (truth['situation'], truth['ambient']))
print('stage armed:', device.gates.on(bypass_sto=True, ignore_interlock=True)['pwm_enabled'])
cooked = {'amps': (200.0, 200.0, 200.0), 'switching': True}
idle = {'amps': (0.0, 0.0, 0.0), 'switching': False}
trace = []
for minute in range(1, 41):
    observer.fast_forward(60.0, seen=cooked if minute <= 2 else idle, live=True)
    b, got = observer.budget(), observer.identification()
    trace.append((minute, got['margin'], b['worst'], b['trips'], got['trip_cap'], device.gates.is_on()))
    if minute in (1, 2, 3, 5, 10, 20, 30, 40):
        print('minute %2d: trips %d, stage armed %-5s worst %.2f of the span in force, '
              'trip cap %.2f, margin %.2f, %s'
              % (minute, b['trips'], device.gates.is_on(), b['worst'], got['trip_cap'],
                 got['margin'], got['state']))
print('disarmed:', not device.gates.off()['pwm_enabled'])'''),
        ),
]

RESULTS = [
    code('''moves = [r['minute'] for a, r in zip(rows, rows[1:]) if a['situation'] != r['situation']]
before = [rows[m - 2] for m in moves]          # the minute before each move: STABLE, the leg settled
print('1. the network     %d nodes; board %.2f K/W and %.0f J/K, tau %.1f min - the two measured numbers; '
      'a leg %.0f K/W between the camera\\'s %.1f and the coupon\\'s %.1f'
      % (len(thermal.ALL_NODES), thermal.CFG['board_to_ambient'], thermal.CFG['board_capacity'],
         tau_board, thermal.LEG_TO_BOARD, camera_leg, sheet_leg))
print('2. the NTC         %.1f K over the board at rest, %.2f of the V leg\\'s rise, lag %.0f s; '
      'the switching state misses by %.1f K'
      % (thermal.NTC_OFFSET, thermal.NTC_SEES_DRIVERS, thermal.NTC_TAU_S, thermal.NTC_CAMPAIGN_RESIDUAL_K))
print('3. the fit         mcu %.1f and regulators %.1f K/W off the passive state, the graph\\'s %.1f and %.1f; '
      'bridge %.1f K/W lumped'
      % (fit['mcu'], fit['regulators'], thermal.CFG['to_board']['mcu'],
         thermal.CFG['to_board']['regulators'], bridge_lumped))
print('4. switching dry   %.2f W: board %.1f C, %s the hottest at %.1f C, NTC expected %.2f C; '
      '25 min is %.0f %% of the way'
      % (sum(thermal.POWER_SWITCHING.values()), steady['board'],
         max(thermal.NODES, key=lambda n: steady[n]), max(steady[n] for n in thermal.NODES),
         ntc_steady, 100 * thermal.settled_fraction(25)))
print('5. the board       NTC %s C, error %s, worst %s at %.1f %%; ceilings written %d, the defaults in force'
      % (st['ntc'], st['error'], budget['worst_node'], 100.0 * budget['worst'], len(limits)))
print('6. holdable        ' + '; '.join('%s: board %.1f C, worst %.1f' % (name, at['board'], max(at[n] for n in thermal.NODES))
                                        for name, (at, holds) in holdable.items() if holds))
print('7. continuous      %.1f A rms a phase = %.1f A of iq = %.2f N.m against the %.0f C throttle point'
      % (rms, iq_cont, motor.KT_NM_PER_AMP * iq_cont, THROTTLE_C))
print('8. a burst         driver %.1f s, shunt %.1f s, board %.1f min; 60 A of iq: %.1f W on the FET, %.1f on the shunt, '
      '%.0f K/s, %.1f s to the throttle point from ambient, %.1f from 60 C; cold to +1 K over the board in %.1f s'
      % (tau_node['driver_u'], tau_node['phase_u'], tau_board, burst[60.0][0], burst[60.0][1], burst[60.0][2],
         burst[60.0][3], burst[60.0][4], max(cooling['driver_u'][2], cooling['phase_u'][2])))
print('9. the board cools %s' % '; '.join('%s +%.1f K, back to +1 K in %.0f min' % (name, rise, minutes)
                                          for name, (rise, minutes) in board_cools.items()))
print('10. the tour       moved at minutes %s; STABLE %d of %d, the margin at the floor %d minutes, whole %d'
      % (moves, len(stable), len(rows), sum(1 for r in rows if r['margin'] <= 0.8 + 1e-9),
         sum(1 for r in rows if r['margin'] >= 0.999)))
print('    the room       at each leg\\'s end: ' + ', '.join('%.1f for %.0f' % (r['room'], r['room_truth']) for r in before)
      + '; worst %.1f K' % max(abs(r['room'] - r['room_truth']) for r in before))
print('    the air path   at each leg\\'s end: ' + ', '.join('%.2f for %.2f' % (r['air'], r['air_truth']) for r in before)
      + '; capacity %.2f for %.2f at the end' % (rows[-1]['capacity'], rows[-1]['capacity_truth']))
print('11. the trip       %d trip, stage armed after it: %s; margin %.2f the minute after, %.2f at minute 10, '
      '%.2f at 20, %.2f at 30, %.2f at 40; the cap %.2f -> %.2f'
      % (trace[-1][3], trace[0][5], trace[2][1], trace[9][1], trace[19][1], trace[29][1], trace[39][1],
         trace[0][4], trace[-1][4]))'''),
    md('- Board 8.33 K/W, 49 J/K: 6.8 min. NTC +6.0 K over the board at rest.\n- 2.40 W '
       'switching dry; the supply corner hottest at 72 C.\n- Continuous ~20 A rms a phase; '
       '60 A lasts seconds.\n- Taken dry: at 100 A the shunt alone is 35 W against 1.2 W.'),
]

BENCH = ('`error` first. Capacities: `dT/dt` right after a power step '
         '(`tools/bench/pulse.py`) is `P / capacity`.')

REFERENCES = [
    ('host/coaxial/model/thermal.py', 'the network on the host: the nodes, the edges, the campaign\'s table, and every constant\'s argument'),
    ('thermal/src/thermal.c', 'the same network as the board integrates it, and the envelope'),
    ('host/coaxial/thermal_ident.py', 'the identification the stand-in runs, mirroring `thermal/src/thermal_ident.c`'),
    ('host/coaxial/devices/thermal_device.py', '`device.thermal`: state, budget, identification, the record\'s ceilings and the sample interval'),
    ('host/coaxial/simulated/thermal/', 'the stand-in: a hypothetical board with a ground truth, the situations, the tour and the trip cap'),
    ('docs/HARDWARE.md', 'the campaign, the camera, and how a measurement here is to be read'),
    ('docs/FINDINGS.md', 'what ran off and what was ruled out: the room inferred two other ways, the clamped scales'),
    ('host/tests/test_thermal_core.py', 'the envelope as the C that will run, and the identification against a ground truth'),
]

