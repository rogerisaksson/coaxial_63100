"""The 3D helicopter and the ornithopter in MuJoCo, and with the aerobat their boards and views."""
import math
import os
import sys

from rich.console import Console
from tools.dev.focus import chosen
from views_kit import Report, ansi_plain


def test_helicopter_stunt_routine(report):
    """`helicopter` and `ornithopter` fly their cards in MuJoCo (the aerobat: `test_aerobat`)."""
    from machine import aerobatics, aerokin, helicopter, ornithopter
    dt, runs = 1.0 / 120.0, {}
    for craft in (helicopter, ornithopter):
        sky, core, law = craft.Sky(), craft.Core(), aerokin.fresh()
        route = aerobatics.routine(craft.CARD)
        pull_max = craft.PULL_MAX
        by_stage, stages = {}, []
        for step_idx in range(int(115.0 / dt)):
            stage, row = aerobatics.fly(route, step_idx * dt, ('held', 'fit') if law['held'] else ('fit',))
            state = sky.state()
            ans = core.step(law, row, state, dt, pull_max)
            sky.step(ans.speeds + ans.controls, dt)
            shown = aerokin.FALL if stage == 'burn' and law['doing'] == aerokin.FALL else stage
            if not stages or stages[-1] != shown:
                stages.append(shown)
                if stages[-2:] == ['land', 'idle']:
                    break
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(state['turn'][1][1])))))
            by_stage.setdefault(stage, []).append({
                'h': state['h'], 'v': state['v'], 'tilt': tilt, 'r11': float(state['turn'][1][1]),
                'lift': state['lift'], 'coll': ans.controls[0], 'cyc_p': abs(ans.controls[1]),
                'cyc_r': abs(ans.controls[2]), 'held': law['held'], 'z': float(state['at'][2]),
                'vz': float(state['vel'][2]), 'hdg': law['heading'][0],
                'r': math.hypot(float(state['at'][0]), float(state['at'][2])),
                'borne': -state['lift'] * sky.ground(ans.controls[0])})
        runs[craft] = (sky, by_stage, stages)

    sky, by_stage, stages = runs[helicopter]
    hover, funnel, spiral = by_stage['hover'], by_stage['funnel'], by_stage['spiral']
    low, up, high = (helicopter.STUNTS[k]['height'] for k in ('hover', 'spiral', 'high'))
    report.check('upright hover holds %.2f m on positive collective' % low,
                 abs(hover[-1]['h'] - low) < 0.05 and hover[-1]['lift'] > 18.0 and hover[-1]['held'])
    report.check('funnel banks > 22 deg on the orbit and spiral climbs to %.1f m' % up,
                 max(p['tilt'] for p in funnel) > 22.0 and max(p['r'] for p in funnel) > 2.0
                 and abs(spiral[-1]['h'] - up) < 0.1)
    inv, piro_inv, tictoc = by_stage['inverted'], by_stage['piro-inv'], by_stage['tic-toc']
    report.check('inverted hover and inverted pirouette fly upside-down on negative collective, '
                 'the floor bearing its share',
                 inv[-1]['tilt'] > 170.0 and inv[-1]['r11'] < -0.98 and inv[-1]['borne'] > 18.0
                 and inv[-1]['coll'] < -0.15 and inv[-1]['held'] and inv[-1]['r'] < aerokin.HELD_SPOT_M
                 and min(p['tilt'] for p in piro_inv) > 165.0 and max(p['coll'] for p in piro_inv) < -0.15)
    report.check('elevator tic-tocs rock across 90 deg vertical snapping collective + and -',
                 min(p['tilt'] for p in tictoc) < 60.0 and max(p['tilt'] for p in tictoc) > 120.0
                 and max(p['lift'] for p in tictoc) > 25.0 and min(p['lift'] for p in tictoc) < -20.0
                 and min(p['h'] for p in tictoc) > helicopter.STUNTS['tictoc']['height'] - 0.2)
    pflip, roll = by_stage['piro-flip'], by_stage['roll']
    report.check('3D piro-flip stirs pitch/roll cyclic while flipping and axial roll recovers at %.2f m'
                 % high,
                 min(p['r11'] for p in pflip) < -0.85 and max(p['cyc_p'] for p in pflip) > 0.25
                 and max(p['cyc_r'] for p in pflip) > 0.25 and min(p['r11'] for p in roll) < -0.85
                 and min(p['h'] for p in pflip + roll) > high - 0.2)
    full_tilt, burn, hold, land = (by_stage[k] for k in ('full tilt', 'burn', 'hold', 'land'))
    report.check('full tilt climbs past 12 m on the governed head and the burn arrests at 10 cm',
                 max(p['v'] for p in full_tilt) > 9.0 and max(p['h'] for p in burn) > 12.0
                 and min(p['lift'] for p in burn if p['r11'] > 0.5) < 2.0
                 and stages.count('fall') == 1 and stages.count('burn') == 1
                 and abs(hold[-1]['h'] - helicopter.FLOOR_M) < 0.04 and hold[-1]['held']
                 and land[-1]['h'] < 0.02 and land[-1]['hdg'] > 0.99 and sky.hit is None)

    c_sky, c_by, c_st = runs[ornithopter]
    hov, orb, inv, cpg, tum = (c_by[k] for k in ('hover', 'spiral', 'dive', 'wing-rock', 'slash'))
    ft, burn, hold, land = (c_by[k] for k in ('full tilt', 'burn', 'hold', 'land'))
    report.check('machine.ornithopter flies its routine and arrests over the floor',
                 abs(hov[-1]['h'] - ornithopter.STUNTS['hover']['height']) < 0.06 and hov[-1]['held']
                 and abs(orb[-1]['h'] - ornithopter.STUNTS['thermal_spiral']['height']) < 0.1
                 and inv[-1]['r11'] < -0.97 and inv[-1]['lift'] < -17.0
                 and min(p['tilt'] for p in cpg) < 65.0 and max(p['tilt'] for p in cpg) > 115.0
                 and min(p['r11'] for p in tum) < -0.80 and max(p['v'] for p in ft) > 9.0
                 and max(p['h'] for p in burn) > 12.0 and min(p['lift'] for p in burn if p['r11'] > 0.5) < 2.0
                 and c_st.count('fall') == 1 and abs(hold[-1]['h'] - ornithopter.FLOOR_M) < 0.05
                 and land[-1]['h'] < 0.02 and land[-1]['hdg'] > 0.99 and c_sky.hit is None)


def test_helicopter_wind_observer_and_inverted_hold(report):
    """The three flip from `high` to their inverted rows and track gusts."""
    from machine import aerobat, aerokin, helicopter, ornithopter, weather
    dt = 1.0 / 120.0
    for c, inv_row in ((helicopter, helicopter.INVERTED),
                       (ornithopter, ornithopter.SANDSTORM_DIVE),
                       (aerobat, aerobat.STUNTS['high'])):
        sky, core, law, air = c.Sky(), c.Core(), aerokin.fresh(), weather.air(7, 'gusty')
        for k in range(int(10.0 / dt)):
            weather.blown(air, dt)
            ans = core.step(law, c.STUNTS['high'] if k * dt < 5.0 else inv_row, sky.state(), dt, c.PULL_MAX)
            sky.step(ans.speeds + ans.controls, dt, air)
        st = sky.state()
        act, est = weather.wind_at(air['wind'], st['h'], air['steady']), ans.wind
        # Gusty, the aerobat's figures higher by its law's margin.
        aim = inv_row['height'] + (core.margin(inv_row) if c is aerobat else 0.0)
        report.check('%s wind observer tracks gusty wind while holding its hover' % c.__name__,
                     abs(st['h'] - aim) < 0.1
                     and float(st['turn'][1][1]) * (-1.0 if c is aerobat else 1.0) < -0.90
                     and (act[0] * est[0] + act[2] * est[2]) > 0.5 and sky.hit is None)


def test_helicopter_boards_and_view(report):
    """Two boards fly the three; four renderers and HUDs fit at 150x44."""
    from coaxial import Coaxial63100
    from coaxial.graphics import (aerobat as aero_gfx, engine, helicopter as heli_gfx, shapes,
                                  ornithopter as orni_gfx, quadcopter as quad_gfx)
    from machine import aerobat, aerobatics, aerokin, helicopter, ornithopter
    from machine.modes import SIMULATED
    from terminal.ui import stage
    from terminal.views import (show_aerobat as aview, show_helicopter as view,
                                show_ornithopter as oview, show_quad as quad_view)
    from terminal.views.aerobat import flight as aflown
    from terminal.views.helicopter import flight as flown
    from terminal.views.ornithopter import flight as oflown

    card = (('spool', aerobatics.PRESSED, 1.0, 1.0, ''), ('lift', aerobatics.HOVER, 1.8, 1.8, ''),
            ('full tilt', aerobatics.SKY, 0.8, 0.0, ''), ('flip', helicopter.FLIP_IN, 1.0, 0.0, ''),
            ('inverted', helicopter.INVERTED, 1.5, 0.0, ''))
    rotors = [flown.arm(Coaxial63100(execution_mode=SIMULATED).open()) for _ in flown.ROTORS]
    try:
        sky, route = helicopter.Sky(), aerobatics.routine(card)
        law, flight, clock = aerokin.fresh(), flown.fresh(), 0.0
        seen, min_coll, max_w, spool_a, flip_a = set(), 0.0, 0.0, 99.0, 0.0
        while clock < 5.8:
            for clock, frame in flown.passed(rotors, sky, route, law, flight, clock, 0.04):
                seen.add(flight['stage'])
                min_coll, max_w = min(min_coll, flight['controls'][0]), max(max_w, rotors[0]['w'])
                if flight['stage'] == 'spool' and clock > 0.8:
                    spool_a = min(spool_a, rotors[0]['amps'])
                if flight['stage'] == 'flip':
                    flip_a = max(flip_a, rotors[0]['amps'])
        report.check('two 63100 boards fly spool -> lift -> full tilt -> flip -> inverted with '
                     'no faults, negative collective, and warmed finned-sink cooling state',
                     [r['fault'] for r in rotors] == [None, None] and 'inverted' in seen
                     and min_coll < -0.15 and max_w > 350.0 and flip_a > spool_a + 6.0
                     and rotors[0]['amps'] > rotors[1]['amps']
                     and rotors[0]['cooling']['switches_c'] > rotors[0]['cooling']['sink_c'] > 25.0)

        osky, oroute, olaw, oflt = ornithopter.Sky(), aerobatics.routine(oflown.CARD), aerokin.fresh(), oflown.fresh()
        asky, aroute, alaw, aflt = aerobat.Sky(), aerobatics.routine(aflown.CARD), aerokin.fresh(), aflown.fresh()
        for s_k, r_k, l_k, f_k, fn in ((osky, oroute, olaw, oflt, oflown.passed),
                                       (asky, aroute, alaw, aflt, aflown.passed)):
            list(fn(rotors, s_k, r_k, l_k, f_k, 0.0, 0.08))
        oframe, aframe = osky.state(), asky.state()
        sr = [(0.2, 10.0, 45.0, 40.0, 35.0), (0.5, 10.0, 42.0, 39.0, 34.0)]
        fr = [(r['angle'], r['w'], 55.0, 48.0, 42.0) for r in rotors]
        h_s, h_f = ('\n'.join(heli_gfx.render(frame, r, 60, 24, thrusts=view.thrusts(frame, rotors))) for r in (sr, fr))
        o_s, o_f = ('\n'.join(orni_gfx.render(oframe, r, 60, 24, thrusts=oview.thrusts(oframe, rotors))) for r in (sr, fr))
        a_s, a_f = ('\n'.join(aero_gfx.render(aframe, r, 60, 24, thrusts=aview.thrusts(aframe, rotors))) for r in (sr, fr))
        q_art = '\n'.join(quad_gfx.render(frame, [(0.3 * k, 420.0, 65.0, 58.0, 85.0) for k in range(4)], 60, 24))

        con = Console(width=150, height=44, theme=stage.THEME, force_terminal=True)
        orig, tr = rotors[0]['rig'].origin, [(clock, frame['h'], 62.0, 240.0, 48.0)]
        hi_f = dict(frame, h=45.2, v=-22.4, a=-11.2 * 9.80665)
        outs = []
        for fn, args in ((view.compose, (con, orig, rotors, frame, flight, tr, clock, h_f)),
                         (oview.compose, (con, orig, rotors, oframe, oflt, tr, clock, o_f)),
                         (aview.compose, (con, orig, rotors, aframe, aflt, tr, clock, a_f)),
                         (quad_view.compose, (con, orig, rotors * 2, hi_f, flight, tr, clock, q_art))):
            with con.capture() as cap:
                con.print(fn(*args))
            outs.append(ansi_plain(cap.get()))
        h_txt, o_txt, a_txt, q_txt = outs
        m, cam = shapes.view(30.0, 0.0), engine.fine(engine.camera(60, 24, 1.8, distance=7.0))
        up_p = {'at': (0.0, 0.0, 0.0), 'turn': ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))}
        inv_p = {'at': (0.0, 0.0, 0.0), 'turn': ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))}
        arr_ok = all(g.force_arrows(up_p, [25.0, 0.0], g.ARROWS, m, cam, up_p['at'])[0][0][:45].any()
                     and g.force_arrows(up_p, [-60.0, 0.0], g.ARROWS, m, cam, up_p['at'])[1][0][45:].any()
                     and g.force_arrows(inv_p, [-60.0, 0.0], g.ARROWS, m, cam, inv_p['at'])[1][0][:45].any()
                     for g in (heli_gfx, orni_gfx, aero_gfx))
        report.check('all four aircraft renderers and HUDs fit all 5 boxes at 150x44 without truncation',
                     len(h_s.splitlines()) == 24 == len(o_f.splitlines()) == len(a_f.splitlines())
                     and h_s != h_f and o_s != o_f and a_s != a_f and arr_ok
                     and heli_gfx.BLADES_RAD_S == orni_gfx.BLADES_RAD_S == aero_gfx.BLADES_RAD_S == 20.0
                     and '▾' not in (h_txt + o_txt + a_txt + q_txt) and '45.20 m, -22.4 m/s, -11.2 g' in q_txt
                     and all(t in h_txt for t in ('HELICOPTER', 'MAIN', 'TAIL', 'SWASH', 'SINK'))
                     and all(t in o_txt for t in ('ORNITHOPTER', 'FORE', 'AFT', 'WINGS', 'SINK'))
                     and all(t in a_txt for t in ('AEROBAT', 'PROP', 'VECT', 'SURFS', 'SINK')))

        sky.hit = 'floor'
        stage_hit = flown.step(rotors, sky, route, law, flight, clock + 0.02, 0.02)
        flight['wreck']['for'] = flown.CRASH_S
        flown.step(rotors, sky, route, law, flight, clock + 0.04, 0.02)
        report.check('struck helicopter enters CRASHED, cuts rotors, and resets after CRASH_S',
                     stage_hit == flown.CRASHED and flight['wreck'] is None
                     and flight['crashes'] == 1 and sky.hit is None)
    finally:
        for r in rotors:
            r['rig'].board.drive.off()
            r['rig'].gates.off()
            r['rig'].close()


def test_helicopter_course_laps(report):
    """The helicopter and the ornithopter fly the 16-gate course in wind and land on gate 0's
    heading (the aerobat its airfield's: `test_aerobat`)."""
    from machine import aerobatics, aerokin, course, grounds, helicopter, ornithopter, pull, weather
    solids, dt, tick = grounds.solids(), 0.02, 0.002
    for c in (helicopter, ornithopter):
        sky, core, law = c.Sky(solids), c.Core(), aerokin.fresh()
        route, air = aerobatics.routine(course.TWIN_CARD), weather.air(7, 'gusty')
        top, whole = c.K_THRUST * pull.held(pull.I_MAX) ** 2 / c.MASS_KG, aerokin.HEADROOM * c.PULL_MAX
        have, stages, max_dc, sum_dc, n_lap, prev_c, end_l = c.COURSE_SPHERE, [], 0.0, 0.0, 0, (0.0, 0.0), None
        for k in range(int(100.0 / dt)):
            st = sky.state()
            route['seen'] = dict(aerokin.seen(law, st, dt, have, top), spent=False)
            stage, row = aerobatics.fly(route, k * dt, ('held', 'fit') if law['held'] else ('fit',))
            if not stages or stages[-1] != stage:
                stages.append(stage)
            along = core.plan(law, row, st, dt, whole)
            for _ in range(10):
                ans = core.step(law, row, sky.state(), tick, whole, None, along)
                weather.blown(air, tick)
                sky.step(ans.speeds + ans.controls, tick, air)
            if stage == 'lap':
                cyc = ans.controls[1:3]
                dc = max(abs(cyc[0] - prev_c[0]), abs(cyc[1] - prev_c[1]))
                max_dc, sum_dc, n_lap, prev_c = max(max_dc, dc), sum_dc + dc, n_lap + 1, cyc
            elif stage == 'land':
                end_l = (sky.state()['h'], abs(sky.state()['vel'][2]), law['heading'][0])
            if sky.hit is not None or (len(stages) > 6 and stage == 'idle'):
                break
        # The ornithopter's cyclic steps 1.3-1.9 a pass at its dives' turn-over in ten gusty seeds.
        report.check('%s laps the 16-gate course in gusty wind and lands' % c.__name__,
                     sky.hit is None and (route.get('lap') or {}).get('laps', 0) >= 1
                     and stages[-2:] == ['land', 'idle'] and end_l is not None
                     and end_l[0] < 0.025 and end_l[1] < 0.03 and end_l[2] > 0.99
                     and sum_dc / max(1, n_lap) < 0.14 and max_dc < 2.0,
                     'hit %s, laps %s, %s, landed %s' % (
                         sky.hit, (route.get('lap') or {}).get('laps'), stages[-3:], end_l))


def test_lisp_manifests_and_validation(report):
    """Shared Lisp blocks compile alone and into the four crafts' files; cooling and errors check."""
    from machine import aerobat, aerobatics, aerokin, aerolisp, heat, helicopter, ornithopter, quad
    hm, qm, om, am = helicopter.MANIFEST, quad.MANIFEST, ornithopter.MANIFEST, aerobat.MANIFEST
    mdir = os.path.dirname(os.path.abspath(helicopter.LISP_PATH))
    shared_files = ('aerodynamics.lisp', 'dynamics.lisp', 'kinematics.lisp')
    fns = (aerolisp.compile_aerodynamics, aerolisp.compile_dynamics, aerolisp.compile_kinematics)
    shared = [fn(open(os.path.join(mdir, name), encoding='utf-8').read()) for fn, name in zip(fns, shared_files)]
    report.check('shared Lisp files compile and puzzle into helicopter, quadcopter, ornithopter, and aerobat',
                 shared[0]['name'] == 'shared' and shared[1]['name'] == 'shared'
                 and shared[2]['kinematics']['name'] == 'shared' and len(shared[2]['stunts']) == 11
                 and all(m['includes'] == shared_files for m in (hm, qm, om, am))
                 and (hm['vehicle'], qm['vehicle'], om['vehicle'], am['vehicle']) == (
                     'helicopter_3d', 'quadcopter', 'ornithopter_dune', 'aerobat_3d')
                 and [s.sequence_id for s in hm['streams']] == [301, 302, 303, 304, 305, 306]
                 and [s.sequence_id for s in om['streams']] == [401, 402, 403, 404, 405, 406]
                 and [s.sequence_id for s in am['streams']] == [501, 502, 503, 504, 505, 506]
                 and len(helicopter.CARD) == 24 == len(ornithopter.CARD) and len(aerobat.CARD) == 21)

    q_card = qm['routines']['quadcopter']
    try:
        quad.sized(0.75)
        aerobatics.sized()
        aerokin.sized()
        scaled_h = aerobatics.CARD[2][1]['height']
    finally:
        quad.sized(1.0)
        aerobatics.sized()
        aerokin.sized()
    report.check('quadcopter.lisp drives quad, aerobatics, and aerokin and resizes under quad.sized',
                 qm is helicopter.QUAD_MANIFEST and qm['aerodynamics']['name'] == 'quadcopter'
                 and [s.sequence_id for s in qm['streams']] == [101, 102, 103, 104]
                 and all(aerobatics.CARD[k][1] is q_card[k][1] for k in range(len(q_card)))
                 and abs(scaled_h - 0.3 * 0.75) < 1e-9 and abs(aerobatics.HOVER['height'] - 0.3) < 1e-9)

    sf, nf = heat.cooling_state(25.0, quad.COOLING), dict(quad.COOLING, fan_temp_on=500.0, fan_temp_full=600.0)
    sn = heat.cooling_state(25.0, nf)
    for _ in range(400):
        heat.step_cooling(sf, 85.0, 25.0, 1.0, 0.0, quad.COOLING)
        heat.step_cooling(sn, 85.0, 25.0, 1.0, 0.0, nf)
    s_on, s_cut = (heat.step_board_cooling(heat.cooling_state(25.0, quad.COOLING), 0.0, 63.0, 1.0, 0.0, quad.COOLING, switching=sw)
                   for sw in (True, False))
    report.check('finned aluminium heatsink + fan cooling model is stable and tracks switching loss',
                 quad.COOLING == helicopter.COOLING == ornithopter.COOLING == aerobat.COOLING
                 and sf['switches_c'] > sf['sink_c'] > 25.0 and sf['fan_duty'] > 0.3
                 and sf['sink_c'] + 25.0 < sn['sink_c'] and s_on['switches_c'] > s_cut['switches_c'] + 0.5)

    bad_cases = (
        ('(defunknown x)', 'unknown'), ('(include "no_such.lisp")', 'no_such.lisp'),
        ('(defaerodynamics h (warp_drive (x 1.0)))', 'warp_drive'),
        ('(defaerodynamics h (main_rotor (disc_height 0.1m)))', 'missing fields'),
        ('(defdynamics h (rigid_body (mass 2.0kg)))', 'missing fields'),
        ('(defdynamics h (impedance (stiffness warp_axis 1.0)))', 'warp_axis'),
        ('(defkinematics h (envelope (floor_height 1.0knots)))', '1.0knots'),
        ('(defstunt s (base no_such_stunt) (height 1.0m))', 'no_such_stunt'),
        ('(defbehavior b (phase p (stunt no_stunt) (duration 1.0s)))', 'no_stunt'),
        ('(defbehavior empty_behavior)', 'defbehavior requires'),
        ('(defroutine empty_routine)', 'defroutine requires'),
        ('(stream_inject (sequence_id not_an_int))', 'integer'),
        ('(defblock unknown_kind b (x 1.0))', 'no block kind'),
        ('(defaerodynamics h (use no_such_block))', 'unknown aerodynamics block'),
        ('(defvehicle empty_v)', 'defvehicle requires'),
    )
    refused = 0
    for src, clue in bad_cases:
        try:
            aerolisp.compile_manifest(src)
        except SyntaxError as exc:
            refused += int(clue in str(exc))
    report.check('aerolisp.compile_manifest refuses malformed forms with SyntaxError',
                 refused == len(bad_cases), '%d/%d refused' % (refused, len(bad_cases)))


def test_helicopter_behaviors_and_streams(report):
    """3D stunt behaviors, each from `high`, fly in MuJoCo; compiled `stream_inject` payloads
    translate to stunt rows."""
    from machine import aerobat, aerokin, aerolisp, helicopter, ornithopter
    dt = 1.0 / 120.0
    for craft, s_cpg, s_tum in ((helicopter, 'aileron_tictoc', 'chaos'),
                                (ornithopter, 'wing_rock', 'coriolis_storm'),
                                (aerobat, 'waterfall', 'roll')):
        pmax = craft.PULL_MAX
        sky, core, law = craft.Sky(), craft.Core(), aerokin.fresh()
        cpg_v, chaos_r, ends = [], [], []
        for phases in craft.BEHAVIORS.values():
            for _ in range(int(2.5 / dt)):
                ans = core.step(law, craft.STUNTS['high'], sky.state(), dt, pmax)
                sky.step(ans.speeds + ans.controls, dt)
            for phase in phases:
                for _ in range(int(phase['duration'] / dt)):
                    st = sky.state()
                    ans = core.step(law, phase['row'], st, dt, pmax)
                    sky.step(ans.speeds + ans.controls, dt)
                    if phase['stunt'] == s_cpg:
                        cpg_v.append(float(st['turn'][1][1]))
                    if phase['stunt'] == s_tum:
                        chaos_r.append(math.hypot(*[float(x) for x in st['spin']]))
            ends.append(sky.state()['h'] - phases[-1]['row']['height'])
        report.check('%s multi-phase stunt behaviors fly in MuJoCo' % craft.__name__,
                     bool(cpg_v) and max(cpg_v) - min(cpg_v) > 0.6 and bool(chaos_r) and max(chaos_r) > 6.0
                     and max(abs(e) for e in ends) < 0.3 and sky.hit is None,
                     'their ends %s m off their last rows, struck %s' % (
                         ' '.join('%+.2f' % e for e in ends), sky.hit))

    rows, orows, arows = ([c.row_from_payload(p) for p in c.STREAMS] for c in (helicopter, ornithopter, aerobat))
    rcpg = helicopter.row_from_payload(aerolisp.compiled(
        '(stream_inject (cpg_config (enable true) (frequency 1.25Hz) (amplitude mode_attitude_roll 52.0deg)))')[0])
    report.check('compiled stream_inject payloads #301..#506 and roll CPG translate to stunt rows',
                 rows[0]['height'] == 1.5 == orows[0]['height'] == arows[0]['height']
                 and rows[2]['nose'] == -1.0 == orows[2]['nose'] == arows[2]['nose']
                 and rows[3]['nose'] == -2.0 == orows[3]['nose'] == arows[3]['nose']
                 and rcpg['nose'] == -3.0 and rows[5]['pace'] == 100.0)


ROSTER = (
    test_lisp_manifests_and_validation,
    test_helicopter_stunt_routine,
    test_helicopter_behaviors_and_streams,
    test_helicopter_wind_observer_and_inverted_hold,
    test_helicopter_course_laps,
    test_helicopter_boards_and_view,
)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
