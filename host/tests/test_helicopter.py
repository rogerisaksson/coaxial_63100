"""The 3D helicopter, Dune ornithopter, and 3D aerobat in MuJoCo, their boards, and their views."""
import math
import os
import sys

from rich.console import Console
from tools.dev.focus import chosen
from views_kit import Report, ansi_plain


def test_helicopter_stunt_routine(report):
    """`helicopter`, `ornithopter`, and `aerobat` fly their full 24-stage stunt cards in MuJoCo:
    hover/prop-hang, orbits, inverted hover/dive/spin, CPG tic-tocs/rock/waterfall, flips, rolls,
    full-tilt skyrocket, collective burn arrest at 10 cm, and landing."""
    from machine import aerobat, aerobatics, aerokin, helicopter, ornithopter, pull
    dt, runs = 1.0 / 120.0, {}
    for craft in (helicopter, ornithopter, aerobat):
        sky, core, law = craft.Sky(), craft.Core(), aerokin.fresh()
        route = aerobatics.routine(craft.CARD)
        pull_max = craft.K_THRUST * pull.held(pull.I_MAX) ** 2 / craft.MASS_KG
        by_stage, stages = {}, []
        for step_idx in range(int(62.0 / dt)):
            clock = step_idx * dt
            stage, row = aerobatics.fly(route, clock, ('held', 'fit') if law['held'] else ('fit',))
            state = sky.state()
            ans = core.step(law, row, state, dt, pull_max)
            sky.step(ans.speeds + ans.controls, dt)
            shown = aerokin.FALL if stage == 'burn' and law['doing'] == aerokin.FALL else stage
            if not stages or stages[-1] != shown:
                stages.append(shown)
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(state['turn'][1][1])))))
            by_stage.setdefault(stage, []).append({
                'h': state['h'], 'v': state['v'], 'tilt': tilt, 'r11': float(state['turn'][1][1]),
                'lift': state['lift'], 'coll': ans.controls[0], 'cyc_p': abs(ans.controls[1]),
                'cyc_r': abs(ans.controls[2]), 'held': law['held'],
                'r': math.hypot(float(state['at'][0]), float(state['at'][2]))})
        runs[craft] = (sky, by_stage, stages)

    sky, by_stage, stages = runs[helicopter]
    hover, funnel, spiral = by_stage['hover'], by_stage['funnel'], by_stage['spiral']
    report.check('upright hover holds 1.50 m on positive collective',
                 abs(hover[-1]['h'] - 1.5) < 0.05 and hover[-1]['lift'] > 18.0 and hover[-1]['held'])
    report.check('funnel banks > 22 deg on the orbit and spiral climbs to 8 m',
                 max(p['tilt'] for p in funnel) > 22.0 and max(p['r'] for p in funnel) > 2.0
                 and spiral[-1]['h'] > 7.5)
    inv, piro_inv, tictoc = by_stage['inverted'], by_stage['piro-inv'], by_stage['tic-toc']
    report.check('inverted hover and inverted pirouette fly upside-down on negative collective',
                 inv[-1]['tilt'] > 170.0 and inv[-1]['r11'] < -0.98 and inv[-1]['lift'] < -18.0
                 and inv[-1]['coll'] < -0.15 and inv[-1]['held'] and inv[-1]['r'] < aerokin.HELD_SPOT_M
                 and min(p['tilt'] for p in piro_inv) > 165.0 and max(p['lift'] for p in piro_inv) < -17.0)
    report.check('elevator tic-tocs rock across 90 deg vertical snapping collective + and -',
                 min(p['tilt'] for p in tictoc) < 60.0 and max(p['tilt'] for p in tictoc) > 120.0
                 and max(p['lift'] for p in tictoc) > 25.0 and min(p['lift'] for p in tictoc) < -20.0
                 and min(p['h'] for p in tictoc) > 7.0)
    pflip, roll = by_stage['piro-flip'], by_stage['roll']
    report.check('3D piro-flip stirs pitch/roll cyclic while flipping and axial roll recovers at 8 m',
                 min(p['r11'] for p in pflip) < -0.85 and max(p['cyc_p'] for p in pflip) > 0.25
                 and max(p['cyc_r'] for p in pflip) > 0.25 and min(p['r11'] for p in roll) < -0.85
                 and min(p['h'] for p in pflip + roll) > 7.2)
    full_tilt, burn, hold, land = (by_stage[k] for k in ('full tilt', 'burn', 'hold', 'land'))
    report.check('full tilt climbs past 40 m and collective burn arrests at 10 cm without chattering',
                 max(p['v'] for p in full_tilt) > 22.0 and max(p['h'] for p in burn) > 45.0
                 and stages.count('fall') == 1 and stages.count('burn') == 1
                 and abs(hold[-1]['h'] - helicopter.FLOOR_M) < 0.04 and hold[-1]['held']
                 and land[-1]['h'] < 0.02 and sky.hit is None)

    for craft, s_hov, s_orb, s_inv, s_cpg, s_tum in (
            (ornithopter, 'hover', 'spiral', 'dive', 'wing-rock', 'slash'),
            (aerobat, 'prop-hang', 'harrier', 'inverted', 'waterfall', 'lomcevak')):
        c_sky, c_by, c_st = runs[craft]
        hov, orb, inv, cpg, tum = (c_by[k] for k in (s_hov, s_orb, s_inv, s_cpg, s_tum))
        ft, burn, hold, land = (c_by[k] for k in ('full tilt', 'burn', 'hold', 'land'))
        report.check('%s flies full 24-stage stunt routine and arrests at 10 cm' % craft.__name__,
                     abs(hov[-1]['h'] - 1.5) < 0.06 and hov[-1]['held'] and orb[-1]['h'] > 7.5
                     and inv[-1]['r11'] < -0.97 and inv[-1]['lift'] < -17.0
                     and min(p['tilt'] for p in cpg) < 65.0 and max(p['tilt'] for p in cpg) > 115.0
                     and min(p['r11'] for p in tum) < -0.80 and max(p['v'] for p in ft) > 22.0
                     and max(p['h'] for p in burn) > 45.0 and c_st.count('fall') == 1
                     and abs(hold[-1]['h'] - craft.FLOOR_M) < 0.05 and land[-1]['h'] < 0.02
                     and c_sky.hit is None)


def test_helicopter_wind_observer_and_inverted_hold(report):
    """The helicopter, ornithopter, and aerobat flip cleanly from upright `HOVER` to `INVERTED` /
    `SANDSTORM_DIVE` and hold inverted hover in gusty wind while tracking the wind."""
    from machine import aerobat, aerokin, helicopter, ornithopter, pull, quad
    dt = 1.0 / 120.0
    for craft, inv_row in ((helicopter, helicopter.INVERTED),
                           (ornithopter, ornithopter.SANDSTORM_DIVE),
                           (aerobat, aerobat.INVERTED)):
        pmax = craft.K_THRUST * pull.held(pull.I_MAX) ** 2 / craft.MASS_KG
        sky, core, law, air = craft.Sky(), craft.Core(), aerokin.fresh(), quad.air(7, 'gusty')
        for step_idx in range(int(10.0 / dt)):
            row = helicopter.HOVER if step_idx * dt < 5.0 else inv_row
            quad.blown(air, dt)
            ans = core.step(law, row, sky.state(), dt, pmax)
            sky.step(ans.speeds + ans.controls, dt, air)
        state = sky.state()
        actual, est = quad.wind_at(air['wind'], state['h']), ans.wind
        report.check('%s wind observer tracks gusty wind while holding inverted hover' % craft.__name__,
                     state['h'] > 6.5 and float(state['turn'][1][1]) < -0.90
                     and (actual[0] * est[0] + actual[2] * est[2]) > 0.5 and sky.hit is None)


def test_helicopter_boards_and_view(report):
    """Two stand-in 63100 boards arm and fly `helicopter`, `ornithopter`, and `aerobat`, and all
    four aircraft 3D braille renderers and HUDs fit at 150x44 without scroll truncation."""
    from coaxial import Coaxial63100
    from coaxial.graphics import (aerobat as aero_gfx, helicopter as heli_gfx,
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
                     and min_coll < -0.15 and max_w > 500.0 and flip_a > spool_a + 6.0
                     and rotors[0]['amps'] > rotors[1]['amps']
                     and rotors[0]['cooling']['switches_c'] > rotors[0]['cooling']['sink_c'] > 25.0)

        osky, oroute, olaw, oflt = ornithopter.Sky(), aerobatics.routine(oflown.CARD), aerokin.fresh(), oflown.fresh()
        asky, aroute, alaw, aflt = aerobat.Sky(), aerobatics.routine(aflown.CARD), aerokin.fresh(), aflown.fresh()
        for _, oframe in oflown.passed(rotors, osky, oroute, olaw, oflt, 0.0, 0.08):
            pass
        for _, aframe in aflown.passed(rotors, asky, aroute, alaw, aflt, 0.0, 0.08):
            pass

        slow_r = [(0.2, 10.0, 45.0, 40.0, 35.0), (0.5, 10.0, 42.0, 39.0, 34.0)]
        fast_r = [(r['angle'], r['w'], 55.0, 48.0, 42.0) for r in rotors]
        h_slow = '\n'.join(heli_gfx.render(frame, slow_r, 60, 24, thrusts=view.thrusts(frame, rotors)))
        h_fast = '\n'.join(heli_gfx.render(frame, fast_r, 60, 24, thrusts=view.thrusts(frame, rotors)))
        o_slow = '\n'.join(orni_gfx.render(oframe, slow_r, 60, 24, thrusts=oview.thrusts(oframe, rotors)))
        o_fast = '\n'.join(orni_gfx.render(oframe, fast_r, 60, 24, thrusts=oview.thrusts(oframe, rotors)))
        a_slow = '\n'.join(aero_gfx.render(aframe, slow_r, 60, 24, thrusts=aview.thrusts(aframe, rotors)))
        a_fast = '\n'.join(aero_gfx.render(aframe, fast_r, 60, 24, thrusts=aview.thrusts(aframe, rotors)))
        quad_art = '\n'.join(quad_gfx.render(frame, [(0.3 * k, 420.0, 65.0, 58.0, 85.0) for k in range(4)], 60, 24))

        con = Console(width=150, height=44, theme=stage.THEME, force_terminal=True)
        orig, tr = rotors[0]['rig'].origin, [(clock, frame['h'], 62.0, 240.0, 48.0)]
        hi_frame = dict(frame, h=45.2, v=-22.4, a=-11.2 * 9.80665)
        outs = []
        for fn, args in ((view.compose, (con, orig, rotors, frame, flight, tr, clock, h_fast)),
                         (oview.compose, (con, orig, rotors, oframe, oflt, tr, clock, o_fast)),
                         (aview.compose, (con, orig, rotors, aframe, aflt, tr, clock, a_fast)),
                         (quad_view.compose, (con, orig, rotors * 2, hi_frame, flight, tr, clock, quad_art))):
            with con.capture() as cap:
                con.print(fn(*args))
            outs.append(ansi_plain(cap.get()))
        h_txt, o_txt, a_txt, q_txt = outs
        report.check('all four aircraft renderers and HUDs fit all 5 boxes at 150x44 without truncation',
                     len(h_slow.splitlines()) == 24 == len(o_fast.splitlines()) == len(a_fast.splitlines())
                     and h_slow != h_fast and o_slow != o_fast and a_slow != a_fast
                     and heli_gfx.BLADES_RAD_S == orni_gfx.BLADES_RAD_S == aero_gfx.BLADES_RAD_S == 20.0
                     and '▾' not in (h_txt + o_txt + a_txt + q_txt)
                     and '45.20 m, -22.4 m/s, -11.2 g' in q_txt
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
        for rotor in rotors:
            rotor['rig'].board.drive.off()
            rotor['rig'].gates.off()
            rotor['rig'].close()


def test_helicopter_course_laps(report):
    """The 3D helicopter, Dune ornithopter, and 3D aerobat fly the 16-gate race course
    (`machine.course.CARD`) in wind without striking any gate."""
    from machine import aerobat, aerobatics, aerokin, course, grounds, helicopter, ornithopter, pull, quad
    solids, dt = grounds.solids(), 1.0 / 120.0
    for craft in (helicopter, ornithopter, aerobat):
        sky, core, law = craft.Sky(solids), craft.Core(), aerokin.fresh()
        route, air = aerobatics.routine(course.CARD), quad.air(7, 'gusty')
        top = craft.K_THRUST * pull.held(pull.I_MAX) ** 2 / craft.MASS_KG
        have, stages = aerokin.HEADROOM * top, []
        for step_idx in range(int(51.0 / dt)):
            clock, state = step_idx * dt, sky.state()
            route['seen'] = dict(aerokin.seen(law, state, dt, have, top), spent=False)
            stage, row = aerobatics.fly(route, clock, ('held', 'fit') if law['held'] else ('fit',))
            if not stages or stages[-1] != stage:
                stages.append(stage)
            ans = core.step(law, row, state, dt, have)
            quad.blown(air, dt)
            sky.step(ans.speeds + ans.controls, dt, air)
            if sky.hit is not None or (stage == 'land' and state['h'] < 0.02):
                break
        report.check('%s completes both 16-gate course laps in gusty wind and lands cleanly' % craft.__name__,
                     sky.hit is None and (route.get('lap') or {}).get('laps', 0) >= 1
                     and 'land' in stages and state['h'] < 0.02)


def test_lisp_manifests_and_validation(report):
    """Shared Lisp block files compile standalone and puzzle into `helicopter.lisp`,
    `quadcopter.lisp`, `ornithopter.lisp`, and `aerobat.lisp`; cooling and error checks pass."""
    from machine import aerobat, aerobatics, aerokin, aerolisp, heat, helicopter, ornithopter, quad
    hm, qm, om, am = helicopter.MANIFEST, quad.MANIFEST, ornithopter.MANIFEST, aerobat.MANIFEST
    mdir = os.path.dirname(os.path.abspath(helicopter.LISP_PATH))
    shared_files = ('aerodynamics.lisp', 'dynamics.lisp', 'kinematics.lisp')
    fns = (aerolisp.compile_aerodynamics, aerolisp.compile_dynamics, aerolisp.compile_kinematics)
    shared = [fn(open(os.path.join(mdir, name), encoding='utf-8').read())
              for fn, name in zip(fns, shared_files)]
    report.check('shared Lisp files compile and puzzle into helicopter, quadcopter, ornithopter, and aerobat',
                 shared[0]['name'] == 'shared' and shared[1]['name'] == 'shared'
                 and shared[2]['kinematics']['name'] == 'shared' and len(shared[2]['stunts']) == 10
                 and all(m['includes'] == shared_files for m in (hm, qm, om, am))
                 and hm['vehicle'] == 'helicopter_3d' and qm['vehicle'] == 'quadcopter'
                 and om['vehicle'] == 'ornithopter_dune' and am['vehicle'] == 'aerobat_3d'
                 and [s.sequence_id for s in hm['streams']] == [301, 302, 303, 304, 305, 306]
                 and [s.sequence_id for s in om['streams']] == [401, 402, 403, 404, 405, 406]
                 and [s.sequence_id for s in am['streams']] == [501, 502, 503, 504, 505, 506]
                 and len(helicopter.CARD) == 24 == len(ornithopter.CARD) == len(aerobat.CARD))

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
                 and abs(scaled_h - 1.5 * 0.75) < 1e-9 and abs(aerobatics.HOVER['height'] - 1.5) < 1e-9)

    st_fan = heat.cooling_state(25.0, quad.COOLING)
    no_fan = dict(quad.COOLING, fan_temp_on=500.0, fan_temp_full=600.0)
    st_nofan = heat.cooling_state(25.0, no_fan)
    for _ in range(400):
        heat.step_cooling(st_fan, 85.0, 25.0, 1.0, 0.0, quad.COOLING)
        heat.step_cooling(st_nofan, 85.0, 25.0, 1.0, 0.0, no_fan)
    st_on = heat.step_board_cooling(heat.cooling_state(25.0, quad.COOLING), 0.0, 63.0, 1.0, 0.0,
                                    quad.COOLING, switching=True)
    st_cut = heat.step_board_cooling(heat.cooling_state(25.0, quad.COOLING), 0.0, 63.0, 1.0, 0.0,
                                     quad.COOLING, switching=False)
    report.check('finned aluminium heatsink + fan cooling model is stable and tracks switching loss',
                 quad.COOLING == helicopter.COOLING == ornithopter.COOLING == aerobat.COOLING
                 and st_fan['switches_c'] > st_fan['sink_c'] > 25.0
                 and st_fan['fan_duty'] > 0.3 and st_fan['sink_c'] + 25.0 < st_nofan['sink_c']
                 and st_on['switches_c'] > st_cut['switches_c'] + 0.5)

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
    """Multi-phase 3D stunt behaviors and compiled `stream_inject` payloads execute in MuJoCo for
    `helicopter`, `ornithopter`, and `aerobat`."""
    from machine import aerobat, aerokin, aerolisp, helicopter, ornithopter, pull
    dt = 1.0 / 120.0
    for craft, s_cpg, s_tum in ((helicopter, 'aileron_tictoc', 'chaos'),
                                (ornithopter, 'wing_rock', 'coriolis_storm'),
                                (aerobat, 'waterfall', 'gyro_tumble')):
        pull_max = craft.K_THRUST * pull.held(pull.I_MAX) ** 2 / craft.MASS_KG
        sky, core, law = craft.Sky(), craft.Core(), aerokin.fresh()
        for _ in range(int(2.5 / dt)):
            ans = core.step(law, craft.STUNTS['high'], sky.state(), dt, pull_max)
            sky.step(ans.speeds + ans.controls, dt)
        cpg_vals, chaos_rates = [], []
        for phases in craft.BEHAVIORS.values():
            for phase in phases:
                row = phase['row']
                for _ in range(int(phase['duration'] / dt)):
                    st = sky.state()
                    ans = core.step(law, row, st, dt, pull_max)
                    sky.step(ans.speeds + ans.controls, dt)
                    if phase['stunt'] == s_cpg:
                        cpg_vals.append(float(st['turn'][1][1]))
                    if phase['stunt'] == s_tum:
                        chaos_rates.append(math.hypot(float(st['spin'][0]), float(st['spin'][1]),
                                                      float(st['spin'][2])))
        st_end = sky.state()
        report.check('%s multi-phase stunt behaviors fly in MuJoCo' % craft.__name__,
                     bool(cpg_vals) and max(cpg_vals) - min(cpg_vals) > 0.6
                     and bool(chaos_rates) and max(chaos_rates) > 6.0
                     and st_end['h'] > 5.0 and sky.hit is None)

    rows = [helicopter.row_from_payload(p) for p in helicopter.STREAMS]
    orows = [ornithopter.row_from_payload(p) for p in ornithopter.STREAMS]
    arows = [aerobat.row_from_payload(p) for p in aerobat.STREAMS]
    roll_cpg = aerolisp.compiled(
        '(stream_inject (cpg_config (enable true) (frequency 1.25Hz) '
        '(amplitude mode_attitude_roll 52.0deg)))')[0]
    row_roll_cpg = helicopter.row_from_payload(roll_cpg)
    report.check('compiled stream_inject payloads #301..#506 and roll CPG translate to stunt rows',
                 rows[0]['height'] == 1.5 and orows[0]['height'] == 1.5 and arows[0]['height'] == 1.5
                 and rows[2]['nose'] == -1.0 and orows[2]['nose'] == -1.0 and arows[2]['nose'] == -1.0
                 and rows[3]['nose'] == -2.0 and orows[3]['nose'] == -2.0 and arows[3]['nose'] == -2.0
                 and row_roll_cpg['nose'] == -3.0 and rows[5]['pace'] == 100.0)


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
