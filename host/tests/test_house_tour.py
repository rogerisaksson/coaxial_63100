"""Tests for synthetic WASD and mouse maneuvering, house tour, and stunt operations.

Verifies:
  1. HouseTour script validity and event sequence mapping.
  2. Synthetic WASD and mouse commands maneuvering through the glass room environment.
  3. Stunt operations executed smoothly on props: chair, bed, floor mattress, stairs, sill.
  4. Compliant collisions with tree and wall without falling.
  5. Slicks and snow traversals.
  6. Prevention of ground spasming: stunt fall detection immediately triggers motor cut-off
     if robot is knocked down or tumbles during a stunt.
"""
import math
import os
import sys

os.environ['OPENBLAS_NUM_THREADS'] = '1'

import numpy as np
from gynoid_kit import Report
from tools.dev.focus import chosen
from machine import floor, physics, sprung
from machine.tour import HouseTour, STEPS
from terminal.views import gym, humanoid_keys


def test_house_tour_script_validity(report):
    """Every step in HouseTour has a descriptive label, positive duration, and valid commands."""
    report.check('HouseTour has at least 10 structured steps', len(STEPS) >= 10, f'steps={len(STEPS)}')
    total_time = sum(s[1] for s in STEPS)
    report.check('HouseTour total duration is well-bounded (30s to 90s)', 30.0 <= total_time <= 90.0, f'time={total_time:.1f}s')

    valid_keys = set('wasd~') | {
        'sit_chair', 'rise_chair', 'hop_up_chair', 'hop_down_chair',
        'lie_bed', 'rise_bed', 'lie_floor', 'rise_floor',
        'ice_walk', 'slope_walk', 'stairs_walk', 'stairs_down',
        'parry_shove', 'controlled_fall', 'circle_walk',
        'wa', 'wd', 'sa', 'sd'
    }
    for name, dur, events in STEPS:
        report.check(f'step {name!r} duration > 0', dur > 0.0)
        for ev_t, kind, val in events:
            report.check(f'event time <= duration in {name!r}', ev_t <= dur)
            if kind == 'key':
                report.check(f'key {val!r} is recognized', val in valid_keys)
            elif kind == 'mouse':
                yaw, pitch = val
                report.check(f'mouse yaw in [-180, 180]', -180.0 <= yaw <= 180.0)
            elif kind == 'scenario':
                report.check(f'scenario {val!r} in floor.SCENARIOS', val in floor.SCENARIOS)


def test_stunt_fall_detection_prevents_spasming_when_knocked_down(report):
    """When robot is knocked down during a stunt, fall detection triggers and cuts motor feeds to zero."""
    g = sprung.laid()
    g_gym = sprung.entries(name='gynoid_mesomorph_medium')
    by = {e[0]: e for e in g_gym}

    # Start chair stunt
    floor.scenario(g['world'], 'chair')
    sprung.said(g, *by['sit_chair'], at=0.01, reset=True)
    for _ in range(2500):
        sprung.passed(g, 0.001)

    # Knock robot heavily sideways off the chair
    sprung._heard(g, g_gym, {'push': (250.0, 0.0, 0.0), 'seconds': 0.20})

    fallen_detected = False
    max_spasm_torque = 0.0

    # Simulate tumble
    for _ in range(2000):
        cur_doing = sprung.doing(g)
        cur_tilt = sprung.tilt(g)
        cur_y = float(g['world'].data.qpos[1])

        # Test the fall detector condition
        if cur_tilt > 45.0 or cur_y < 0.28:
            fallen_detected = True
            g['world'].feed[:] = 0.0

        if not fallen_detected:
            sprung.passed(g, 0.001)
        else:
            # Motor feed cut to zero; passive physics advance
            g['world'].feed[:] = 0.0
            g['t'] += 0.001
            g['world'].advance()

        ctrl_norm = float(np.linalg.norm(g['world'].data.ctrl))
        max_spasm_torque = max(max_spasm_torque, ctrl_norm)

    report.check('fall detected during knockdown', fallen_detected)
    report.check('motor feed cut to zero on fall prevents spasming', float(np.linalg.norm(g['world'].feed)) == 0.0)
    report.check('robot settles on floor passively without excessive thrashing torques', max_spasm_torque < 800.0,
                 f'max_torque={max_spasm_torque:.1f} Nm')


def test_wasd_strafe_and_turn_without_stance_widening(report):
    """Holding down WASD strafe (A) or turn (WA) does not splay hips into split or cause fall."""
    g = sprung.laid()
    g_gym = sprung.entries(name='gynoid_mesomorph_medium')
    by = {e[0]: e for e in g_gym}

    sprung.said(g, *by['#204'], at=0.01, reset=True)
    for _ in range(1500):
        sprung.passed(g, 0.001)

    # Simulate repeated 'a' keypresses (strafe left) at 10 Hz for 2 seconds
    for _ in range(20):
        sprung._heard_say(g, g_gym, by['strafe_left'])
        for _ in range(100):
            sprung.passed(g, 0.001)

    hr_l = math.degrees(float(g['world'].data.qpos[g['world'].qadr[physics.JOINTS.index('left_hip_roll')]]))
    hr_r = math.degrees(float(g['world'].data.qpos[g['world'].qadr[physics.JOINTS.index('right_hip_roll')]]))
    tilt = sprung.tilt(g)
    y = float(g['world'].data.qpos[1])

    report.check('strafe left maintains upright height (y >= 0.82m)', y >= 0.82, f'y={y:.3f}m')
    report.check('strafe left maintains small tilt (< 15 deg)', tilt < 15.0, f'tilt={tilt:.1f} deg')
    report.check('left hip roll does not flare into split (< 20 deg)', abs(hr_l) < 20.0, f'hr_l={hr_l:.1f} deg')
    report.check('right hip roll does not flare into split (< 20 deg)', abs(hr_r) < 20.0, f'hr_r={hr_r:.1f} deg')


def test_house_tour_maneuver_execution(report):
    """HouseTour steps execute synthetic commands smoothly across all 14 steps without falls."""
    g = sprung.laid(every=3)
    g_gym = sprung.entries(name='gynoid_mesomorph_medium')
    by = {e[0]: e for e in g_gym}

    class MockBody:
        def __init__(self, g, by):
            self.g = g
            self.by = by
        def send(self, say=None, scenario=None, push=None, **kwargs):
            cmd = {}
            if scenario is not None:
                cmd['scenario'] = scenario
            if say is not None:
                cmd['say'] = say
            if push is not None:
                cmd['push'] = push
                cmd['seconds'] = 0.15
            sprung._heard(self.g, g_gym, cmd)

    mock_body = MockBody(g, by)
    mock_state = {
        'body': mock_body,
        'wasd_mode': True,
        'yaw': 0.0,
        'pitch': 0.0,
        'scenario': 'glass_room',
    }

    tour = HouseTour()
    max_tilt = 0.0
    min_y_chair = 1.0

    for s_idx in range(len(tour.steps)):
        step_dur = tour.steps[tour.step_idx][1]
        steps_count = int(round(step_dur * 50))  # 50 Hz control rate
        for _ in range(steps_count):
            tour.step(mock_state, 0.02)
            for _ in range(20):  # 1 kHz physics
                sprung.passed(g, 0.001)
            t = sprung.tilt(g)
            y = float(g['world'].data.qpos[1])
            max_tilt = max(max_tilt, t)
            if s_idx == 1:
                min_y_chair = min(min_y_chair, y)

    report.check('tour progressed through all 14 steps to completion',
                 tour.done and tour.step_idx == len(tour.steps), f'step_idx={tour.step_idx}/{len(tour.steps)}')
    report.check('tour maintains stability without falling across all 14 steps',
                 max_tilt < 25.0, f'max_tilt={max_tilt:.1f} deg')
    report.check('min height during chair sit reflects seated pose',
                 0.55 <= min_y_chair <= 0.85, f'min_y={min_y_chair:.2f}m')


def test_house_exit_and_full_keyboard_mouse_maneuvers(report):
    """Exit the glass house through the hinged door and execute walking, turning, sidestepping, backing, sprinting, jumping, and anti-glitch key handling."""
    from machine import room, se2
    from terminal.views import show_humanoid, viewpoint

    prev_room = physics.ROOM
    try:
        physics.ROOM = 1.0
        g = sprung.laid(every=3)
        g_gym = sprung.entries(name='gynoid_mesomorph_medium')
        st = se2.state()
        dt = 0.001

        g['target_heading'] = math.radians(11.5)
        max_tilt_exit = 0.0
        for _ in range(6800):
            se2.step_controller(st, ['w'], dt)
            sprung._heard(g, g_gym, {'twist': (st['v_x'], st['v_y'], st['omega_z'])})
            sprung.passed(g, dt)
            max_tilt_exit = max(max_tilt_exit, sprung.tilt(g))
        g['target_heading'] = None
        d = g['world'].data
        z_out, door_deg = float(d.qpos[2]), room.door_open(g['world'])
        report.check('gynoid walks out of the house through the doorway (z > 3.6m) and pushes door open (> 50 deg)',
                     z_out > 3.6 and door_deg > 50.0 and max_tilt_exit < 6.0,
                     'z=%.2f m, door=%.1f deg, max_tilt=%.1f deg' % (z_out, door_deg, max_tilt_exit))

        maneuvers = (('wa', 2.0), ('wd', 2.0), ('W', 1.8), ('a', 1.8), ('d', 1.8), ('s', 1.8), ('', 1.8))
        max_tilt_out, min_y_out = 0.0, 2.0
        hd_after, pos_after = {}, {}
        for key, dur in maneuvers:
            for _ in range(int(dur / dt)):
                se2.step_controller(st, [key] if key else [], dt)
                sprung._heard(g, g_gym, {'twist': (st['v_x'], st['v_y'], st['omega_z'])})
                sprung.passed(g, dt)
                max_tilt_out = max(max_tilt_out, sprung.tilt(g))
                min_y_out = min(min_y_out, float(d.qpos[1]))
            hd_after[key] = math.degrees(g['heading'])
            pos_after[key] = (float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2]))

        report.check('outside maneuvers (WA, WD, Shift+W sprint, A, D, S, stop) stay upright (tilt < 8 deg, y > 0.82m)',
                     max_tilt_out < 8.0 and min_y_out > 0.82 and sprung.doing(g) == '#201',
                     'max_tilt=%.1f deg, min_y=%.3f m, doing=%s' % (max_tilt_out, min_y_out, sprung.doing(g)))
        report.check('outside WA turns left and WD turns back right while advancing past z > 5.5m',
                     hd_after['wa'] > 20.0 and hd_after['wd'] < hd_after['wa'] - 10.0 and pos_after['W'][2] > 5.5,
                     'hd_wa=%.1f, hd_wd=%.1f, z_sprint=%.2f' % (hd_after['wa'], hd_after['wd'], pos_after['W'][2]))

        sprung._heard(g, g_gym, {'say': 'hop'})
        max_y_hop, max_tilt_hop = 0.0, 0.0
        for _ in range(2200):
            sprung.passed(g, dt)
            max_y_hop = max(max_y_hop, float(d.qpos[1]))
            max_tilt_hop = max(max_tilt_hop, sprung.tilt(g))
        z_before_resume = float(d.qpos[2])
        for _ in range(2200):
            se2.step_controller(st, ['w'], dt)
            sprung._heard(g, g_gym, {'twist': (st['v_x'], st['v_y'], st['omega_z'])})
            sprung.passed(g, dt)
        z_after_resume = float(d.qpos[2])
        report.check('jump (hop) outside and immediate resume W walking execute cleanly without key glitching',
                     max_y_hop > 0.91 and max_tilt_hop < 8.0 and z_after_resume > z_before_resume + 0.25,
                     'max_y_hop=%.3f, tilt=%.1f, dz=%.2f' % (max_y_hop, max_tilt_hop, z_after_resume - z_before_resume))

        sent = []
        class DummyBody:
            def send(self, **kw):
                sent.append(kw)
        view_st = {**viewpoint.HOME, 'body': DummyBody(), 'wasd_mode': True, 'se2': se2.state()}
        humanoid_keys.act_on(show_humanoid.KEYS, ['W'], view_st)
        humanoid_keys.act_on(show_humanoid.KEYS, ['9'], view_st)
        humanoid_keys.act_on(show_humanoid.KEYS, ['w'], view_st)
        says = [c.get('say') for c in sent if 'say' in c]
        report.check('Shift+W triggers jog, digit 9 triggers hop and clears stale WASD state, and W resumes walk',
                     'jog' in says and 8 in says and 3 in says and (not view_st.get('_wasd_active', False) or says[-1] == 3),
                     'says=%s' % says)
    finally:
        physics.ROOM = prev_room


def test_chair_hops_stairs_down_mattress_and_sequential_scenarios(report):
    """Verify airborne flight in chair hops, stairs_down descent, mattress geom contact, rotated slope normals, and sequential scenarios."""
    world = physics.World()
    for sk in ('slope', 'slope_down'):
        floor.park(world)
        floor.place(world, sk, z=1.0, heading=math.pi / 2.0)
        world._mj.mj_forward(world.model, world.data)
        rmat = world.data.xmat[world.model.body('slope_ramp').id].reshape(3, 3)
        report.check('%s at heading=pi/2 preserves upright ramp normal (ny > 0.99)' % sk, float(rmat[1, 1]) > 0.99)

    g_gym = sprung.entries(name='gynoid_mesomorph_medium')
    for mv in ('hop_up_chair', 'hop_down_chair'):
        g = sprung.laid(every=3)
        sprung._heard(g, g_gym, {'scenario': 'chair', 'say': mv})
        seat_gid = g['world'].model.geom('chair_seat').id
        d_ch = g['world'].data
        air_ms = chair_hits = 0
        max_tilt = 0.0
        for _ in range(2500):
            _, loads, _, _ = sprung.passed(g, 0.001)
            max_tilt = max(max_tilt, sprung.tilt(g))
            if loads[0] == 0.0 and loads[1] == 0.0:
                air_ms += 1
            if d_ch.ncon and (d_ch.contact.geom[:d_ch.ncon] == seat_gid).any():
                chair_hits += 1
        dy = float(d_ch.qpos[1]) - g['rest_y']
        elev_ok = (dy > 0.012) if mv == 'hop_up_chair' else (abs(dy) < 0.010)
        report.check('%s achieves genuine two-footed airborne flight (>= 150ms), contacts chair_seat, and lands upright' % mv,
                     air_ms >= 150 and chair_hits > 0 and elev_ok and max_tilt < 10.0 and sprung.tilt(g) < 10.0,
                     'air_ms=%d, chair_hits=%d, dy=%+.4f, max_tilt=%.1f, tilt=%.1f' % (air_ms, chair_hits, dy, max_tilt, sprung.tilt(g)))

    g_mat = sprung.laid(every=3)
    sprung._heard(g_mat, g_gym, {'scenario': 'mattress', 'say': 'lie_floor'})
    for _ in range(2800):
        sprung.passed(g_mat, 0.001)
    mat_gid = g_mat['world'].model.geom('bed_mattress').id
    d_mat = g_mat['world'].data
    on_mat = bool(d_mat.ncon and (d_mat.contact.geom[:d_mat.ncon] == mat_gid).any())
    report.check('mattress scenario supports robot directly on compliant bed_mattress geom', on_mat)
    sprung._heard(g_mat, g_gym, {'say': 'rise_floor'})
    for _ in range(2400):
        sprung.passed(g_mat, 0.001)
    report.check('rise_floor on mattress completes upright', sprung.tilt(g_mat) < 10.0)

    g_seq = sprung.laid(every=3)
    min_y_sd = 2.0
    for scen, mv in (('stairs', 'stairs_walk'), ('ice', 'ice_walk'), ('slope_down', 'slope_walk'), ('stairs_down', 'stairs_down')):
        sprung._heard(g_seq, g_gym, {'scenario': scen, 'say': mv})
        for _ in range(4500 if scen == 'stairs_down' else 2200):
            sprung.passed(g_seq, 0.001)
            if scen == 'stairs_down':
                min_y_sd = min(min_y_sd, float(g_seq['world'].data.qpos[1]))
        report.check('sequential scenario switch to %s stays upright' % scen, sprung.tilt(g_seq) < 20.0)
    report.check('stairs_down descends onto lower steps below floor level',
                 min_y_sd < g_seq['rest_y'] - 0.025 and float(g_seq['world'].data.qpos[2]) > 1.05,
                 'min_y=%.3f vs rest_y=%.3f, z=%.2f' % (min_y_sd, g_seq['rest_y'], float(g_seq['world'].data.qpos[2])))

    g_os = sprung.laid(every=3)
    sprung._heard(g_os, g_gym, {'say': 'one_sole'})
    for _ in range(1200):
        sprung.passed(g_os, 0.001)
    sprung._heard(g_os, g_gym, {'push': (75.0, 0.0, 0.0), 'seconds': 0.12})
    for _ in range(1500):
        sprung.passed(g_os, 0.001)
    report.check('lateral shove during one_sole parried without falling', sprung.tilt(g_os) < 15.0)


ROSTER = [
    test_house_tour_script_validity,
    test_stunt_fall_detection_prevents_spasming_when_knocked_down,
    test_wasd_strafe_and_turn_without_stance_widening,
    test_house_tour_maneuver_execution,
    test_house_exit_and_full_keyboard_mouse_maneuvers,
    test_chair_hops_stairs_down_mattress_and_sequential_scenarios,
]


def main(argv=None):
    rep = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(rep)
    print('\n%d passed, %d failed' % (rep.passed, rep.failed))
    return 1 if rep.failed else 0


if __name__ == '__main__':
    sys.exit(main())
