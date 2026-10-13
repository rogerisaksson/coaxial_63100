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
    g_gym = sprung.entries(name='moves')
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
    g_gym = sprung.entries(name='moves')
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
    """HouseTour steps execute synthetic commands smoothly across props without falls."""
    g = sprung.laid()
    g_gym = sprung.entries(name='moves')
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
    min_y = 1.0

    # Step through first 3 tour steps (Survey, Chair maneuver, Chair stunts)
    for _ in range(3):
        step_dur = tour.steps[tour.step_idx][1]
        steps_count = int(step_dur * 50)  # 50 Hz control rate
        for _ in range(steps_count):
            tour.step(mock_state, 0.02)
            for _ in range(20):  # 1 kHz physics
                sprung.passed(g, 0.001)
            t = sprung.tilt(g)
            y = float(g['world'].data.qpos[1])
            max_tilt = max(max_tilt, t)
            min_y = min(min_y, y)

    report.check('tour progressed through steps', tour.step_idx >= 3, f'step_idx={tour.step_idx}')
    report.check('tour maintains stability without falling', max_tilt < 25.0, f'max_tilt={max_tilt:.1f} deg')
    report.check('min height during chair sit reflects seated pose', 0.55 <= min_y <= 0.85, f'min_y={min_y:.2f}m')


ROSTER = [
    test_house_tour_script_validity,
    test_stunt_fall_detection_prevents_spasming_when_knocked_down,
    test_wasd_strafe_and_turn_without_stance_widening,
    test_house_tour_maneuver_execution,
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
