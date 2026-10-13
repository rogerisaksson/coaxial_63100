"""Comprehensive third-person shooter (TPS / GTA) gameplay movement and combat test suite.

Simulates realistic gamer combat maneuvers in physics:
1. Tactical strafe dance: Forward advance, peek left (A), counter-strafe right (D), and resume.
2. Sprint dash and tactical backpedal: Shift sprint to cover, then tactical retreat under fire (S).
3. Mouselook aim steering and flanking curves: Camera view rotation actively steers character heading.
4. Kinetic strike and contextual interaction: Spacebar charged melee strike along crosshair and F key interact.
"""
import math
import os
import sys

# Windows OpenBLAS cap before numpy/mujoco
os.environ['OPENBLAS_NUM_THREADS'] = '1'

from tools.dev.focus import chosen
from views_kit import Report


def test_tactical_strafe_peek_and_fire(report):
    """Gamer executes tactical peeking: advance forward, strafe left (A), snap strafe right (D), and resume forward."""
    from machine import figure, sprung

    g = sprung.laid()
    gym = sprung.entries(name='moves')

    # 1. Normal forward advance
    sprung.walked(g)
    for _ in range(3000):
        sprung.passed(g, 0.001)

    initial_x = float(g['world'].data.qpos[0])
    report.check('forward advance established without tilt (< 5 deg)', sprung.tilt(g) < 5.0)

    # 2. Strafe left (A key peek) for 2.0s
    for step in range(100):
        sprung._heard(g, gym, {'say': 'strafe_left'})
        for _ in range(20):
            sprung.passed(g, 0.001)

    x_left = float(g['world'].data.qpos[0])
    tilt_left = sprung.tilt(g)
    y_left = float(g['world'].data.qpos[1])
    hr_l = abs(math.degrees(float(g['world'].data.qpos[g['world'].qadr[figure.JOINTS.index('left_hip_roll')]])))
    hr_r = abs(math.degrees(float(g['world'].data.qpos[g['world'].qadr[figure.JOINTS.index('right_hip_roll')]])))

    report.check('strafe left executes with natural hip roll (< 16 deg)', max(hr_l, hr_r) < 16.0,
                 'max hip roll: %.1f deg' % max(hr_l, hr_r))
    report.check('strafe left maintains standing height (> 0.85m)', y_left > 0.85, 'y: %.3f m' % y_left)
    report.check('strafe left remains upright (tilt < 6 deg)', tilt_left < 6.0, 'tilt: %.1f deg' % tilt_left)

    # 3. Snap counter-strafe right (D key) for 2.0s
    for step in range(100):
        sprung._heard(g, gym, {'say': 'strafe_right'})
        for _ in range(20):
            sprung.passed(g, 0.001)

    x_right = float(g['world'].data.qpos[0])
    tilt_right = sprung.tilt(g)
    y_right = float(g['world'].data.qpos[1])

    report.check('snap counter-strafe right shifts position across', x_right > x_left,
                 'left x: %.3f, right x: %.3f' % (x_left, x_right))
    report.check('counter-strafe maintains standing height (> 0.85m)', y_right > 0.85, 'y: %.3f m' % y_right)
    report.check('counter-strafe remains upright (tilt < 6 deg)', tilt_right < 6.0, 'tilt: %.1f deg' % tilt_right)

    # 4. Resume forward cruise (#204)
    sprung._heard(g, gym, {'say': 3})
    for _ in range(1500):
        sprung.passed(g, 0.001)

    report.check('smooth return to forward advance without falling', sprung.tilt(g) < 6.0)


def test_sprint_dash_and_tactical_backpedal(report):
    """Gamer executes sprint dash to cover (Shift+W) and tactical backpedal retreat (S)."""
    from machine import sprung

    g = sprung.laid()
    gym = sprung.entries(name='moves')

    sprung.walked(g)
    for _ in range(3000):
        sprung.passed(g, 0.001)

    # 1. Shift key sprint dash (jog) for 2.5s
    sprung._heard(g, gym, {'say': 'jog'})
    max_tilt_sprint = 0.0
    for _ in range(2500):
        sprung.passed(g, 0.001)
        max_tilt_sprint = max(max_tilt_sprint, sprung.tilt(g))

    sprint_spd = float(g['world'].data.qvel[2])
    sprint_y = float(g['world'].data.qpos[1])
    report.check('sprint dash achieves forward velocity (>= 0.5 m/s)', sprint_spd >= 0.5, 'speed: %.2f m/s' % sprint_spd)
    report.check('sprint dash maintains upright posture (tilt < 8 deg)', max_tilt_sprint < 8.0, 'max tilt: %.1f deg' % max_tilt_sprint)
    report.check('sprint dash maintains standing height (> 0.86m)', sprint_y > 0.86, 'y: %.3f m' % sprint_y)

    # 2. S key tactical backpedal retreat
    sprung._heard(g, gym, {'say': 4})
    max_tilt_retreat = 0.0
    for _ in range(2000):
        sprung.passed(g, 0.001)
        max_tilt_retreat = max(max_tilt_retreat, sprung.tilt(g))

    report.check('tactical backpedal retreat brakes and steps back stably (tilt < 8 deg)', max_tilt_retreat < 8.0,
                 'max tilt: %.1f deg' % max_tilt_retreat)
    report.check('retreat completes with zero falls', sprung.doing(g) != 'fallen')


def test_mouselook_target_tracking_and_flanking(report):
    """Mouselook view angle actively steers gynoid heading, compound flanking curves carve smoothly."""
    from machine import sprung

    g = sprung.laid()
    gym = sprung.entries(name='moves')

    sprung.walked(g)
    for _ in range(3500):
        sprung.passed(g, 0.001)

    q = g['world'].data.qpos[3:7]
    pyaw0 = 2.0 * math.atan2(float(q[2]), float(q[0]))

    # 1. Turn camera 60 deg left (target_heading)
    target_hd = pyaw0 + math.radians(60.0)
    for i in range(2500):
        if i % 20 == 0:
            sprung._heard(g, gym, {'heading': target_hd})
        sprung.passed(g, 0.001)

    q = g['world'].data.qpos[3:7]
    pyaw_steered = 2.0 * math.atan2(float(q[2]), float(q[0]))
    turned_deg = math.degrees(pyaw_steered - pyaw0)
    report.check('mouselook heading steers character by >= 35 deg', turned_deg >= 35.0, 'turned: %.1f deg' % turned_deg)
    report.check('mouselook steering maintains stability (tilt < 5 deg)', sprung.tilt(g) < 5.0)

    # 2. Compound flank turn (turn_left)
    sprung._heard(g, gym, {'say': 'turn_left'})
    for _ in range(2000):
        sprung.passed(g, 0.001)

    report.check('compound turn executes cleanly without falling', sprung.tilt(g) < 6.0)


def test_kinetic_strike_and_tps_reticle(report):
    """Spacebar charges kinetic push/strike along crosshair line of sight, aiming reticle displays on HUD."""
    from terminal.views import show_humanoid, viewpoint

    state = {**viewpoint.HOME, 'wasd_mode': True, 'yaw': 180.0, 'pitch': 8.0}
    now = {
        'angles': {},
        'where': (0.0, 0.9, 0.0),
        'turn': (1.0, 0.0, 0.0, 0.0),
        'stage': 'walk',
        'speed': 0.6,
    }

    # 1. TPS crosshair reticle rendered
    mark = show_humanoid.marked(state, now, 0.0)
    report.check('TPS aiming crosshair reticle is active', mark is not None and mark[1] == (96, 220, 255))
    aim_pos = mark[0]
    report.check('aiming reticle positioned along line of sight', aim_pos[2] > now['where'][2])

    # 2. Along line of sight calculation
    sight = show_humanoid.along(state)
    report.check('sight vector is unit length', abs(math.hypot(sight[0], sight[1], sight[2]) - 1.0) < 1e-6)

    # 3. Mode badge displays Third Person Control active
    g_text = show_humanoid.gauges(state)
    report.check('gauges render discreet Third Person Control banner',
                 'Third Person Control' in g_text.plain or 'TPW' in g_text.plain)

    # 4. Bar displays TPW action shortcuts
    bar = show_humanoid.bar_for(state)
    actions = [a for _, a in bar]
    report.check('bar includes SPRINT, INTERACT, MOVE, STRIKE',
                 'SPRINT' in actions and 'INTERACT' in actions and 'MOVE' in actions and 'STRIKE' in actions)


ROSTER = (test_tactical_strafe_peek_and_fire, test_sprint_dash_and_tactical_backpedal,
          test_mouselook_target_tracking_and_flanking, test_kinetic_strike_and_tps_reticle)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
