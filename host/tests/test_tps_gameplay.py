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

    report.check('snap counter-strafe right shifts position across',
                 x_left > initial_x + 0.08 and x_right < x_left - 0.08,
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


def test_se2_camera_decoupling_and_body_frame_projection(report):
    """SE(2) body-frame twist and world velocity depend strictly on robot yaw, with zero camera influence."""
    from machine import se2
    from terminal.views import humanoid_keys, show_humanoid, viewpoint

    # 1. Camera yaw/pitch changes have zero effect on SE(2) twist or world velocity
    results = []
    for cam_yaw, cam_pitch in ((0.0, 0.0), (60.0, 12.0), (135.0, -20.0), (-90.0, 35.0), (270.0, 0.0)):
        st = se2.state(yaw=0.4)
        view_state = {**viewpoint.HOME, 'wasd_mode': True, 'yaw': cam_yaw, 'pitch': cam_pitch, 'se2': st}
        keys = se2.poll_input(['w'], st=st)
        twist = se2.step_twist(st, keys, 0.1)
        pose = se2.step_kinematics(st, 0.1)
        results.append((twist, pose, st['v_wx'], st['v_wy'], view_state['yaw']))

    first_twist, first_pose, first_vwx, first_vwy, _ = results[0]
    report.check('camera orientation has zero effect on body twist [v_x, v_y, omega_z]',
                 all(r[0] == first_twist for r in results), str([r[0] for r in results]))
    report.check('camera orientation has zero effect on world velocity and pose',
                 all(r[1] == first_pose and r[2] == first_vwx and r[3] == first_vwy for r in results))

    # 2. Body-to-world transformation rotates strictly by robot yaw
    wx0, wy0 = se2.body_to_world(1.0, 0.0, 0.0)
    wx90, wy90 = se2.body_to_world(1.0, 0.0, math.pi / 2.0)
    report.check('body +X at yaw=0 maps to world (+1, 0) and at yaw=pi/2 maps to (0, +1)',
                 abs(wx0 - 1.0) < 1e-6 and abs(wy0) < 1e-6 and abs(wx90) < 1e-6 and abs(wy90 - 1.0) < 1e-6)

    # 3. Pressing W in terminal view does not inject camera yaw into body heading
    sent = []
    class DummyBody:
        def send(self, **kw):
            sent.append(kw)
    st_view = {**viewpoint.HOME, 'body': DummyBody(), 'wasd_mode': True, 'yaw': 60.0, 'pitch': 12.0,
               'se2': se2.state()}
    humanoid_keys.act_on(show_humanoid.KEYS, ['w'], st_view)
    report.check('pressing W with camera at 60 deg sends forward gait without camera heading override',
                 any(c.get('say') == 3 for c in sent) and not any('heading' in c for c in sent), str(sent))


def test_se2_longitudinal_ramp_decay_and_reverse_clamp(report):
    """Unshifted W walks at 0.69 m/s; Shift+W ('W') sprints up to 1.40 m/s; releasing WASD decays to 0; S backs."""
    from machine import se2

    st = se2.state()
    # Initial unshifted W press walks at WALK_SPEED (0.69 m/s)
    se2.step_controller(st, 'w', 0.0)
    v_initial = st['v_x']
    report.check('W starts at walk speed (0.69 m/s) with zero strafe and zero yaw rate',
                 abs(v_initial - se2.WALK_SPEED) < 1e-6 and st['v_y'] == 0.0 and st['omega_z'] == 0.0,
                 'v_x=%.3f' % v_initial)

    # Holding Shift+W ('W') continuously accelerates toward sprint speed (RUN_SPEED)
    se2.step_controller(st, 'W', 0.35)
    v_mid = st['v_x']
    se2.step_controller(st, 'W', 0.35)
    v_late = st['v_x']
    report.check('holding Shift+W continuously accelerates v_x over time',
                 v_initial < v_mid < v_late <= se2.RUN_SPEED,
                 '%.3f < %.3f < %.3f' % (v_initial, v_mid, v_late))

    # Clamps at RUN_SPEED even when held for a long time
    for _ in range(50):
        se2.step_controller(st, 'W', 0.1)
    report.check('holding Shift+W clamps v_x at maximum run speed (1.40 m/s)',
                 abs(st['v_x'] - se2.RUN_SPEED) < 1e-6, 'v_x=%.3f' % st['v_x'])

    # Releasing W smoothly decays v_x -> 0
    se2.step_controller(st, (), 0.2)
    v_decaying = st['v_x']
    for _ in range(20):
        se2.step_controller(st, (), 0.1)
    report.check('releasing W smoothly decays v_x through intermediate values to 0.0',
                 0.0 < v_decaying < se2.RUN_SPEED and st['v_x'] == 0.0,
                 'decaying=%.3f, final=%.3f' % (v_decaying, st['v_x']))

    # Holding S drives v_x negatively, clamped to lower reverse speed
    for _ in range(10):
        se2.step_controller(st, 's', 0.1)
    report.check('S drives v_x negatively and clamps at lower reverse walk speed (-0.35 m/s)',
                 abs(st['v_x'] + se2.BACK_SPEED) < 1e-6 and se2.BACK_SPEED < se2.WALK_SPEED,
                 'v_x=%.3f' % st['v_x'])

    # Opposing W+S cancels longitudinal input and decays to 0
    for _ in range(10):
        se2.step_controller(st, ['w', 's'], 0.1)
    report.check('simultaneous W+S cancels longitudinal input and decays v_x to 0',
                 st['v_x'] == 0.0, 'v_x=%.3f' % st['v_x'])


def test_se2_isolated_strafe_vs_composite_curvilinear_arcs(report):
    """Isolated A/D produces pure lateral strafe (omega_z=0); composite WA/WD/SA/SD suppresses v_y and turns on an arc."""
    from machine import se2

    # 1. Isolated A and D -> pure lateral velocity, strictly zero yaw rotation
    st_a = se2.state(yaw=0.25)
    for _ in range(10):
        se2.step_controller(st_a, 'a', 0.1)
    report.check('isolated A generates pure left strafe (v_y > 0, v_x == 0, omega_z == 0) and preserves heading',
                 st_a['v_y'] == se2.STRAFE_SPEED and st_a['v_x'] == 0.0 and st_a['omega_z'] == 0.0
                 and abs(st_a['yaw'] - 0.25) < 1e-9,
                 'twist=(%.2f, %.2f, %.2f), yaw=%.3f' % (st_a['v_x'], st_a['v_y'], st_a['omega_z'], st_a['yaw']))

    st_d = se2.state(yaw=-0.40)
    for _ in range(10):
        se2.step_controller(st_d, 'd', 0.1)
    report.check('isolated D generates pure right strafe (v_y < 0, v_x == 0, omega_z == 0) and preserves heading',
                 st_d['v_y'] == -se2.STRAFE_SPEED and st_d['v_x'] == 0.0 and st_d['omega_z'] == 0.0
                 and abs(st_d['yaw'] + 0.40) < 1e-9,
                 'twist=(%.2f, %.2f, %.2f), yaw=%.3f' % (st_d['v_x'], st_d['v_y'], st_d['omega_z'], st_d['yaw']))

    # 2. Transitioning from isolated A to composite WA immediately suppresses v_y -> 0 and enables omega_z > 0
    se2.step_controller(st_a, ['w', 'a'], 0.1)
    report.check('composite WA suppresses lateral strafe (v_y == 0) and maps A to +omega_z',
                 st_a['v_x'] > 0.0 and st_a['v_y'] == 0.0 and st_a['omega_z'] == se2.YAW_RATE
                 and st_a['gait'] == 'turn_left',
                 'twist=(%.2f, %.2f, %.2f), gait=%s' % (st_a['v_x'], st_a['v_y'], st_a['omega_z'], st_a['gait']))

    # 3. Composite WD, SA, SD twist invariants and circular arc trajectory
    st_wd = se2.state(yaw=0.0)
    for _ in range(10):
        se2.step_controller(st_wd, ['w', 'd'], 0.05)
    report.check('composite WD generates forward right arc (v_x > 0, v_y == 0, omega_z < 0, yaw < 0)',
                 st_wd['v_x'] > 0.0 and st_wd['v_y'] == 0.0 and st_wd['omega_z'] == -se2.YAW_RATE
                 and st_wd['yaw'] < -0.20 and st_wd['gait'] == 'turn_right')

    st_sa = se2.state(yaw=0.0)
    se2.step_controller(st_sa, ['s', 'a'], 0.1)
    st_sd = se2.state(yaw=0.0)
    se2.step_controller(st_sd, ['s', 'd'], 0.1)
    report.check('composite SA and SD suppress v_y and map A/D to +/-omega_z with reverse v_x < 0',
                 st_sa['v_x'] == -se2.BACK_SPEED and st_sa['v_y'] == 0.0 and st_sa['omega_z'] == se2.YAW_RATE
                 and st_sd['v_x'] == -se2.BACK_SPEED and st_sd['v_y'] == 0.0 and st_sd['omega_z'] == -se2.YAW_RATE)

    # 4. Sustained chord key-repeats in humanoid_keys never drop out of turn_left into strafe_left
    from terminal.views import humanoid_keys, show_humanoid, viewpoint
    sent = []
    class DummyBody:
        def send(self, **kw):
            sent.append(kw)
    view_st = {**viewpoint.HOME, 'body': DummyBody(), 'wasd_mode': True, 'se2': se2.state()}
    for k in ('w', 'a', 'w', 'a', 'a'):
        humanoid_keys.act_on(show_humanoid.KEYS, [k], view_st)
    says = [c['say'] for c in sent if 'say' in c]
    report.check('sustained W+A key repeats maintain turn_left without dropping to strafe_left',
                 'turn_left' in says and 'strafe_left' not in says, 'says=%s' % says)


def test_se2_mujoco_full_steering_repertoire_without_ragdoll(report):
    """Full MuJoCo closed-loop steering repertoire verifies physical world trajectories and upright stability."""
    import numpy as np
    from machine import figure, modal, se2, sprung

    gym = sprung.entries(name='moves')

    # 1. Verify Windows 560ms typematic delay and hardware held-key polling never drop WASD state
    st_win = se2.state()
    se2.step_controller(st_win, ['w'], 0.02, now=1.00)
    se2.step_controller(st_win, [], 0.55, now=1.55)
    report.check('550ms Windows typematic delay keeps W active without dropping to rest',
                 st_win['keys']['w'] is True and st_win['v_x'] >= se2.WALK_SPEED - 0.15,
                 'v_x=%.3f, keys=%s' % (st_win['v_x'], st_win['keys']))
    se2.step_controller(st_win, [], 0.10, now=2.50, held={'w', 'a'})
    report.check('hardware held={"w", "a"} maintains composite WA arc regardless of repeat stream',
                 st_win['keys']['w'] and st_win['keys']['a'] and st_win['v_y'] == 0.0
                 and st_win['omega_z'] == se2.YAW_RATE)
    se2.step_controller(st_win, [], 0.02, now=2.52, held=set())
    report.check('hardware key release (held=set()) immediately clears active keys on the next frame',
                 not any(st_win['keys'].values()))

    # 2. Verify isolated A/D pure physical strafe, S reverse, WA/WD/SA/SD circular arcs, and rotated heading in MuJoCo
    def run_mode(key, secs=2.5, init_heading=0.0):
        g_m = sprung.laid(every=3)
        if abs(init_heading) > 1e-6:
            q_init = (math.cos(0.5 * init_heading), 0.0, math.sin(0.5 * init_heading), 0.0)
            y0 = float(g_m['world'].data.qpos[1])
            g_m['world'].reset(figure.stand(), where=(0.0, y0, 0.0), turn=q_init)
            g_m['heading'] = init_heading
            s_hd, c_hd = math.sin(init_heading), math.cos(init_heading)
            g_m['fr'][modal.HEADING] = init_heading
            g_m['fr'][modal.ROT] = np.array([[s_hd, 0.0, c_hd], [c_hd, 0.0, -s_hd], [0.0, 1.0, 0.0]]).ravel()
        st_m = se2.state(yaw=init_heading)
        mt = 0.0
        for _ in range(int(secs * 1000)):
            se2.step_controller(st_m, key, 0.001)
            cmd = {'twist': (st_m['v_x'], st_m['v_y'], st_m['omega_z'])}
            if st_m['gait']:
                cmd['say'] = st_m['gait']
            sprung._heard(g_m, gym, cmd)
            sprung.passed(g_m, 0.001)
            mt = max(mt, sprung.tilt(g_m))
        d = g_m['world'].data
        return float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2]), math.degrees(g_m['heading']), mt

    xa, ya, za, hda, mta = run_mode('a', 2.5)
    xd, yd, zd, hdd, mtd = run_mode('d', 2.5)
    report.check('MuJoCo isolated A/D produces physical lateral stepping (xa > 0.12m, xd < -0.10m, tilt < 5 deg)',
                 xa > 0.12 and abs(za) < 0.18 and abs(hda) < 4.0
                 and xd < -0.10 and abs(zd) < 0.18 and abs(hdd) < 4.0 and max(mta, mtd) < 5.0,
                 'A=(%.2f, %.2f, %.1f deg), D=(%.2f, %.2f, %.1f deg)' % (xa, za, hda, xd, zd, hdd))

    xs, ys, zs, hds, mts = run_mode('s', 2.5)
    report.check('MuJoCo isolated S drives physical reverse stepping (dz < -0.12m, tilt < 5 deg)',
                 zs < -0.12 and abs(xs) < 0.18 and mts < 5.0, 'S=(%.2f, %.2f, tilt=%.1f)' % (xs, zs, mts))

    xwa, _, zwa, hdwa, mtwa = run_mode('wa', 2.5)
    xwd, _, zwd, hdwd, mtwd = run_mode('wd', 2.5)
    xsa, _, zsa, hdsa, mtsa = run_mode('sa', 2.5)
    xsd, _, zsd, hdsd, mtsd = run_mode('sd', 2.5)
    report.check('MuJoCo composite WA/WD/SA/SD carve physical arcs with yaw rotation and upright posture',
                 zwa > 0.70 and hdwa > 10.0
                 and zwd > 0.70 and hdwd < -10.0
                 and zsa < -0.10 and hdsa > 5.0
                 and zsd < -0.10 and hdsd < -4.0
                 and max(mtwa, mtwd, mtsa, mtsd) < 5.0,
                 'WA=(%.2f,%.2f,%.1f) WD=(%.2f,%.2f,%.1f) SA=(%.2f,%.2f,%.1f) SD=(%.2f,%.2f,%.1f)'
                 % (xwa, zwa, hdwa, xwd, zwd, hdwd, xsa, zsa, hdsa, xsd, zsd, hdsd))

    xw90, _, zw90, hdw90, mtw90 = run_mode('w', 2.5, init_heading=math.pi / 2.0)
    report.check('MuJoCo W at heading=90 deg walks physically along world +X (world +X > 0.80m, |world Z| < 0.15m)',
                 xw90 > 0.80 and abs(zw90) < 0.15 and mtw90 < 5.0,
                 'pos=(%.2f, %.2f), tilt=%.2f' % (xw90, zw90, mtw90))

    # 3. Continuous multi-mode sequence on a single body
    g = sprung.laid(every=3)
    st = se2.state()
    sequence = (
        ('w', 2.5),
        ('wa', 2.5),
        ('wd', 2.5),
        ('sa', 2.0),
        ('sd', 1.5),
        ('w', 2.0),
        ('a', 1.5),
        ('d', 1.5),
        ('', 2.5),
    )
    max_tilt = 0.0
    min_y = 2.0
    headings = {}
    for key, duration in sequence:
        for _ in range(int(duration * 1000)):
            se2.step_controller(st, key, 0.001)
            gait = st['gait']
            cmd = {'twist': (st['v_x'], st['v_y'], st['omega_z'])}
            if gait:
                cmd['say'] = gait
            sprung._heard(g, gym, cmd)
            sprung.passed(g, 0.001)
            max_tilt = max(max_tilt, sprung.tilt(g))
            min_y = min(min_y, float(g['world'].data.qpos[1]))
        headings[key] = math.degrees(g['heading'])

    final_spd = math.hypot(float(g['world'].data.qvel[0]), float(g['world'].data.qvel[2]))
    report.check('multi-gait SE(2) steering sequence (incl. SA/SD) maintains upright posture (max tilt < 8 deg)',
                 max_tilt < 8.0, 'max_tilt=%.1f deg' % max_tilt)
    report.check('multi-gait SE(2) steering sequence never ragdolls or drops pelvis (min_y > 0.82 m)',
                 min_y > 0.82, 'min_y=%.3f m' % min_y)
    report.check('WA turns heading left (> 12 deg) and WD turns heading back right',
                 headings['wa'] > 12.0 and headings['wd'] < headings['wa'] - 12.0,
                 'headings=%s' % headings)
    report.check('releasing WASD keys decays twist and brakes gynoid to rest (speed < 0.15 m/s)',
                 final_spd < 0.15 and sprung.doing(g) == '#201',
                 'final_spd=%.3f m/s, doing=%s' % (final_spd, sprung.doing(g)))


ROSTER = (test_tactical_strafe_peek_and_fire, test_sprint_dash_and_tactical_backpedal,
          test_mouselook_target_tracking_and_flanking, test_kinetic_strike_and_tps_reticle,
          test_se2_camera_decoupling_and_body_frame_projection,
          test_se2_longitudinal_ramp_decay_and_reverse_clamp,
          test_se2_isolated_strafe_vs_composite_curvilinear_arcs,
          test_se2_mujoco_full_steering_repertoire_without_ragdoll)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())


