"""The live bus and software diagrams (docs/architecture): her buses (`edge.LIMBS`) and her layers.

    open('docs/architecture/bus_layout.svg').read() == layouts.bus_layout_svg()
    open('docs/architecture/software_layout.svg').read() == layouts.software_layout_svg()
"""
from machine.senses.edge import BAUD, LIMBS, SENSED
from machine.body.figure import JOINTS


def _bus_row_svg(k, name, joints, rtu, drives):
    y0 = 112 + k * 108
    mbps = BAUD / 1e6
    col = ('#38bdf8', '#a78bfa', '#34d399', '#f472b6', '#fbbf24')[k % 5]
    end_x = 260 + len(joints) * 134
    out = [
        '<g font-family="monospace">',
        '<path d="M 220 %d L 250 %d" stroke="%s" stroke-width="2.5" fill="none"/>' % (280, y0 + 28, col),
        '<line x1="250" y1="%d" x2="%d" y2="%d" stroke="%s" stroke-width="3" stroke-dasharray="6,3"/>' % (y0 + 28, end_x, y0 + 28, col),
        '<rect x="250" y="%d" width="175" height="20" rx="4" fill="#0f172a" stroke="%s" stroke-width="1.2"/>' % (y0, col),
        '<text x="258" y="%d" fill="%s" font-size="11" font-weight="bold">BUS %d: %s (%.0f Mbit/s)</text>' % (y0 + 14, col, k + 1, name, mbps),
    ]
    for u, j in enumerate(joints, 1):
        nx = 260 + (u - 1) * 134
        pas = drives.passive(j)
        pk = 0.0 if pas else float(drives.peak(j))
        reg = rtu.SETPOINT_REG + 2 * (u - 1)
        spec = 'passive' if pas else ('%s / %.0f Nm' % (drives.of(j)[0], pk))
        fill = '#1e293b' if not pas else '#0f172a'
        out += [
            '<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="%s" stroke-width="1.8"/>' % (nx + 60, y0 + 28, nx + 60, y0 + 40, col),
            '<rect x="%d" y="%d" width="124" height="54" rx="5" fill="%s" stroke="%s" stroke-width="1.3"/>' % (nx, y0 + 40, fill, col),
            '<text x="%d" y="%d" fill="#f8fafc" font-size="10" font-weight="bold">#%d %s</text>' % (nx + 6, y0 + 54, u, j),
            '<text x="%d" y="%d" fill="#94a3b8" font-size="9">j%02d / reg %#06x</text>' % (nx + 6, y0 + 69, JOINTS.index(j), reg),
            '<text x="%d" y="%d" fill="%s" font-size="9">%s</text>' % (nx + 6, y0 + 83, col, spec),
        ]
    out.append('</g>')
    return '\n'.join(out)


def bus_layout_svg():
    """Render the live 10 Mbit/s RS485 Modbus RTU per-extremity SIL bus architecture (`LIMBS`,
    `BAUD`, `rtu`, `drives`) as a deterministic SVG diagram for `docs/architecture/bus_layout.svg`."""
    from machine.body import drives
    from machine.rig import rtu
    total_boards = sum(len(j) for _, j in LIMBS)
    driven_boards = sum(1 for _, jnts in LIMBS for j in jnts if not drives.passive(j))
    banks = ('SETPOINT %#06x' % rtu.SETPOINT_REG, 'KP       %#06x' % rtu.KP_REG,
             'FEED     %#06x' % rtu.FEED_REG, 'LIMIT    %#06x' % rtu.LIMIT_REG,
             'STATE    %#06x' % rtu.STATE_REG, 'GATE     %#06x' % rtu.GATE_REG)
    head = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1220 665" width="100%" height="100%">',
        '<rect width="1220" height="665" rx="10" fill="#020617"/>',
        '<g font-family="monospace">',
        '<text x="24" y="34" fill="#f8fafc" font-size="16" font-weight="bold">'
        'COAXIAL 63100 -- %dx %.0f Mbit/s RS485 Modbus RTU Extremity Buses (%d SIL Boards, %d Driven)</text>'
        % (len(LIMBS), BAUD / 1e6, total_boards, driven_boards),
        '<text x="24" y="54" fill="#94a3b8" font-size="11">'
        'Generated from machine.senses.edge (LIMBS, BAUD=%d), machine.rig.rtu (CRC-16/MODBUS), and wbc_pass.c (mb_rtu_t + mb_slave_t)</text>' % BAUD,
        '<rect x="20" y="160" width="200" height="245" rx="8" fill="#0f172a" stroke="#38bdf8" stroke-width="2"/>',
        '<text x="32" y="186" fill="#38bdf8" font-size="12" font-weight="bold">SOLVER &amp; SIL MASTER</text>',
        '<text x="32" y="204" fill="#e2e8f0" font-size="10">machine.motion -&gt; edge.drive</text>',
        '<text x="32" y="220" fill="#94a3b8" font-size="10">wbc_sil_drive (1 kHz)</text>',
        '<line x1="32" y1="232" x2="208" y2="232" stroke="#334155" stroke-width="1"/>',
        '<text x="32" y="250" fill="#f8fafc" font-size="10" font-weight="bold">Modbus RTU Banks (0x10):</text>',
    ]
    for i, b in enumerate(banks):
        head.append('<text x="32" y="%d" fill="#cbd5e1" font-size="10">%s</text>' % (268 + i * 18, b))
    head.append('</g>')
    rows = [_bus_row_svg(k, name, jnts, rtu, drives) for k, (name, jnts) in enumerate(LIMBS)]
    return '\n'.join(head + rows + ['</svg>\n'])


def software_layout_svg():
    """Render the live 5-layer gynoid software systems engineering & real-time control architecture
    as a deterministic SVG diagram for `docs/architecture/software_layout.svg`."""
    from machine.law import latent, modal
    from machine.process import sprung
    from machine.aircraft import senses
    law_hz = round(sprung.RATE_HZ / sprung.STACK_EVERY)
    rate_khz = round(sprung.RATE_HZ / 1000.0)
    mbps = round(BAUD / 1e6)
    shared = (
        ('#38bdf8', 'cmd / states  IPC (60 Hz)'),
        ('#a78bfa', 'y_in[%d]     edge.SENSED' % SENSED),
        ('#a78bfa', 'y[%d], fr[%d] modal.SENSED' % (modal.SENSED, modal.FRAME)),
        ('#34d399', 'st[%d], tab  latent.STATE' % latent.STATE),
        ('#34d399', 'out[%d]       latent.OUT' % latent.OUT),
        ('#fbbf24', 'a[%d]         modal.ASK' % modal.ASK),
        ('#fbbf24', 'got[qacc,tau] wrench.solve'),
        ('#f472b6', 'eso[3,21]     %d kHz observer' % rate_khz),
        ('#f472b6', '%dx RS485 RTU  %d Mbit/s SIL' % (len(LIMBS), mbps)),
    )
    layers = (
        ('terminal, outer setpoint &amp; rig front door (60 Hz)', (
            ('show_humanoid.py', 'humanoid_keys / WASDQE', 'chase &amp; orbit cam', 'out: twist, say, charge'),
            ('worker.py &amp; se2.heard', 'outer command -&gt; setpoint', 'asked, twist, heading, tab', 'zero inner mode strings'),
            ('playback.py &amp; apart.py', 'LAG_S = 60 ms / SLICE_S', 'quat slerp &amp; pose lerp', 'in: 60 Hz state snapshots'),
            ('sight.py / scanner.py', 'stereo.py disparity', 'room occupancy &amp; props', 'out: chair / bed / table'),
            ('coaxial/ &amp; mcp/', 'rig.py / 0x6E wire / 8763', 'afe, gates, boot, acquire', 'Kalman &amp; thermal twin'),
        )),
        ('sensing, continuum touch &amp; outward situation (%d Hz, C)' % law_hz, (
            ('edge.py (y_in[%d])' % SENSED, 'IMU + 27 encoders + qm', '4 analog insoles + 28 skin', 'edge.felt &amp; edge.read'),
            ('estimate.py &amp; sense', 'bodied.on_body C-compiled', 'com, vcom, M, h, feet J6', 'out: qpos(34), qvel(33)'),
            ('borne.py &amp; grip.py', 'branchless support span', 'online sole friction mu', 'out: prop, contacts, held'),
            ('touch.py &amp; se2.floored', 'branchless skin admittance', 'insole floor height/pitch', 'out: twist, RAISE, GROUND'),
            ('situation.py (outward)', '6 continuous readings', 'into(S) -&gt; state[situation]', 'outward to UI / LLM only'),
        )),
        ('branchless inner SE(3) &amp; modal continuum (%d Hz, C)' % law_hz, (
            ('ortho.py &amp; *.lisp', 'gait / postures / stunts', '244-byte latent.WIRE rows', 'continuous table schedule'),
            ('latent.py &amp; se2.py', '11 modes + CPG phase THETA', 'continuous twist &amp; gripped', 'out: out[%d] (REF..DS)' % latent.OUT),
            ('support.py (couple)', 'upper-body admittance dq', 'ESO disturbance bleed', 'out: dq(21), PI @ phi'),
            ('impedance.py &amp; poise', 'spanned ZMP + h_axial', 'capture point xi = c+v/om', 'out: a[%d] (STANCE, ACC)' % modal.ASK),
            ('stride.py &amp; modal.py', 'beat, meets, heels, clears', 'SE(3) turns &amp; knee folds', 'out: ask dict for WBC'),
        )),
        ('whole-body solver, surety field &amp; health (%d Hz)' % law_hz, (
            ('wrench.py / wbc_wrench', 'reduced contact-wrench QP', '6D sole + prop contacts', '0/9668 C vs Python diff'),
            ('motion.surety (ukemi)', 'Wendland attitude &amp; support', 'convex Euler-Lagrange brace', 'out: qacc(33), tau(21)'),
            ('reach.py &amp; leap.py', 'analytic leg IK guards', 'airborne capture landing', 'reach.within &amp; reach.land'),
            ('heat.py &amp; drives.py', '2-node Cu/Stator thermal', 'WEP_S = 8.0 s power tank', 'voltage ellipse &amp; SOA'),
            ('sysid.py &amp; wear.py', 'RNE base regressor window', 'payload, friction, lash', 'out: identified.toml'),
        )),
        ('inner ESO, SIL buses &amp; MCU (%d kHz)' % rate_khz, (
            ('wbh_pass_drives (C)', '3rd-order ESO (wo=100)', 'qacc -&gt; ref_v, ref_q, kp', 'dt-scaled leak &amp; bleed'),
            ('wbc_sil.c &amp; rtu.py', '%dx %d Mbit/s RS485 buses' % (len(LIMBS), mbps), 'CRC-16/MODBUS + 27 slaves', 'see: bus_layout.svg'),
            ('physics.py (MuJoCo)', 'figure + mjcf + sneaker', '27 joints, shells &amp; pads', '1 kHz contact &amp; insoles'),
            ('STM32H743 Firmware', 'Cortex-M7 475 MHz (src/)', '35 kHz FOC + CORDIC + ADC', 'DRV8353RS + 12x IPT015N10'),
            ('Stand-In Emulator', 'world_libm.h (277 ns/step)', 'native:// &amp; emulator://', 'bit-identical telemetry'),
        )),
    )
    head = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1280 685" width="100%" height="100%">',
        '<rect width="1280" height="685" rx="10" fill="#020617"/>',
        '<g font-family="monospace">',
        '<text x="24" y="34" fill="#f8fafc" font-size="16" font-weight="bold">'
        'COAXIAL 63100 -- Software Systems Engineering &amp; Real-Time Control Architecture</text>',
        '<text x="24" y="54" fill="#94a3b8" font-size="11">'
        '5-Layer Data-Oriented Pipeline: Outer Setpoint UI (60 Hz) &lt;-&gt; Sensing &amp; Outward Situation (%d Hz) &lt;-&gt; Branchless SE(3)/Modal Continuum (%d Hz) &lt;-&gt; Inner ESO &amp; SIL (%d kHz)</text>'
        % (law_hz, law_hz, rate_khz),
        '<rect x="20" y="150" width="205" height="295" rx="8" fill="#0f172a" stroke="#38bdf8" stroke-width="2"/>',
        '<text x="32" y="176" fill="#38bdf8" font-size="12" font-weight="bold">RUNTIME ORCHESTRATOR</text>',
        '<text x="32" y="194" fill="#e2e8f0" font-size="10">machine.worker / apart</text>',
        '<text x="32" y="210" fill="#94a3b8" font-size="10">sprung.laid + motion.laid</text>',
        '<line x1="32" y1="222" x2="213" y2="222" stroke="#334155" stroke-width="1"/>',
        '<text x="32" y="240" fill="#f8fafc" font-size="10" font-weight="bold">Shared State Arrays:</text>',
    ] + ['<text x="32" y="%d" fill="%s" font-size="9">%s</text>' % (258 + i * 18, c, s) for i, (c, s) in enumerate(shared)] + [
        '<text x="32" y="424" fill="#94a3b8" font-size="9">0 branches in inner core</text>',
        '</g>',
    ]
    rows = [senses._layer_row_svg(k, title, cards) for k, (title, cards) in enumerate(layers)]
    return '\n'.join(head + rows + ['</svg>\n'])



