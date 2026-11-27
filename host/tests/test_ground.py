"""The landscape behind the board (coaxial.graphics.ground): the floor laid at once."""
import sys

from coaxial.graphics import approach, engine, ground, wireframe
from structure_kit import Report
from tools.dev.focus import chosen


def test_the_floor_is_laid_at_once(report):
    """`_lay` lays every segment at once: cell for cell, bit for bit, what `_segment` and
    `_ground_dot` lay one dot at a time, and `_shaded` greys them as `_greys` does - flat and
    rolled by a bank, at two sizes."""

    def dot_by_dot(runs, width, height, roll):
        cells = {}
        for line, points, fade in runs:
            def dot(fx, fy, depth, _k, line=line, fade=fade):
                ground._ground_dot(cells, width, height, fx, fy, depth, line, fade)
            prev = None
            for fx, fy, depth in points:
                if roll is not None:
                    fx, fy = roll(fx, fy)
                if prev is not None:
                    ground._segment(dot, prev, fx, fy, depth, line)
                prev = (fx, fy, depth)
        return cells

    def same(a, b):
        return set(a) == set(b) and all(a[at][:4] == b[at][:4] and a[at][5] == b[at][5]
                                        and len(a[at][4]) == len(b[at][4]) for at in a)

    for width, height in ((60, 20), (108, 44)):
        cam = engine.camera(width, height, 1.5, distance=3.2, zoom=1.0, tip=wireframe.CAMERA_TIP)
        static = ground._ground_static(width, height, 3.2, cam['view'])
        for step, bank, bob in ((0, None, None), (25, 12.0, 0.7), (71, -31.0, -1.2)):
            runs = ground._runs(static, width, height, step)
            roll = (approach.roller({'bank': bank, 'bob': bob}, width, height, static)
                    if bank is not None else None)
            once, slow = ground._lay(runs, width, height, roll), dot_by_dot(runs, width, height, roll)
            laid = ground._laid(runs, width, height, roll)
            shaded = {} if laid is None else {at: (mk, (g, g, g)) for at, mk, g in zip(
                laid[0].tolist(), laid[1].tolist(), ground._shaded(laid, static).tolist())}
            report.check('the floor laid at once is the floor laid dot by dot, %dx%d step %d%s'
                         % (width, height, step, ' rolled' if roll else ''),
                         same(once, slow) and shaded == ground._greys(slow, static),
                         '%d cells against %d' % (len(once), len(slow)))


def test_wave7_c_kernels_match_python(report):
    """`rs_ground_step`, `raster_draw` (`ascii3d`, `cross_section`, `dial`, `thermalmap`,
    `splat_lit`) match their Python/NumPy implementations."""
    import numpy as np
    from coaxial.draw import cross_section, dial, orientation, thermalmap
    from coaxial.graphics import gynoid, lit, shapes
    from tools.cores import raster_board, raster_draw

    report.check('raster_draw C11 library is built and loaded', raster_draw._lib() is not None)
    cam = engine.camera(80, 28, 1.5, distance=3.2, zoom=1.0, tip=wireframe.CAMERA_TIP)
    static = ground._ground_static(80, 28, 3.2, cam['view'])
    step_ok = True
    for step, bank in ((0, None), (13, 11.0), (47, -24.0), (95, None)):
        rl = approach.roller({'bank': bank, 'bob': 0.5}, 80, 28, static) if bank is not None else None
        c_g = raster_board.ground_step_shaded(static, ground._rungs(static, step / ground.RUNG_STEPS),
                                              80, 28, step, 16, rl)
        laid = ground._laid(ground._runs(static, 80, 28, step), 80, 28, rl)
        assert c_g is not None and laid is not None
        step_ok = step_ok and (c_g[0].tolist(), c_g[1].tolist(), c_g[2].tolist()) == (
            laid[0].tolist(), laid[1].tolist(), ground._shaded(laid, static).tolist())
    cam_far = engine.camera(80, 28, 1.5, distance=6.0, zoom=1.0, tip=wireframe.CAMERA_TIP)
    static_far = ground._ground_static(80, 28, 6.0, cam_far['view'])
    c_far = raster_board.ground_step_shaded(static_far, ground._rungs(static_far, 0.0),
                                            80, 28, 0, 16, None)
    laid_far = ground._laid(ground._runs(static_far, 80, 28, 0), 80, 28, None)
    assert c_far is not None and laid_far is not None
    step_ok = step_ok and (c_far[0].tolist(), c_far[1].tolist(), c_far[2].tolist()) == (
        laid_far[0].tolist(), laid_far[1].tolist(), ground._shaded(laid_far, static_far).tolist())
    report.check('rs_ground_step matches _runs + _laid + _shaded across scroll steps', step_ok)

    q = (0.18, 0.31, -0.22, 0.908)
    real_lib = raster_draw._lib
    a3_ok = True
    for tn, wr, cl in ((True, False, True), (True, True, True), (False, False, False)):
        c_txt = orientation.render(q, 64, 22, toon=tn, wire=wr, colour=cl)
        raster_draw._lib = lambda: None
        try:
            p_txt = orientation.render(q, 64, 22, toon=tn, wire=wr, colour=cl)
        finally:
            raster_draw._lib = real_lib
        a3_ok = a3_ok and (c_txt == p_txt)
    report.check('ascii3d C11 raster + resolve match Python bit-for-bit across toon/wire/ascii', a3_ok)

    cs_ok = True
    for deg, blur, amps, tdeg in ((17.5, 0.0, (25.0, -12.0, -13.0), 42.0),
                                  (143.0, 0.45, (0.0, 0.0, 0.0), None)):
        cross_section._SEAT_ARRAYS.clear()
        c_cs = cross_section.render(deg, 24, 28, 60, 24, truth_deg=tdeg, amps=amps, full=40.0,
                                    pointer_deg=deg, colour=True, sweep=14.0, blur=blur)
        cross_section._SEAT_ARRAYS.clear()
        raster_draw._lib = lambda: None
        try:
            p_cs = cross_section.render(deg, 24, 28, 60, 24, truth_deg=tdeg, amps=amps, full=40.0,
                                        pointer_deg=deg, colour=True, sweep=14.0, blur=blur)
        finally:
            raster_draw._lib = real_lib
        cs_ok = cs_ok and (c_cs == p_cs)
    report.check('cross_section C11 seat + body match Python bit-for-bit', cs_ok)

    dl_ok = True
    real_mono = dial.time.monotonic
    dial.time.monotonic = lambda: 10.25
    try:
        for w, h, asp, use_tr in ((58, 21, 2.0, False), (76, 28, 2.3, True)):
            tr_c = dial.trail() if use_tr else None
            tr_p = dial.trail() if use_tr else None
            if tr_c is not None and tr_p is not None:
                dial._glow(tr_c, 1.2, 10.0)
                dial._glow(tr_p, 1.2, 10.0)
            dial._ARRAYS.clear()
            c_dl = dial.instrument(128.4, 520.0, 314.15, width=w, height=h, aspect=asp,
                                   colour=True, trail=tr_c)
            dial._ARRAYS.clear()
            raster_draw._lib = lambda: None
            try:
                p_dl = dial.instrument(128.4, 520.0, 314.15, width=w, height=h, aspect=asp,
                                       colour=True, trail=tr_p)
            finally:
                raster_draw._lib = real_lib
            dl_ok = dl_ok and (c_dl == p_dl)
    finally:
        dial.time.monotonic = real_mono
    report.check('dial C11 sample arrays + frame scatter match Python bit-for-bit', dl_ok)

    nodes = {'phase_u': 64.0, 'phase_v': 57.0, 'mcu': 46.0, 'board': 38.0, 'patch_u': 52.0}
    thermalmap._MASKS.clear()
    c_tm = thermalmap.render(nodes, 38.0, cells=64, colour=True, reserve=0, trailing=0)
    thermalmap._MASKS.clear()
    raster_draw._lib = lambda: None
    try:
        p_tm = thermalmap.render(nodes, 38.0, cells=64, colour=True, reserve=0, trailing=0)
    finally:
        raster_draw._lib = real_lib
    report.check('thermalmap C11 halftone + ramp match Python bit-for-bit', c_tm == p_tm)

    who = gynoid.body()
    pos, nrm = who.pose({'left_hip_z': 15.0, 'right_knee': -25.0}, dense=True)
    m_v = shapes.view(30.0, 12.0)
    fine = engine.fine(engine.camera(64, 24, 0.92, distance=3.2))
    d_c, rgb_c = lit.splat((pos, nrm, who.dense[3], who.dense[2]), m_v, fine, (0.0, 0.84, 0.0))
    raster_draw._lib = lambda: None
    try:
        d_p, rgb_p = lit.splat((pos, nrm, who.dense[3], who.dense[2]), m_v, fine, (0.0, 0.84, 0.0))
    finally:
        raster_draw._lib = real_lib
    rgb_diff = int(np.max(np.abs(rgb_c.astype(np.int32) - rgb_p.astype(np.int32))))
    report.check('splat_lit matches lit.splat 0-ULP depth and <=1 LSB RGBA',
                 np.array_equal(d_c.view(np.uint64), d_p.view(np.uint64)) and rgb_diff <= 1,
                 'rgb max diff %d' % rgb_diff)


def test_fsaa_braille_and_gpu_cpu_parity(report):
    """4x RGSS (4-rooks) Braille FSAA and GPU/CPU lighting parity hold across materials and crafts."""
    import numpy as np
    from coaxial.graphics import card, engine, fsaa, gpu, lit, quadcopter, shapes
    from tools.cores import raster as c_raster, raster_draw
    if c_raster.ensure() is None:
        return

    pose = {'turn': np.eye(3).tolist(), 'at': [0.0, 1.2, 0.0], 'controls': (0.45, 0.1, -0.15, 0.05)}
    rotors = [(0.3, 12.0, 42.0, 38.0, 35.0)] * 4
    m_v = shapes.view(30.0, 18.0)
    fine = engine.fine(engine.camera(80, 28, quadcopter.REACH,
                                     distance=quadcopter.SIGHT * quadcopter.REACH))
    pos, nrm, uvs, mats, idx = quadcopter._posed(pose, rotors, False, 'shell')

    d_c4, c_c4 = fsaa.raster_mesh(pos, nrm, uvs, mats, idx, m_v, fine, pose['at'], fsaa=True)
    d_p4, c_p4 = fsaa.raster_mesh_py(pos, nrm, uvs, mats, idx, m_v, fine, pose['at'], fsaa=True)
    d_c1, _c1 = fsaa.raster_mesh(pos, nrm, uvs, mats, idx, m_v, fine, pose['at'], fsaa=False)
    cov_eq = np.array_equal(d_c4 > 0, d_p4 > 0)
    d_err = float(np.max(np.abs(d_c4 - d_p4)))
    c_err = int(np.max(np.abs(c_c4.astype(np.int32) - c_p4.astype(np.int32))))
    report.check('C11 4x RGSS FSAA triangle raster matches NumPy oracle (<=1 LSB RGBA)',
                 cov_eq and d_err <= 1e-6 and c_err <= 1,
                 'cov_eq=%s d_err=%.2e c_err=%d' % (cov_eq, d_err, c_err))
    report.check('4x RGSS FSAA captures thin struts/skids missed by 1x point sampling',
                 int((d_c4 > 0).sum()) > int((d_c1 > 0).sum()) + 20,
                 '4x=%d vs 1x=%d dots' % (int((d_c4 > 0).sum()), int((d_c1 > 0).sum())))

    fl = np.zeros_like(d_c4)
    _np, _k, _f, _h, _pl, _bt, fl_ink, ld_ink = lit._const()
    bc_c = c_raster.braille_cells(d_c4, c_c4, fl, lit._noise64(), None, None, None,
                                  fl_ink, ld_ink, 80, 28)
    bc_p = lit._braille_cells_py(d_c4, c_c4, fl, None, None, None, fl_ink, ld_ink, 80, 28)
    assert bc_c is not None
    report.check('C11 coverage-aware Braille fold matches _braille_cells_py on 4x FSAA rasters',
                 np.array_equal(bc_c[0], bc_p[0]) and np.array_equal(bc_c[1], bc_p[1])
                 and float(np.max(np.abs(bc_c[2] - bc_p[2]))) <= 1e-12)

    ad = card.adapter()
    if ad is not None:
        lr = gpu.LitRaster(found=ad)
        d_g4, c_g4 = lr.raster(pos, nrm, uvs, mats, idx, m_v, fine, tuple(pose['at']),
                               quadcopter.REACH)
        cg, cc = d_g4 > 0, d_c4 > 0
        jacc = float((cg & cc).sum()) / float(max(1, (cg | cc).sum()))
        both = cg & cc
        rgb_mad = float(np.mean(np.abs(c_g4[both, :3].astype(float) - c_c4[both, :3].astype(float))))
        report.check('GPU LitRaster 4x FSAA agrees with C11 4x FSAA (Jaccard > 0.97, RGB MAD < 2.0)',
                     jacc > 0.97 and rgb_mad < 2.0,
                     'jaccard=%.4f rgb_mad=%.2f' % (jacc, rgb_mad))
    else:
        _ = raster_draw


ROSTER = (test_the_floor_is_laid_at_once, test_wave7_c_kernels_match_python,
          test_fsaa_braille_and_gpu_cpu_parity)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
