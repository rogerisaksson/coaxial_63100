"""The robot's parametric mechanical head and expressive face (`tools.render.face_built`)."""
import os
import sys
from typing import Any

from tools.dev.focus import chosen
from views_kit import Report

#: Coaxial humanoid build verified in an isolated subprocess per check.
BUILD = 'gynoid_mesomorph_medium_coax'


def _told(code):
    """Run `code` in an isolated subprocess with `COAXIAL_ROBOT=BUILD` and parse its trailing JSON line."""
    import json
    import subprocess
    host = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env = dict(os.environ, PYTHONPATH=host, COAXIAL_ROBOT=BUILD)
    env.setdefault('OPENBLAS_NUM_THREADS', '1')
    run = subprocess.run([sys.executable, '-X', 'utf8', '-c', 'import json; ' + code],
                         cwd=host, capture_output=True, text=True, env=env)
    return json.loads(run.stdout.strip().splitlines()[-1]) if run.returncode == 0 else {'failed': run.stderr[-500:]}


def test_the_mechanical_face_clears_plates_gimbals_and_neck(report):
    """Verify ovoid cranium, stereo gimbals, flush ear mics, expression linkages, C1 neck clearance and manifolds."""
    got: Any = _told(
        'import numpy as np; '
        'from machine.eyes import gimbal, stereo; '
        'from tools.render import face_built as f; '
        'chk = f.checked(); '
        'neu = f.head_parts(**f.PRESETS["neutral"], plates=True); '
        'spk = f.head_parts(**f.PRESETS["speaking"], plates=True); '
        'smi = f.head_parts(**f.PRESETS["smile"], plates=True); '
        'cur = f.head_parts(**f.PRESETS["curious"], plates=True); '
        'foc = f.head_parts(**f.PRESETS["focused"], plates=True); '
        'bare = f.head_parts(**f.PRESETS["curious"], plates=False); '
        'chin_dy = abs(float(np.vstack([t for t, _ in neu["oled_bezel"]])[:, 1].min() '
        '                    - np.vstack([t for t, _ in spk["oled_bezel"]])[:, 1].min())); '
        'oled_open_dy = float((np.vstack([t for t, _ in spk["oled_mouth_graphic"]])[:, 1].max() '
        '                      - np.vstack([t for t, _ in spk["oled_mouth_graphic"]])[:, 1].min()) '
        '                     - (np.vstack([t for t, _ in neu["oled_mouth_graphic"]])[:, 1].max() '
        '                        - np.vstack([t for t, _ in neu["oled_mouth_graphic"]])[:, 1].min())); '
        'oled_smile_dy = float(np.vstack([t for t, _ in smi["oled_mouth_graphic"]])[:, 1].max() '
        '                      - np.vstack([t for t, _ in neu["oled_mouth_graphic"]])[:, 1].max()); '
        'brow_span = float(np.vstack([t for t, _ in cur["left_brow"]])[:, 1].mean() '
        '                  - np.vstack([t for t, _ in foc["left_brow"]])[:, 1].mean()); '
        'ear_x = float(np.vstack([t for t, _ in neu["left_ear_mic"]])[:, 0].max()); '
        'cranium_x = float(np.vstack([t for t, _ in neu["left_temporal"]])[:, 0].max()); '
        'face_x = float(np.vstack([t for t, _ in neu["left_maxilla"]])[:, 0].max()); '
        'pos, nrm, uv, mat, idx = f.lit_mesh(f.posed("neutral")); '
        'from coaxial.graphics import lit; '
        'from tools.render import face_audit as fa, face_cranium as fc, face_honeycomb as fh, face_laid as fl, face_prints as fp, robot_worn as rw, solids; '
        'pans, spots = fp.assembled_panels(screw_free=True); '
        'm2_open = solids.printable([1e3 * t for t, _ in fc.standard_torx_screw("M2")])["open"]; '
        'm3_open = solids.printable([1e3 * t for t, _ in fc.standard_torx_screw("M3")])["open"]; '
        'eo_pts = 1e3 * np.vstack([t for t, _ in f.eye_oled_local()]); '
        'eo_rxy = np.hypot(eo_pts[:, 0], eo_pts[:, 1]); '
        'eo_open = solids.printable([1e3 * t for t, _ in f.eye_oled_local()])["open"]; '
        'nh_keys = sorted(k for k in fc.neck_hull_panels() if k.startswith("neck_conduit")); '
        'seam_dz = max(abs(fl._eval_wing_z(48.0, 176.0) - fl._eval_brow_z(48.0, 174.0)), '
        '              abs(fl._eval_brow_z(48.0, 150.0) - fl._eval_ocheek_z(48.0, 148.0))); '
        'ma = fa.mesh_and_seam_audit(); '
        'audit_ok = (callable(fa.audit_discrepancies) and seam_dz <= 1.5 and len(fc.temple_prof_loop()) >= 12 '
        '            and ma["porcelain_flipped_nrm"] <= 32 and ma["porcelain_sliver_tris"] <= 32 '
        '            and ma["tiara_crown_gap_mm"] <= 4.8 and ma["occipital_crown_step_mm"] <= 3.5 '
        '            and ma["icheek_nose_step_mm"] <= 1.5 '
        '            and (fc.eval_temple_abs_x(166.5, -17.5) - fc.eval_temple_abs_x(191.5, -17.5)) >= 4.5); '
        'pan_open = sum(solids.printable([1e3 * t for t, _ in ts])["open"] for ts, _ in pans.values()); '
        'm2_qty = sum(1 for _, _, s in spots if s == "M2"); '
        'm3_qty = sum(1 for _, _, s in spots if s == "M3"); '
        'nca = np.asarray([c for _t, c in rw.neck_skin()], np.uint32); '
        'ntm = rw._part_mat("neck skin", nca); '
        'smask = (nca == fh.SHELL_CF).all(1); '
        'ssoft = int((smask & ((ntm & lit.HARD) == 0)).sum()); '
        'dhard = int((~smask & ((ntm & lit.HARD) != 0)).sum()); '
        'print(json.dumps({"chk": chk, "chin_mm": round(1e3 * chin_dy, 3), '
        '"oled_open_mm": round(1e3 * oled_open_dy, 2), "oled_smile_mm": round(1e3 * oled_smile_dy, 2), '
        '"brow_mm": round(1e3 * brow_span, 2), '
        '"ear_mm": round(1e3 * ear_x, 2), "cranium_mm": round(1e3 * cranium_x, 2), '
        '"face_mm": round(1e3 * face_x, 2), "bare_keys": sorted(bare.keys()), '
        '"verts": int(len(pos)), "tris": int(len(idx)), "target_b_mm": round(1e3 * stereo.BASELINE_M, 2), '
        '"shell_n": int(smask.sum()), "shell_soft": ssoft, "dark_hard": dhard, '
        '"pan_n": len(pans), "pan_open": pan_open, "eo_open": eo_open, '
        '"eo_hole_mm": round(float(eo_rxy.min()), 2), "eo_outer_mm": round(float(eo_rxy.max()), 2), '
        '"nh_keys": nh_keys, "audit_ok": audit_ok, "m2_open": m2_open, "m3_open": m3_open, '
        '"m2_qty": m2_qty, "m3_qty": m3_qty}))'
    )
    chk: Any = got.get('chk') or {}
    report.check('face_built.checked passes with zero plate, gimbal, mount and C1 neck crossings',
                 bool(chk.get('ok')) and chk.get('neck_cross') == 0 and chk.get('mount_cross') == 0
                 and chk.get('lid_shut_cross') == 0 and sum((chk.get('preset_cross') or {'x': 1}).values()) == 0,
                 got.get('failed') or chk)
    report.check('stereo camera gimbals match the 64 mm build stereo baseline and open-edge count stays zero',
                 abs((chk.get('baseline_mm') or 0.0) - (got.get('target_b_mm') or 64.0)) < 1.0
                 and abs((got.get('target_b_mm') or 0.0) - 64.0) < 0.5
                 and 'inner_liner' in (chk.get('open_edges') or {})
                 and sum((chk.get('open_edges') or {'x': 1}).values()) == 0,
                 (chk.get('baseline_mm'), got.get('target_b_mm'), chk.get('open_edges')))
    report.check('robot_worn.lit_mesh applies lit.HARD to every SHELL_CF clavicle gorget triangle only',
                 (got.get('shell_n') or 0) > 0 and got.get('shell_soft') == 0 and got.get('dark_hard') == 0,
                 (got.get('shell_n'), got.get('shell_soft'), got.get('dark_hard')))
    report.check('IPD-calibrated cranium, facial porcelain mask and ear housings match robot_face_1 targets',
                 220.0 <= (chk.get('crown_mm') or 0.0) <= 229.0
                 and 84.0 <= (got.get('cranium_mm') or 0.0) <= 89.5
                 and 74.0 <= (got.get('face_mm') or 0.0) <= 82.0
                 and 86.0 <= (got.get('ear_mm') or 0.0) <= 91.5
                 and {'chassis', 'gaze_servos', 'gaze_linkages', 'brow_servos', 'lid_servos',
                      'oled_driver_pcb'} <= set(got.get('bare_keys') or ()),
                 (chk.get('crown_mm'), got.get('cranium_mm'), got.get('face_mm'), got.get('ear_mm')))
    report.check('fixed curved OLED bezel stays stationary while 2D OLED mouth graphic and brows animate across presets',
                 (got.get('chin_mm') if got.get('chin_mm') is not None else 9.0) == 0.0
                 and (got.get('oled_open_mm') or 0.0) >= 3.0
                 and (got.get('oled_smile_mm') or 0.0) >= 1.8
                 and (got.get('brow_mm') or 0.0) >= 3.5 and (got.get('tris') or 0) >= 4000,
                 (got.get('chin_mm'), got.get('oled_open_mm'), got.get('oled_smile_mm'), got.get('brow_mm'), got.get('tris')))
    report.check('screw-free panels, curved eye OLED displays and standalone M2/M3 Torx screws stay 100% watertight',
                 (got.get('pan_n') or 0) >= 85 and got.get('pan_open') == 0 and got.get('eo_open') == 0
                 and 2.4 <= (got.get('eo_hole_mm') or 0.0) <= 3.2 and 12.5 <= (got.get('eo_outer_mm') or 0.0) <= 16.5
                 and len(got.get('nh_keys') or ()) == 3 and bool(got.get('audit_ok'))
                 and got.get('m2_open') == 0 and got.get('m3_open') == 0
                 and (got.get('m2_qty') or 0) >= 80 and got.get('m3_qty') == 2,
                 (got.get('pan_n'), got.get('pan_open'), got.get('eo_open'), got.get('eo_hole_mm'), got.get('eo_outer_mm'), got.get('audit_ok')))


ROSTER = (test_the_mechanical_face_clears_plates_gimbals_and_neck,)


def main(argv=None):
    """Run every test in ROSTER or the subset selected by `tools.dev.focus.chosen`."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
