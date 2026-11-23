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
        'jaw_dy = float(np.vstack([t for t, _ in neu["mandible"]])[:, 1].min() '
        '               - np.vstack([t for t, _ in spk["mandible"]])[:, 1].min()); '
        'smile_dy = float(np.vstack([t for t, _ in smi["left_lip_corner"]])[:, 1].max() '
        '                 - np.vstack([t for t, _ in neu["left_lip_corner"]])[:, 1].max()); '
        'brow_span = float(np.vstack([t for t, _ in cur["left_brow"]])[:, 1].mean() '
        '                  - np.vstack([t for t, _ in foc["left_brow"]])[:, 1].mean()); '
        'ear_x = float(np.vstack([t for t, _ in neu["left_ear_mic"]])[:, 0].max()); '
        'cranium_x = float(np.vstack([t for t, _ in neu["left_temporal"]])[:, 0].max()); '
        'face_x = float(np.vstack([t for t, _ in neu["left_maxilla"]])[:, 0].max()); '
        'pos, nrm, uv, mat, idx = f.lit_mesh(f.posed("neutral")); '
        'print(json.dumps({"chk": chk, "jaw_mm": round(1e3 * jaw_dy, 2), '
        '"smile_mm": round(1e3 * smile_dy, 2), "brow_mm": round(1e3 * brow_span, 2), '
        '"ear_mm": round(1e3 * ear_x, 2), "cranium_mm": round(1e3 * cranium_x, 2), '
        '"face_mm": round(1e3 * face_x, 2), "bare_keys": sorted(bare.keys()), '
        '"verts": int(len(pos)), "tris": int(len(idx)), "target_b_mm": round(1e3 * stereo.BASELINE_M, 2)}))'
    )
    chk: Any = got.get('chk') or {}
    report.check('face_built.checked passes with zero plate, gimbal, mount and C1 neck crossings',
                 bool(chk.get('ok')) and chk.get('neck_cross') == 0 and chk.get('mount_cross') == 0
                 and chk.get('lid_shut_cross') == 0 and sum((chk.get('preset_cross') or {'x': 1}).values()) == 0,
                 got.get('failed') or chk)
    report.check('stereo camera gimbals match the 120 mm stereo baseline and open-edge count stays zero',
                 abs((chk.get('baseline_mm') or 0.0) - (got.get('target_b_mm') or 120.0)) < 1.0
                 and sum((chk.get('open_edges') or {'x': 1}).values()) == 0,
                 (chk.get('baseline_mm'), chk.get('open_edges')))
    report.check('ovoid cranium crown, narrower facial proportions and flush ear mics match anthropometric targets',
                 225.0 <= (chk.get('crown_mm') or 0.0) <= 232.0
                 and 70.0 <= (got.get('cranium_mm') or 0.0) <= 74.5
                 and 68.0 <= (got.get('face_mm') or 0.0) <= 71.8
                 and 68.5 <= (got.get('ear_mm') or 0.0) <= 72.5
                 and {'chassis', 'gaze_servos', 'gaze_linkages', 'brow_servos', 'lid_servos',
                      'smile_servos', 'jaw_servos'} <= set(got.get('bare_keys') or ()),
                 (chk.get('crown_mm'), got.get('cranium_mm'), got.get('face_mm'), got.get('ear_mm')))
    report.check('jaw, lip-corner smile linkages and brows articulate visibly across the five presets',
                 (got.get('jaw_mm') or 0.0) >= 8.0 and (got.get('smile_mm') or 0.0) >= 2.0
                 and (got.get('brow_mm') or 0.0) >= 3.5 and (got.get('tris') or 0) >= 4000,
                 (got.get('jaw_mm'), got.get('smile_mm'), got.get('brow_mm'), got.get('tris')))


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
