#!/usr/bin/env python3
"""Her floor: the plane under her soles, its grip as she feels it, the surfaces that cap her rhythm."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report


def test_the_floor_runs_on_under_her_soles(report):
    """Level, the floor under each sole is the floor's height to the bit; pitched as her standing
    soles stand, a sole 0.3 m on along her way stands on it 0.3 tan(pitch) higher."""
    import numpy as np
    from machine import modal
    fr, y = np.zeros(modal.FRAME), np.zeros(modal.SENSED)
    fr[modal.ROT] = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float).ravel()
    fr[modal.FLOOR], fr[modal.FLOOR_AT] = 0.0123, (0.0, 0.0)
    y[modal.SOLE] = (0.1, 0.0, 0.0, -0.1, 0.0, 0.3)
    fr[modal.GROUND] = (0.004, -0.006)
    level, rise = modal.under(y, fr, np.array([1.0, 0.0]))
    report.check('a level floor (soft soles read it 0.004 rad) is its height under both soles',
                 (level == fr[modal.FLOOR]).all() and rise == 0.0, str(level))
    fr[modal.GROUND] = (-0.17, -0.17)
    up, rise = modal.under(y, fr, np.array([1.0, 0.0]))
    report.check('up a 10 deg ramp a sole 0.3 m ahead stands 0.3 tan(0.17) higher',
                 abs(up[1] - up[0] - 0.3 * math.tan(0.17)) < 1e-9 and abs(rise - math.tan(0.17)) < 1e-12,
                 '%.4f m' % (up[1] - up[0]))


def test_a_grown_prop_bears_where_it_has_grown(report):
    """A prop grown past its compiled size has its bounds grown with it: a 6 m top bears 2 m from its
    middle, where MuJoCo's cull by its compiled ones left it nothing."""
    import numpy as np
    from machine import floor, physics
    world = physics.World()
    floor.place(world, 'slope', z=0.25, top=6.0)
    m, d = world.model, world.data
    b = m.body('slope_top').id
    g = m.body_geomadr[b]
    world._mj.mj_forward(m, d)
    half = m.geom_size[g][:3]
    report.check('its geom\'s box, its sphere and its body\'s box are its size',
                 np.allclose(m.geom_aabb[g][3:], half) and np.allclose(m.bvh_aabb[m.body_bvhadr[b]][3:], half)
                 and abs(m.geom_rbound[g] - np.linalg.norm(half)) < 1e-9, '%s, %.3f' % (half, m.geom_rbound[g]))


def test_her_grip_bounds_her_rhythm(report):
    """Her text's surfaces cap her rhythm under their grip, its targets kept; at her law's own grip
    nothing moves; a sliding sole's grip is what it used, a holding one's at least that."""
    import numpy as np
    from machine import grip, latent, law, modal, ortho, sprung
    from machine.wbc import MU
    got = {n: (g, c) for n, g, c in ortho.surfaces()}
    report.check('her text has the slippery floor under a quarter of a sole\'s load',
                 got.get('slippery', (None,))[0] == 0.25, str(got))
    try:
        ortho.surfaces('(defsurface s (cpg_config (amplitude mode_foot_rock 0.0)))')
        refused = ''
    except SyntaxError as error:
        refused = str(error)
    report.check('a surface without its grip is refused', len(refused) > 12, refused)
    rng = np.random.default_rng(1)
    out = np.zeros(latent.OUT)
    out[latent.REF], out[latent.RATE], out[latent.ACC] = rng.normal(size=(3, ortho.N))
    out[latent.SWAY], out[latent.PSI], out[latent.TURNS] = 0.6, rng.uniform(0, 6, ortho.N), 5.9
    surfaces, stance = sprung.surfaced(), np.ones(2)
    held = out.copy()
    law.gripped(held, np.full(2, MU), stance, surfaces)
    report.check('at her law\'s own grip her rhythm is the same to the bit', (held == out).all())
    iced = out.copy()
    law.gripped(iced, np.full(2, 0.1), stance, surfaces)
    target = lambda o: o[latent.REF] - o[latent.SWAY] * np.sin(o[latent.PSI])
    report.check('on ice (0.1) her drive sways 0.20 at most, her soles rock none, her targets kept',
                 iced[latent.SWAY][modal.DRIVE] <= 0.2 + 1e-12 and iced[latent.SWAY][modal.ROCK] == 0.0
                 and np.allclose(target(iced), target(out)),
                 'drive %.3f, rock %.3f' % (iced[latent.SWAY][modal.DRIVE], iced[latent.SWAY][modal.ROCK]))
    frames = np.broadcast_to(np.eye(3), (2, 2, 3, 3))
    touch = np.zeros((len(grip.edge.SKIN), 3))
    touch[grip.SOLES[:, 0]] = (8.0, 300.0, 0.0)
    touch[grip.SOLES[:, 0]] = (-8.0, 300.0, 0.0)
    seen, slid = np.zeros((2, grip.SEEN)), np.full(2, MU)
    for _ in range(grip.SEEN):
        slid = grip.felt(slid, seen, frames, touch, np.array([0.5, 0.0, 0.0]), 550.0, 1.0)
    held_on = grip.felt(np.full(2, 0.01), np.zeros((2, grip.SEEN)), frames, touch, np.zeros(3), 550.0, grip.TRUST_S)
    report.check('a sole sliding at one share its last passes has that grip, a holding one at least it',
                 np.allclose(slid, 8.0 / 300.0) and (held_on >= 8.0 / 300.0).all(), '%s, %s' % (slid, held_on))


ROSTER = [test_the_floor_runs_on_under_her_soles, test_a_grown_prop_bears_where_it_has_grown,
          test_her_grip_bounds_her_rhythm]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
