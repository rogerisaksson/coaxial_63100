"""The kernel's clocked sled gaits as her text's phases: a turn her anchors' stroke, then her sled's.

    text = clocked.text('scoot_floor')      # (defbehavior scoot_floor (extends scoot_floor) ..): her descent, its strokes
    texts = clocked.texts()                 # every gait of hers the kernel's clock plays (CLOCKED), for her gym

A turn of the clock (`axial-torus-clock`: its bpm, its limbs' and its sled's duties) is her anchors' stance - her
weight on them the vault's share (`modulate-stick-slip-sled-ratchet`'s gain), her pelvis carried the stroke forward
over them, her hands behind it as far, her sled sliding the stroke in that time - then her sled's, all of her on it,
her anchors carried forward unborne. A segment's bars are BAR turns each. Her units and stiffness her manifest's
(`defrobot.read`); her anchors and sled her told manifolds by their bodies (TOLD).
"""
from machine import dsp, figure, kernel, maneuver

#: Each gait of hers the kernel's clock plays: its patch, the segment of its schedule, her own text's phases it
#: follows, the one its strokes use and her sagittal shift and arms' reach as that one leaves her.
CLOCKED = {'scoot_floor': ('compile-scoot-and-lowcrawl-suite-soa', 0, ('kneel', 'squat', 'tip', 'catch', 'sit'), 'sit',
                           -1.5, -1.32)}
#: A bar's turns of the clock.
BAR = 4
#: Her told manifold for each of the kernel's patches, by the body that bears.
TOLD = {':plantar-bilateral': 'plantar', ':metatarsal-equinus-bilateral': 'plantar',
        ':ulnar-carpal-shove-bilateral': 'palmar', ':palmar-carpal-bilateral': 'palmar',
        ':ischial-bilateral': 'ischial', ':posterior-femoral-bilateral': 'ischial',
        ':ventral-prone-continuum': 'dorsal', ':dorsal-supine-continuum': 'dorsal'}
#: The modes a stroke drives; her arm, shoulder to her fist's heel, m.
SAGITTAL, REACH = 'mode_sagittal_shift', 'mode_arm_reach'
ARM_M = figure.UPPER_ARM + figure.FOREARM + abs(figure.FIST_AT[1]) + figure.FIST_R


def _told(patches, share):
    """{her manifold: share} of `share` over the kernel's `patches`, evenly."""
    out = {}
    for p in patches:
        out[TOLD[p]] = out.get(TOLD[p], 0.0) + share / len(patches)
    return out


def _manifolds(told):
    return ' '.join('(%s %.3f)' % (m, told.get(m, 0.0)) for m in ('plantar', 'ischial', 'palmar'))


def turns(name):
    """(her turns of the clock, s a turn, s its stroke, m a stroke) of gait `name`'s segment (CLOCKED)."""
    patch, at = CLOCKED[name][:2]
    b0, b1, g = dsp.schedule(kernel.read()[patch])[at]
    clock = g['axial-torus-clock']
    turn = 60.0 / clock['bpm']
    return (int(round((b1 - b0) * BAR)), turn, turn * clock['limb-duty'] / (clock['limb-duty'] + clock['axial-duty']),
            g['stick-slip']['stroke-amplitude-m'])


def text(name):
    """(defbehavior NAME (extends NAME) ..) of gait `name` as the kernel's clock plays it (CLOCKED): her own phases,
    then each turn of its segment a stroke and her sled's, then her rest on her sled."""
    patch, at, own, used, sagittal, reach = CLOCKED[name]
    g = dsp.schedule(kernel.read()[patch])[at][2]
    sled = g['stick-slip']
    n, turn, stroke, metres = turns(name)
    modes, _body = maneuver._manifest()
    carried = metres / modes[SAGITTAL][0]
    behind = metres / ARM_M / modes[REACH][0]
    gain = sled['pelvic-unweighting-gain']
    anchored = _told(sled['anchors'], gain)
    for m, v in _told(sled['sleds'], 1.0 - gain).items():
        anchored[m] = anchored.get(m, 0.0) + v
    borne = _told(sled['sleds'], 1.0)
    k, d = modes[SAGITTAL][1:3]
    rows = ['  (phase %s)' % p for p in own]
    last = used
    for t in range(1, n + 1):
        rows.append('  (phase stroke_%d (use %s)\n    (duration %dms) (blend_duration %dms)\n'
                    '    (latent_targets (%s %.3f) (%s %.3f))\n'
                    '    (impedance (stiffness %s %g) (damping %s %g))\n'
                    '    (task_constraints (maintain_zmp (polygon 0)) (slide %.3f) (contact_manifolds %s)))' % (
                        t, last, 1e3 * stroke, 1e3 * stroke, SAGITTAL, sagittal + carried, REACH, reach - behind,
                        SAGITTAL, k, SAGITTAL, d, metres / stroke, _manifolds(anchored)))
        rows.append('  (phase sled_%d (use stroke_%d)\n    (duration %dms) (blend_duration %dms)\n'
                    '    (latent_targets (%s %.3f) (%s %.3f))\n'
                    '    (task_constraints (maintain_zmp (polygon 0)) (slide 0.0) (contact_manifolds %s)))' % (
                        t, t, 1e3 * (turn - stroke), 1e3 * (turn - stroke), SAGITTAL, sagittal, REACH, reach,
                        _manifolds(borne)))
        last = 'sled_%d' % t
    rows.append('  (phase floor_rest (use %s) (duration 60s) (blend_duration 1000ms))' % last)
    return '(defbehavior %s (extends %s)\n%s)\n' % (name, name, '\n'.join(rows))


def texts():
    """Every gait of hers the kernel's clock plays (CLOCKED), as one text for her gym."""
    return '\n'.join(text(name) for name in CLOCKED)
