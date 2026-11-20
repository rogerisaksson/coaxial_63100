"""The trunk's guards - cups, sternum, belly slats, back - laid clear of the trunk's draft.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/trunk_worn.py OUT.png

The robot's guards on its trunk are the page's (`coaxial.graphics.guards.TRUNK`, where the build's `[guards]
torso` lays them): a cup a side, a sternum's, two slats down the belly, a back's over the shoulder blades and a
plate under that - the hips, the seat and the small of the back wear none, open all the way. Here they are laid on
the figure's torso as the legs' and the arms' are (`leg_worn.worn`): clear of what the chest carries - the trunk's
draft (`trunk_clamped`) and the torso's sketch, the neck's mount and its bay (`neck_mount`) - and of what turns on
the chest through the trunk's travel, the cranks and the rods; each guard's struts on to the spine's tube - or,
where the build clips them on carbon hoops, each hoop's legs in toward it -, stood on what they meet first.
`guards` gives them in the trunk's joint's frame, upright; `crossing` how many of their triangles cross what the
chest carries - none, by the page's rule.
"""
import argparse
import functools
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The trunk's travel's poses the cranks and the rods are swept through for the guards to clear: across the
#: roll's, across the pitch's.
STEPS = (3, 5)


def _carried():
    """{name: tris} of what the chest carries, the trunk's joint's frame, upright: the trunk's chest's parts and the
    torso's sketch - its block, its tube, its drives, its bay, the shoulders' mounts."""
    from tools.render import neck_mount as nm
    from tools.render import trunk_clamped as tc
    from tools.render import trunk_laid as tl
    out = {'trunk ' + name: tris for name, tris in tc.links()['chest'].items() if name != 'tube'}
    for name, tris in nm.posed(low=tc.seat()).items():
        out['neck ' + name] = [(tri - (0.0, tl.RAISED_M, 0.0), c) for tri, c in tris]
    return out


def _swept():
    """[(tri, colour)]: the cranks and the rods through the trunk's travel (STEPS), as the chest sees them."""
    import numpy as np
    from tools.render import hip_frame as hf
    from tools.render import trunk_clamped as tc
    from tools.render import trunk_laid as tl
    (r0, r1), (p0, p1) = tl.travel()
    out = []
    for roll in np.linspace(r0, r1, STEPS[0]):
        for pitch in np.linspace(p0, p1, STEPS[1]):
            chest = tl.turn(float(roll), float(pitch))[1]
            out += [t for (link, _name), tris in tc.posed(float(roll), float(pitch)).items()
                    if 'crank' in link or 'rod' in link for t in hf._put(tris, chest.T)]
    return out


@functools.lru_cache(maxsize=None)
def guards():
    """[(tri, colour)]: the trunk's guards on the figure's torso, the trunk's joint's frame, upright - none where no
    likeness stands beside the tree."""
    import numpy as np
    from coaxial.graphics import likeness
    from tools.render import leg_worn as lw
    from tools.render import trunk_laid as tl
    if not likeness.FOUND:
        return []
    got = lw.worn(_carried(), {'torso': np.array((0.0, -tl.RAISED_M, 0.0))}, {'torso': 'neck spine'},
                  {'torso': _swept()})
    return got.get('torso', [])


def crossing():
    """How many triangle pairs of the trunk's guards cross what the chest carries (`clips`)."""
    from tools.render import clips
    return clips.crossed(guards(), [t for tris in _carried().values() for t in tris])


def main(argv=None):
    from PIL import Image
    from tools.render import mounting as m
    from tools.render import trunk_clamped as tc
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    tris = [t for (link, _name), part in tc.posed().items() for t in part]
    tris += [t for part in _carried().values() for t in part] + guards()
    px, looks = 800, ((20.0, 8.0), (90.0, 4.0), (200.0, 8.0))
    sheet = Image.new('RGB', (px * len(looks), px), (18, 18, 18))
    for i, (yaw, pitch) in enumerate(looks):
        sheet.paste(m.draw(tris, yaw=yaw, pitch=pitch, px=px, half=0.26, centre=(0.0, 0.12, 0.0)), (px * i, 0))
    sheet.save(args.out)
    print(args.out, 'crossing', crossing())
    return 0


if __name__ == '__main__':
    sys.exit(main())
