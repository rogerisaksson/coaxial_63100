"""Her fake camera (`machine.camera`): a look round on its frames - the renderer's parallax (`sight.Eyes.shot`) -
telling the room's furniture where it stands; her map apart telling the very things her map here does, a look
later; on her loop under `physics.SIGHT` her things its map's (`sprung.LOOKS`)."""
import math
import sys

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report
from test_sight import LOOKS, _head, _nearest, _room, _stood


def test_her_fake_camera_tells_the_room(report):
    """Looked round on her fake camera's frames: the chair a seat with a back, the table a seat and the bed a bed
    within 0.3 m of where they stand; her camera apart (`camera.Camera`) tells the things her map here does."""
    from machine import camera, room, sight
    body, world = _room()
    cam = None
    try:
        eyes, keep, cam = sight.Eyes(world), sight.Keep(), camera.Camera(world)
        for k in range(LOOKS + 1):
            _stood(world, math.tau * min(k, LOOKS - 1) / LOOKS)
            if k < LOOKS:
                keep.took(*eyes.shot(world, _head(world)))
            cam.look(world, k * sight.LOOK_S, _head(world))
        here = sight.told(keep)
        for name, affords, at in (('chair', 'chair', room.CHAIR_AT), ('table', 'seat', room.TABLE_AT), ('bed', 'bed', room.BED_AT)):
            found = _nearest(here, affords, at)
            report.check('the %s is told a %s within 0.3 m of where it stands' % (name, affords),
                         found is not None and found[0] < 0.3, '%.2f m off' % found[0] if found else 'not told')
        laid = lambda things: [(t[0], np.round(t[1], 9).tolist(), np.round(t[2], 9).tolist()) for t in things]
        report.check('her camera apart tells the things her map here does, a look later',
                     cam.looks == keep.map.looks and laid(cam.told()) == laid(here),
                     '%d looks apart, %d here; %d things apart, %d here' % (cam.looks, keep.map.looks, len(cam.told()), len(here)))
    finally:
        if cam is not None:
            cam.close()
        body.close()


def test_her_loop_sees_through_it(report):
    """On her loop under `physics.SIGHT` her things are her fake camera's map's: two looks told LOOK_S past the
    second, things among them."""
    from machine import camera, physics, sight, sprung
    was, g = (physics.ROOM, physics.SIGHT), None
    physics.ROOM, physics.SIGHT = 1.0, 1.0
    try:
        g = sprung.laid(sprung.STACK_EVERY)
        while g['t'] < 2.0 * sight.LOOK_S + 0.1:
            sprung.passed(g)
        report.check('on her loop under SIGHT her things are her fake camera\'s: two looks told, things',
                     isinstance(g['keep'], camera.Camera) and g['keep'].looks >= 2 and len(g['things']) >= 1,
                     '%s, %d looks, %d things: %s' % (type(g['keep']).__name__, getattr(g['keep'], 'looks', 0),
                                                    len(g['things']), sorted({b[0] for b in g['things']})))
    finally:
        physics.ROOM, physics.SIGHT = was
        if g is not None and g['keep'] is not None:
            g['keep'].close()


def test_her_vision_panel_holds_her_view(report):
    """Her vision's panel (`scanner.seen`) names what her eyes' view holds alone - a chair ahead, not one 62 deg
    aside nor a door 3.6 m off -; F's offers keep their own reach (`scanner.scan`, test_views_floor)."""
    from terminal.views import scanner
    up = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    props = [('wood', (0.2, 0.45, 1.5), (0.22, 0.02, 0.22), up), ('wood', (1.5, 0.45, 0.8), (0.22, 0.02, 0.22), up),
             ('door', (0.75, 1.05, 3.5), (0.45, 1.05, 0.03), up)]
    now, sc = {'where': (0.0, 0.9, 0.0), 'heading': 0.0, 'props': props}, scanner.VisionScanner()
    seen, offered = ([(d['kind'], round(d['pos'][0], 2)) for d in got] for got in (sc.seen(now), sc.scan(now)))
    report.check("her vision's panel names the chair ahead alone, the offers all three",
                 seen == [('chair', 0.2)] and len(offered) == 3, (seen, offered))


ROSTER = [test_her_fake_camera_tells_the_room, test_her_loop_sees_through_it, test_her_vision_panel_holds_her_view]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
