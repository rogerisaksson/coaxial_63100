"""Her fake camera (`machine.eyes.camera`): a look round on its frames - the renderer's parallax (`sight.Eyes.shot`) -
telling the room's furniture where it stands; her map apart telling the very things her map here does, a look
later; on her loop under `physics.SIGHT` her things its map's (`vision.LOOKS`); and what her eyes take things for
(`machine.eyes.classes`): a house's things by their boxes, her registry's slots keeping it, her picture past her map."""
import math
import sys

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report
from test_sight import LOOKS, _head, _nearest, _room, _stood

#: A box upright: (kind, centre, half sizes, turn, its slot or her eyes' id, whether the space over it is seen clear).
UP = ((1, 0, 0), (0, 1, 0), (0, 0, 1))


def _box(kind, centre, half, clear=1.0, slot=0):
    return (kind, centre, half, UP, slot, clear)


def test_her_fake_camera_tells_the_room(report):
    """Looked round on her fake camera's frames: the chair a seat with a back, the table a seat and the bed a bed
    within 0.3 m of where they stand; her camera apart (`camera.Camera`) tells the things her map here does; her
    things told by class, none by material, her picture the floor under her map's (`classes`, `sight.Keep.hulls`)."""
    from machine.eyes import camera, classes, sight
    from machine.world import room
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
        materials = {look for look, told in classes.LOOKED.items() if told is None}
        named, shown = {n for n in classes.named(here) if n}, {k for k, _i, _h in keep.hulls()}
        report.check('her map tells her things by class, none by its material; her picture names the floor',
                     bool(named) and not named & materials and 'floor' in shown and not shown & materials,
                     '%s; picture %s' % (sorted(named), sorted(shown)))
    finally:
        if cam is not None:
            cam.close()
        body.close()


def test_the_lodge_told_as_a_house(report):
    """The lodge's own boxes (`world.props()`, her eyes' stand-in) told as a house is (`classes.of`): its walls,
    windows, door, railing, roof, veranda, terrace, stairs, loft, chairs, table, bed, lamp and switch, none by
    material; the lot a cabin (`classes.buildings`)."""
    from machine.eyes import classes
    body, world = _room()
    try:
        world._mj.mj_forward(world.model, world.data)
        props = world.props()
        told, built = set(classes.of(*classes.stacked(props))[0]), [b[0] for b in classes.buildings(props)]
    finally:
        body.close()
    house = {'wall', 'window', 'door', 'railing', 'roof', 'veranda', 'terrace', 'stairs', 'platform', 'chair', 'table',
             'bed', 'lamp', 'switch'}
    materials = {look for look, kind in classes.LOOKED.items() if kind is None}
    report.check('the lodge told as a house, none of it by material, the lot a cabin',
                 house <= told and not told & materials and built == ['cabin'], '%s; %s' % (', '.join(sorted(told)), built))


#: A cabin of boxes and what lies about it, each with what her eyes should take it for: four timber walls, the front's
#: a window and a door in it, a roof, its floor; outside, a veranda along its front, a stone terrace by it, a rail, a
#: stair down off the veranda and the ground under.
CABIN = (('wall', _box('timber', (0.0, 1.2, 0.0), (3.0, 1.2, 0.05), 0.0)),
         ('wall', _box('timber', (-3.0, 1.2, 2.0), (0.05, 1.2, 2.0), 0.0)),
         ('wall', _box('timber', (3.0, 1.2, 2.0), (0.05, 1.2, 2.0), 0.0)),
         ('wall', _box('timber', (-2.25, 1.2, 4.0), (0.75, 1.2, 0.05), 0.0)),
         ('window', _box('glass', (-1.0, 1.2, 4.0), (0.5, 1.2, 0.01), 0.0)),
         ('door', _box('door', (0.0, 1.0, 4.0), (0.45, 1.0, 0.02), 0.0)),
         ('wall', _box('timber', (1.75, 1.2, 4.0), (1.25, 1.2, 0.05), 0.0)),
         ('roof', _box('roof', (0.0, 2.6, 2.0), (3.4, 0.08, 2.4))),
         ('floor', _box('wood', (0.0, -0.03, 2.0), (2.9, 0.03, 1.9))),
         ('veranda', _box('wood', (0.0, -0.03, 5.0), (3.0, 0.03, 0.95))),
         ('terrace', _box('stone', (-3.8, 0.1, 5.0), (0.7, 0.1, 0.95))),
         ('railing', _box('rail', (1.5, 1.0, 5.95), (1.5, 0.03, 0.03))),
         ('stairs', _box('wood', (-0.5, -0.17, 6.1), (0.5, 0.02, 0.15))),
         ('stairs', _box('wood', (-0.5, -0.32, 6.4), (0.5, 0.02, 0.15))),
         ('stairs', _box('wood', (-0.5, -0.47, 6.7), (0.5, 0.02, 0.15))),
         ('ground', _box('stone', (0.0, -0.63, 8.5), (3.0, 0.03, 1.65))))


def test_a_cabin_told_from_outside(report):
    """A cabin and what lies about it, each from its box (CABIN, `classes.of`): its walls, window, door, roof and floor,
    outside it its veranda, terrace, rail, a stair down and the ground; the lot one cabin (`classes.buildings`); her
    vision's panel names it from its veranda, not from inside it (`scanner.seen`), and each thing about her once - the
    front's wall either side of its window and door one, the stair's treads one stair (`scanner.scan`)."""
    import collections
    from machine.eyes import classes
    from terminal.views import scanner
    told, boxes = [t for t, _b in CABIN], [b for _t, b in CABIN]
    names, built = classes.of(*classes.stacked(boxes))[0], classes.buildings(boxes)
    wrong = ['%d %s: %s' % (i, t, n) for i, (t, n) in enumerate(zip(told, names)) if t != n]
    report.check('a cabin and what lies about it told by their boxes, the lot one cabin',
                 not wrong and [b[0] for b in built] == ['cabin'], '; '.join(wrong) or str(built))
    outside, inside = ({'where': where, 'heading': heading, 'props': boxes}
                       for where, heading in (((-0.5, 0.9, 5.6), math.pi), ((0.0, 0.9, 2.0), 0.0)))
    out, within = ({d['kind'] for d in scanner.VisionScanner().seen(now)} for now in (outside, inside))
    report.check("her panel names the cabin, its door and window from its veranda, the cabin not from inside it",
                 {'cabin', 'door', 'window'} <= out and 'cabin' not in within, '%s; %s' % (sorted(out), sorted(within)))
    about = collections.Counter(d['kind'] for d in scanner.VisionScanner().scan(outside, reach=8.0))
    report.check('her panel names each thing about her once, the cabin its four walls',
                 about['wall'] == 4 and all(n == 1 for k, n in about.items() if k != 'wall'), str(dict(about)))


def test_the_lodge_from_its_deck(report):
    """Out on the lodge's deck (`floor.scenario` 'lodge') after a look round inside, two looks each way - back, along the
    deck, back again, its facade and a corner seen -: her picture tells the cabin, its door and its windows looking back,
    the veranda, the ground and the rail looking along the deck, no material (`sight.Keep.hulls`); the stand-in's panel
    names the cabin and its door there (`scanner.seen`); her picture back one door and one window - the glass gable's
    panes -, the stand-in's panel within 8 m one window, door, stair down, roof, lamp and rail (`scanner.scan`)."""
    import collections
    from machine.body import figure
    from machine.eyes import classes, sight
    from machine.world import floor, room
    from terminal.views import scanner
    body, world = _room()
    try:
        floor.scenario(world, 'lodge')
        world._mj.mj_forward(world.model, world.data)
        props = world.props()
        eyes, keep, shown = sight.Eyes(world), sight.Keep(), {}
        for k in range(LOOKS):
            _stood(world, math.tau * k / LOOKS)
            keep.took(*eyes.shot(world, _head(world)))
        for side, heading in (('back', math.pi), ('along', -math.pi / 2), ('back', math.pi)):
            world.reset(figure.stand(), where=(room.STAIR_X, figure.standing(), 5.2),
                        turn=(math.cos(heading / 2.0), 0.0, math.sin(heading / 2.0), 0.0))
            for _ in range(2):
                keep.took(*eyes.shot(world, _head(world)))
            shown[side] = collections.Counter(h[0] for h in keep.hulls())
    finally:
        body.close()
    materials = {look for look, kind in classes.LOOKED.items() if kind is None}
    report.check('from the deck her picture tells the cabin, its door and windows back, the veranda, ground and rail along',
                 {'cabin', 'door', 'window'} <= set(shown['back']) and {'veranda', 'ground', 'railing'} <= set(shown['along'])
                 and not (set(shown['back']) | set(shown['along'])) & materials, str(shown))
    panel = {d['kind'] for d in scanner.VisionScanner().seen({'where': (room.STAIR_X, 0.9, 5.2), 'heading': math.pi,
                                                               'props': props})}
    report.check("the stand-in's panel on the deck names the cabin and its door", {'cabin', 'door'} <= panel,
                 str(sorted(panel)))
    about = collections.Counter(d['kind'] for d in scanner.VisionScanner().scan(
        {'where': (room.STAIR_X, 0.9, 5.2), 'heading': math.pi, 'props': props}, reach=8.0))
    report.check('one thing one: back one door and one window; within 8 m one window, door, stair down, roof, lamp, rail',
                 shown['back']['door'] == shown['back']['window'] == 1
                 and all(about[k] == 1 for k in ('window', 'door', 'stairs_down', 'roof', 'lamp', 'railing')),
                 '%s; %s' % (dict(shown['back']), dict(about)))


def test_her_loop_sees_through_it(report):
    """On her loop under `physics.SIGHT` her things are her fake camera's map's: two looks told LOOK_S past the
    second, things among them."""
    from machine.eyes import camera, sight
    from machine.world import physics
    from machine.process import sprung
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
             ('door', (0.75, 1.05, 3.5), (0.45, 1.05, 0.03), up),
             ('wood', (0.2, 0.72, 1.72), (0.22, 0.25, 0.02), up), ('wood', (1.5, 0.72, 1.02), (0.22, 0.25, 0.02), up)]
    now, sc = {'where': (0.0, 0.9, 0.0), 'heading': 0.0, 'props': props}, scanner.VisionScanner()
    seen, offered = ([(d['kind'], round(d['pos'][0], 2)) for d in got] for got in (sc.seen(now), sc.scan(now)))
    report.check("her vision's panel names the chair ahead alone, the offers all three",
                 seen == [('chair', 0.2)] and len(offered) == 3, (seen, offered))


def test_her_page_keeps_its_pace(report):
    """Her page on her worker's state (the lodge, the stand-in's things): the page's process names none of her
    things - her worker does (`now['named']`, `sprung.named`) - and a frame's scanner work, F's offers, the
    prompt and her vision's panel, takes under a quarter of a frame at FPS_CAP: named there, 59 things took 12 ms
    a scan and the page drew 7 frames a second."""
    import time
    from machine.way import affordance
    from machine.world import physics
    from machine.process import worker
    from terminal.ui.screen import FPS_CAP
    from terminal.views import gym, scanner
    was = (physics.ROOM, physics.SIGHT)
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    try:
        s = worker.session()
        for _ in range(400):
            worker.stepped(s)
        now = worker.told(s)
    finally:
        physics.ROOM, physics.SIGHT = was
    sc = scanner.VisionScanner()
    st, calls, kept = {'now': now, 'scanner': sc, 'tour': None}, [0], {}
    for name in ('sits', 'lies', 'backed'):
        kept[name] = getattr(affordance, name)
        setattr(affordance, name, lambda *a, _f=kept[name]: calls.__setitem__(0, calls[0] + 1) or _f(*a))
    took = []
    try:
        for _ in range(20):
            t0 = time.perf_counter()
            scanner.docked_vision(sc, now, 200, 60, gym.offered(st))
            gym.prompt(st)
            sc.seen(now)
            took.append(time.perf_counter() - t0)
    finally:
        for name, f in kept.items():
            setattr(affordance, name, f)
    frame_ms = 1e3 * sorted(took)[len(took) // 2]
    report.check('the page names none of her %d things, her worker has: %d named' % (len(now['things']), sum(map(bool, now['named']))),
                 calls[0] == 0 and len(now['named']) == len(now['groups']) == len(now['things']), '%d calls' % calls[0])
    report.check("a frame's scanner work under a quarter of a frame at %.0f a second" % FPS_CAP,
                 frame_ms < 250.0 / FPS_CAP, '%.2f ms, a quarter %.1f' % (frame_ms, 250.0 / FPS_CAP))


#: A house of boxes, each with what her eyes should take it for and the box it is a piece of (`classes.of`'s whole, None
#: its own): the faces of what has no top seen clear over it (clear 0), the tops, a chair's back and leg.
HOUSE = (('wall', _box('timber', (3.0, 1.2, 0.0), (0.05, 1.2, 2.0), 0.0), None),
         ('window', _box('glass', (0.0, 1.2, 3.0), (1.0, 1.2, 0.01), 0.0), None),
         ('window', _box('timber', (1.05, 1.2, 3.0), (0.05, 1.2, 0.05), 0.0), None),       # the post framing the pane
         ('door', _box('door', (-1.5, 1.0, 3.0), (0.44, 1.0, 0.02), 0.0), None),
         ('wall', _box('door', (2.95, 2.2, 1.0), (0.02, 0.15, 0.5), 0.0), None),          # a door's colour high on a wall
         ('ceiling', _box('timber', (0.0, 2.6, 0.0), (1.0, 0.05, 1.0), 0.0), None),
         ('railing', _box('glass', (0.0, 0.7, -4.0), (1.0, 0.2, 0.01), 0.0), None),
         ('railing', _box('rail', (0.0, 1.0, -4.1), (1.0, 0.03, 0.03)), None),
         ('post', _box('timber', (-3.0, 1.2, -3.0), (0.05, 1.2, 0.05), 0.0), None),
         ('roof', _box('roof', (0.0, 3.5, 0.0), (3.0, 0.08, 3.0)), None),
         ('stairs', _box('wood', (6.0, 0.075, 0.0), (0.5, 0.075, 0.15)), None),
         ('stairs', _box('wood', (6.0, 0.15, -0.3), (0.5, 0.15, 0.15)), None),
         ('stairs', _box('wood', (6.0, 0.225, -0.6), (0.5, 0.225, 0.15)), None),
         ('platform', _box('wood', (6.0, 0.3, -1.25), (0.5, 0.3, 0.5)), None),               # the landing they reach
         ('step', _box('stone', (9.0, 0.1, 0.0), (0.3, 0.1, 0.15)), None),
         ('chair', _box('wood', (12.0, 0.45, 0.0), (0.22, 0.02, 0.22)), None),
         ('chair', _box('wood', (12.0, 0.72, 0.22), (0.22, 0.25, 0.02)), 15),                  # its back
         ('chair', _box('wood', (12.18, 0.215, -0.18), (0.02, 0.215, 0.02)), 15),              # a leg
         ('sofa', _box('wood', (12.0, 0.45, 4.0), (0.9, 0.02, 0.4)), None),
         ('sofa', _box('wood', (12.0, 0.72, 4.4), (0.9, 0.25, 0.02)), 18),
         ('table', _box('wood', (16.0, 0.74, 0.0), (0.6, 0.015, 0.35)), None),
         ('table', _box('wood', (16.55, 0.3625, 0.3), (0.02, 0.3625, 0.02)), 20),
         ('table', _box('wood', (16.0, 0.42, 4.0), (0.6, 0.02, 0.3)), None),                   # a low one
         ('bench', _box('wood', (20.0, 0.45, 0.0), (0.8, 0.02, 0.2)), None),
         ('stool', _box('wood', (20.0, 0.45, 4.0), (0.18, 0.02, 0.18)), None),
         ('cabinet', _box('wood', (24.0, 1.0, 0.0), (0.5, 1.0, 0.3)), None),
         ('shelf', _box('wood', (24.0, 1.5, 4.0), (0.4, 0.02, 0.15)), None),
         ('thing', _box('wood', (28.0, 0.26, 0.0), (0.3, 0.01, 0.3)), None),
         ('bed', _box('bed', (28.0, 0.25, 4.0), (0.45, 0.2, 1.0)), None),
         ('lamp', _box('lamp', (32.0, 0.75, 0.0), (0.02, 0.75, 0.02)), None),
         ('lamp', _box('lit', (32.0, 1.6, 0.0), (0.15, 0.12, 0.15)), None),
         ('switch', _box('switch', (34.0, 1.05, 0.0), (0.02, 0.04, 0.03)), None),
         ('ground', _box('wood', (40.0, -0.03, 0.0), (1.0, 0.03, 1.0)), None),               # flush, far from the house
         ('rug', _box('rug', (44.0, 0.01, 0.0), (0.3, 0.01, 0.6)), None),
         ('wall', _box('wood', (2.95, 1.5, -1.0), (0.05, 0.05, 0.05), 0.0), None))           # a wood sliver on the wall


def test_a_house_told_by_its_boxes(report):
    """What her eyes take a house's things for (`classes.of`), each from its box (HOUSE): walls, a pane past her brace
    and the post framing it a window, a door standing on the floor and its colour high on a wall none, a ceiling,
    railings, a free post, a stair's treads and the landing they reach, a lone step, a chair, a sofa, a table high and
    low, a bench, a stool, a cabinet, a shelf, a bed, a lamp lit or not, a switch, the ground far from the house its
    facade and its wall make, a scene's rug, a sliver
    of another colour on a wall the wall's; a chair's
    back and leg, a sofa's back and a table's leg the top's and unnamed (`classes.named`); her classes told again
    stay; what is of one thing one - a pane and its post, a wall and what lies on it, a stair's treads, a chair's
    pieces -, a door, a landing, a second table and two windows a wall apart apart (`classes.read`)."""
    from machine.eyes import classes
    told, boxes = [t for t, _b, _w in HOUSE], [b for _t, b, _w in HOUSE]
    names, whole = classes.of(*classes.stacked(boxes))
    pieces = [i if w is None else w for i, (_t, _b, w) in enumerate(HOUSE)]
    wrong = ['%d %s: %s' % (i, t, n) for i, (t, n) in enumerate(zip(told, names)) if t != n]
    report.check('a house told by its boxes, %d things, %d classes, each piece its whole\'s' % (len(boxes), len(set(told))),
                 not wrong and whole.tolist() == pieces, '; '.join(wrong) or 'wholes %s' % whole.tolist())
    named = classes.named(boxes)
    unnamed = [i for i, n in enumerate(named) if n is None]
    report.check('a piece of furniture other than its top and what she cannot tell unnamed, the rest named',
                 unnamed == [16, 17, 19, 21, 27] and all(n == t for n, t in zip(named, told) if n is not None), str(unnamed))
    again = classes.of(names, *classes.stacked(boxes)[1:])[0]
    report.check('her classes told again stay', again == names, str([(a, b) for a, b in zip(names, again) if a != b]))
    ones = classes.read(boxes)[1]
    one = lambda *at: len({ones[i] for i in at}) == 1
    joined = ((1, 2), (0, 4, 34), (6, 7), (10, 11, 12), (15, 16, 17), (18, 19), (20, 21), (29, 30))
    apart = classes.read([_box('timber', (0.0, 1.2, 9.0), (0.5, 1.2, 0.05), 0.0)]
                         + [_box('glass', (x, 1.2, 9.0), (0.5, 0.6, 0.01), 0.0) for x in (-1.0, 1.0)])[1]
    report.check('what is of one thing one; a door, a landing, a second table, two windows a wall apart apart',
                 all(one(*at) for at in joined) and not any(one(*at) for at in ((1, 3), (12, 13), (20, 22)))
                 and apart[1] != apart[2], '%s; %s' % (ones, apart))


def test_her_registry_keeps_what_each_is(report):
    """Her registry's slots hold what each thing is (`registry.seen`, `classes.of`): a seat first seen bare a stool, its
    slot a chair once its back shows - her eyes' id keeping the slot, its class its thing's last -; what stands is the
    text's `structural-lock-classes`, what a house is built of."""
    from machine.eyes import registry
    head = {'pos': np.array([0.0, 1.6, 0.0]), 'quat': np.array([1.0, 0.0, 0.0, 0.0])}
    seat = _box('wood', (0.0, 0.45, 2.0), (0.22, 0.02, 0.22), 1.0, 7)
    back = _box('wood', (0.0, 0.72, 2.22), (0.22, 0.25, 0.02), 0.0, 8)
    slots = registry.laid()
    registry.seen(slots, [seat], head, 0.0)
    bare = slots['kinds'][0]
    registry.seen(slots, [seat, back], head, 0.1)
    stands = {'wall', 'window', 'door', 'floor', 'stairs', 'platform', 'railing'}
    report.check("a bare seat a stool, its slot a chair once its back shows; a house's classes stand, a chair not",
                 bare == 'stool' and slots['kinds'][:2] == ['chair', 'chair'] and stands <= set(registry.STRUCTURAL)
                 and 'chair' not in registry.STRUCTURAL, '%s -> %s; %s' % (bare, slots['kinds'][:2], registry.STRUCTURAL))


def test_her_picture_tells_past_her_map(report):
    """What her map takes none of her picture still tells (`sight.Keep.hulls`) - one look's points, her map empty -: the
    floor under it, a pane past its reach a window; a door's colour high on that wall, standing on no floor, no door -
    the two a facade, enclosing nothing, no building."""
    from machine.eyes import sight
    from machine.eyes.stereo import H, W
    rows, cols = (a.ravel() for a in np.mgrid[0:H:2, 0:W:2])
    u, v, low = cols / W - 0.5, rows / H, rows >= H // 2
    xyz = np.where(low[:, None], np.column_stack([4.0 * u, 0.0 * u, 1.0 + 4.0 * (v - 0.5)]),
                   np.column_stack([4.0 * u, 0.1 + 2.3 * (1.0 - 2.0 * v), 6.0 + 0.0 * u]))
    pane = np.where((rows < H // 8)[:, None], sight.LOOKS['door'][0], sight.LOOKS['glass'][0])
    keep = sight.Keep()
    keep.last = sight.Look(sight.Seen(None, None, np.array([0.0, 1.5, 0.0]), np.eye(3)), rows, cols,
                           np.full(len(rows), -1), xyz, np.where(low[:, None], sight.LOOKS['wood'][0], pane))
    shown = sorted({k for k, _i, _h in keep.hulls()})
    report.check("past her map her picture tells the floor and a window; a door's colour high on a wall no door",
                 shown == ['floor', 'wall', 'window'], str(shown))


ROSTER = [test_her_fake_camera_tells_the_room, test_the_lodge_told_as_a_house, test_a_house_told_by_its_boxes,
          test_a_cabin_told_from_outside, test_the_lodge_from_its_deck, test_her_registry_keeps_what_each_is,
          test_her_picture_tells_past_her_map, test_her_loop_sees_through_it, test_her_vision_panel_holds_her_view,
          test_her_page_keeps_its_pace]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
