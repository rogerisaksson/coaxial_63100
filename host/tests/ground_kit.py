"""What her suites on floors and beds share: her skin as points and its parts, her body laid."""
import math


#: The texts' joints as her movements (`term_rows.TERMS`, or `ABSENT`): her trunk one joint, lumbar and thoracic
#: summed.
MOVES = {'talocrural': ('ankle', 'dorsiflexion'), 'tibiofemoral': ('knee', 'flexion'),
         'coxofemoral': ('hip', 'flexion'), 'l4-l5-lumbar': ('trunk', 'flexion'),
         'c7-t12-thoracic': ('trunk', 'flexion'), 'glenohumeral': ('shoulder', 'flexion'),
         'cubital': ('elbow', 'flexion'), 'c1-c2-atlas': ('neck', 'flexion'), 'c1-c2-atlas-yaw': ('head', 'rotation'),
         'iliac-coronal': ('trunk', 'lateral-flexion'), 'calcaneus': ('foot', 'eversion'),
         'glenohumeral-ext-rot': ('shoulder', 'lateral-rotation'), 'glenohumeral-abduction': ('shoulder', 'abduction'),
         'glenohumeral-abduct': ('shoulder', 'abduction'), 'coxofemoral-abduct': ('hip', 'abduction'),
         'coxofemoral-ext-rot': ('hip', 'lateral-rotation')}


#: A part within this of the floor touches it, m.
TOUCH = 1e-3
#: The texts' landmarks by the segment of hers that carries each; her wrist's patch her fist (her hands bear on their
#: fists, `borne`).
LANDMARKS = {'calcaneus': 'foot', 'metatarsal-head': 'toes', 'femoral-diaphysis-dorsal': 'thigh',
             'femoral-diaphysis-ventral': 'thigh', 'ischial-tuberosity': 'pelvis', 'greater-trochanter': 'pelvis',
             'sacral': 'pelvis', 'iliac-anterior-spine': 'pelvis', 'l4-l5-lumbar': 'torso', 'c7-t12-thoracic': 'torso',
             'sternal-pectoral': 'torso', 'occipital': 'head', 'olecranon': 'forearm', 'ulna-distal': 'hand',
             'carpal-heel': 'hand', 'palmar-carpal': 'hand', 'tibial-tuberosity': 'shank', 'tarsal-dorsum': 'foot'}


def laid(origin):
    """{joint: deg} of her figure posed as a text's origin ({name: rad}): her hips, knees, ankles, shoulders and
    elbows as told, both sides, else straight; a movement she lacks left out."""
    from machine.body import figure
    from machine.body.term_rows import ABSENT, TERMS
    deg = figure.stand()
    deg.update({k: 0.0 for k in deg if k.endswith(('_hip', '_knee', '_ankle', '_shoulder', '_elbow'))})
    for name, rad in origin.items():
        if MOVES[name] in ABSENT:
            continue
        kind, sign = TERMS[MOVES[name]][:2]
        d = sign * math.degrees(rad)
        if kind == 'spine':
            deg['spine'] += d
        else:
            deg.update({j: d for j in ([kind] if kind in deg else ['left_' + kind, 'right_' + kind])})
    return deg


def part(segment):
    return segment.split('_', 1)[1] if segment.startswith(('left_', 'right_')) else segment


def skin(deg, turn=None):
    """(points (n, 3), radii (n,), their segments, her centre) of her figure posed `deg`, the pelvis at the origin
    turned `turn`: each capsule of `figure.BODY` its two ends, each sole's and toes' box its corners at 0; her
    fingers left out, her fists what her hands bear on."""
    import itertools
    import numpy as np
    from machine.body import figure
    turn = turn or figure.IDENT
    segments = {s[0] for s in figure.SEGMENTS if not s[0].endswith('fingers')}
    points, radii, of = [], [], []
    for name, (at, here) in figure.frames(deg, (0.0, 0.0, 0.0), turn).items():
        caps = [(r, e) for p, r, a, b in figure.BODY if name in segments and p == part(name) for e in (a, b)]
        boxes = [(0.0, tuple(c[i] + s[i] * size[i] for i in range(3)))
                 for p, _shape, size, c in figure.CONTACTS if name in segments and p == part(name)
                 for s in itertools.product((-1.0, 1.0), repeat=3)]
        for r, e in caps + boxes:
            points.append(np.add(at, figure.apply(here, e)))
            radii.append(r)
            of.append(name)
    return np.array(points), np.array(radii), of, np.array(figure.com(deg, (0.0, 0.0, 0.0), turn))


def dense(deg, step_m, lattice):
    """(points, radii, parts) of her skin posed `deg`, her soles on the floor: a capsule of `figure.BODY` a sphere every
    `step_m` along it, a sole's and toes' box a lattice at 0; her fingers left out."""
    import itertools
    import numpy as np
    from machine.body import figure
    segments = {s[0] for s in figure.SEGMENTS if not s[0].endswith('fingers')}
    points, radii, of = [], [], []
    for name, (at, here) in figure.frames(deg, (0.0, figure.standing(), 0.0), figure.IDENT).items():
        for r, a, b in [(r, a, b) for p, r, a, b in figure.BODY if name in segments and p == part(name)]:
            for s in np.linspace(0.0, 1.0, int(math.ceil(math.dist(a, b) / step_m)) + 1):
                points.append(np.add(at, figure.apply(here, tuple(np.add(a, s * np.subtract(b, a))))))
                radii.append(r)
                of.append(part(name))
        for size, c in [(size, c) for p, _s, size, c in figure.CONTACTS if name in segments and p == part(name)]:
            for s in itertools.product(np.linspace(-1.0, 1.0, lattice), repeat=3):
                points.append(np.add(at, figure.apply(here, tuple(c[i] + s[i] * size[i] for i in range(3)))))
                radii.append(0.0)
                of.append(part(name))
    return np.array(points), np.array(radii), np.array(of)
