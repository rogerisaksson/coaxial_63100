#!/usr/bin/env python3
"""FBX takes in and out: a skeleton's joints' places from one, her look's rows written as one.

    times, joints = fbx.take('TAKE.fbx')        # s, {joint: (n, 3) places, m, y up}
    fbx.wrote('OUT.fbx', rows)                  # her rows (`look.simulated`), a bone a segment
    fbx.write('OUT.fbx', times, bones)          # any skeleton's

Read: binary 7.x (64-bit headers from 7500) and ASCII; a joint's place its chain's Lcl
Translation, PreRotation, Lcl Rotation and PostRotation's inverse. Joints go by HumanIK's names
(Hips, LeftUpLeg, LeftLeg, LeftFoot, LeftToeBase, Spine, Neck, Head, LeftArm, LeftForeArm,
LeftHand), another rig's beside its own (RIGS). Written: binary 7400, a LimbNode a bone, keys
linear; the stamp fixed, the FileId and the footer's code that stamp's - the FBX SDK's own
files' follow theirs so, 48 takes of 48 (2026-10-05).
"""
import os
import re
import struct
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # noqa: E402   behind the OpenBLAS cap

#: FBX time units a second; the orders of `RotationOrder`, the first axis applied first.
KTIME = 46186158000
ORDERS = ('xyz', 'xzy', 'yzx', 'yxz', 'zxy', 'zyx')

#: HumanIK's name of a joint by another rig's - 3ds Max's Biped, Character Creator's, Rokoko's -
#: its lead (`bip`, `CC_Base_`), spaces and underscores off, lower case.
RIGS = {'pelvis': 'Hips', 'hip': 'Hips', 'spine': 'Spine', 'spine1': 'Spine', 'neck': 'Neck',
        'head': 'Head'}
for _side, _s in (('Left', 'l'), ('Right', 'r')):
    RIGS.update({_s + 'thigh': _side + 'UpLeg', _s + 'calf': _side + 'Leg',
                 _s + 'foot': _side + 'Foot', _s + 'toe0': _side + 'ToeBase',
                 _s + 'toebase': _side + 'ToeBase', _s + 'upperarm': _side + 'Arm',
                 _s + 'forearm': _side + 'ForeArm', _s + 'hand': _side + 'Hand',
                 _side.lower() + 'thigh': _side + 'UpLeg', _side.lower() + 'shin': _side + 'Leg',
                 _side.lower() + 'toe': _side + 'ToeBase'})

#: Her segments as a take's bones, less their side.
BONES = {'pelvis': 'Hips', 'torso': 'Spine', 'neck': 'Neck', 'head': 'Head', 'upper_arm': 'Arm',
         'forearm': 'ForeArm', 'hand': 'Hand', 'fingers': 'HandMiddle1', 'thigh': 'UpLeg',
         'shank': 'Leg', 'foot': 'Foot', 'toes': 'ToeBase'}

#: A written take's stamp, the FileId and the footer's code it has, and every footer's end.
STAMP = '1970-01-01 10:00:00:000'
FILE_ID, FOOT = (bytes.fromhex('28b32aebb624ccc2bfc8b02aa92bfcf1'),
                 bytes.fromhex('fabcab09d0c8d466b176fb831cf7267e'))
TAIL = bytes.fromhex('f85a8c6adef5d97eece90ce3758f290b')

#: A take's frames a second as GlobalSettings' TimeMode; another rate is custom (14).
MODES = {120: 1, 100: 2, 60: 3, 50: 4, 48: 5, 30: 6, 24: 11}


def nodes(data):
    """[(name, properties, children)] of an FBX's bytes: binary 7.x or ASCII."""
    if not data.startswith(b'Kaydara FBX Binary'):
        return _ascii(data.decode('utf-8', 'replace'))
    head = '<QQQB' if struct.unpack_from('<I', data, 23)[0] >= 7500 else '<IIIB'
    size = struct.calcsize(head)

    def prop(pos):
        code, pos = chr(data[pos]), pos + 1
        if code in 'YCIFDL':
            fmt = {'Y': '<h', 'C': '<?', 'I': '<i', 'F': '<f', 'D': '<d', 'L': '<q'}[code]
            return struct.unpack_from(fmt, data, pos)[0], pos + struct.calcsize(fmt)
        if code in 'fdlibc':
            n, packed, length = struct.unpack_from('<III', data, pos)
            raw = data[pos + 12:pos + 12 + length]
            kind = {'f': '<f4', 'd': '<f8', 'l': '<i8', 'i': '<i4', 'b': '?', 'c': 'u1'}[code]
            return (np.frombuffer(zlib.decompress(raw) if packed else raw, kind, n),
                    pos + 12 + length)
        n = struct.unpack_from('<I', data, pos)[0]
        raw = data[pos + 4:pos + 4 + n]
        return (raw.decode('utf-8', 'replace') if code == 'S' else raw), pos + 4 + n

    def node(pos):
        end, count, _length, n = struct.unpack_from(head, data, pos)
        if end == 0:
            return None, pos + size + n
        name, pos = data[pos + size:pos + size + n].decode('ascii', 'replace'), pos + size + n
        props = []
        for _ in range(count):
            value, pos = prop(pos)
            props.append(value)
        kids = []
        while pos < end:
            kid, pos = node(pos)
            if kid is None:
                break
            kids.append(kid)
        return (name, props, kids), end

    out, pos = [], 27
    while pos + size < len(data):
        top, pos = node(pos)
        if top is None:
            break
        out.append(top)
    return out


def _ascii(text):
    """[(name, properties, children)] of an ASCII FBX: `Name: values {children}`, an array
    its `a` child's values."""
    tokens = re.findall(r';[^\n]*|"[^"]*"|[{}]|[^\s,{}"]+', text)
    tokens = [w for w in tokens if not w.startswith(';')]

    def value(word):
        if word.startswith('"'):
            return word[1:-1]
        try:
            return int(word)
        except ValueError:
            try:
                return float(word)
            except ValueError:
                return word

    def block(k):
        out = []
        while k < len(tokens) and tokens[k] != '}':
            name, k, props, kids = tokens[k][:-1], k + 1, [], []
            while k < len(tokens) and tokens[k] not in '{}' and not (
                    tokens[k].endswith(':') and not tokens[k].startswith('"')):
                props.append(value(tokens[k]))
                k += 1
            if k < len(tokens) and tokens[k] == '{':
                kids, k = block(k + 1)
                k += 1
            out.append((name, props, kids))
        return out, k
    return block(0)[0]


def _props(n):
    """{name: values} of a node's Properties70."""
    block = next((k for k in n[2] if k[0] == 'Properties70'), None)
    return {} if block is None else {p[1][0]: p[1][4:] for p in block[2] if p[0] == 'P'}


def _keys(n, name):
    """A curve's `name` array: its property, or an ASCII file's `a` child's."""
    kid = next(k for k in n[2] if k[0] == name)
    if kid[1] and isinstance(kid[1][0], np.ndarray):
        return kid[1][0]
    return np.array(next(a for a in kid[2] if a[0] == 'a')[1])


def _turns(deg, order):
    """(n, 3, 3) rotations from Euler degrees (n, 3), `order`'s first axis applied first."""
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    one, nil = np.ones(len(deg)), np.zeros(len(deg))
    by = {'x': (one, nil, nil, nil, c[:, 0], -s[:, 0], nil, s[:, 0], c[:, 0]),
          'y': (c[:, 1], nil, s[:, 1], nil, one, nil, -s[:, 1], nil, c[:, 1]),
          'z': (c[:, 2], -s[:, 2], nil, s[:, 2], c[:, 2], nil, nil, nil, one)}
    out = np.broadcast_to(np.eye(3), (len(deg), 3, 3))
    for axis in order:
        out = np.stack(by[axis], 1).reshape(-1, 3, 3) @ out
    return out


def take(path):
    """(times s, {joint: (n, 3) places, m, y up}) of a take, every joint at every key's time.

    A joint's place: its parent's, then Lcl Translation, PreRotation, Lcl Rotation and the
    inverse of PostRotation (offsets, pivots and scaling taken as none, as a mocap's are)."""
    with open(path, 'rb') as f:
        top = {n[0]: n for n in nodes(f.read())}
    settings = _props(top['GlobalSettings'])
    metres = float(settings.get('UnitScaleFactor', [1.0])[0]) / 100.0
    up = int(settings.get('UpAxis', [1])[0])
    objects = {n[1][0]: n for n in top['Objects'][2]}
    models = {i: n for i, n in objects.items() if n[0] == 'Model'}
    parent, channel, curve = {}, {}, {}
    for c in top['Connections'][2]:
        kind, a, b = c[1][:3]
        what = objects.get(a, ('',))[0]
        if kind == 'OO' and a in models and b in models:
            parent[a] = b
        elif kind == 'OP' and what == 'AnimationCurveNode' and b in models:
            channel[(b, c[1][3])] = a
        elif kind == 'OP' and what == 'AnimationCurve':
            curve[(b, c[1][3][-1])] = a
    keys = {}
    for (node, axis), i in curve.items():
        keys[(node, axis)] = (_keys(objects[i], 'KeyTime') / float(KTIME),
                              _keys(objects[i], 'KeyValueFloat').astype(float))
    times = np.unique(np.concatenate([t for t, _v in keys.values()]))

    def lcl(i, name):
        node, rest = channel.get((i, name)), _props(models[i]).get(name, [0.0, 0.0, 0.0])
        if node is not None:                                   # an axis without keys: its default
            given = _props(objects[node])
            rest = [given.get('d|' + a, [rest[k]])[0] for k, a in enumerate('XYZ')]
        return np.stack([np.interp(times, *keys[(node, a)]) if (node, a) in keys
                         else np.full(len(times), float(rest[j]))
                         for j, a in enumerate('XYZ')], 1)

    placed = {}

    def place(i):
        if i not in placed:
            props = _props(models[i])
            pre = _turns(np.array([props.get('PreRotation', [0, 0, 0])], float), 'xyz')
            post = _turns(np.array([props.get('PostRotation', [0, 0, 0])], float), 'xyz')
            order = ORDERS[int(props.get('RotationOrder', [0])[0])]
            turn = pre @ _turns(lcl(i, 'Lcl Rotation'), order) @ np.transpose(post, (0, 2, 1))
            at = lcl(i, 'Lcl Translation')
            if i in parent:
                up_turn, up_at = place(parent[i])
                turn, at = up_turn @ turn, np.einsum('nij,nj->ni', up_turn, at) + up_at
            placed[i] = (turn, at)
        return placed[i]

    joints = {n[1][1].split('\x00')[0].split(':')[-1]: place(i)[1] * metres
              for i, n in models.items()}
    for name in list(joints):
        key = name.lower().replace(' ', '').replace('_', '')
        key = next((key[len(lead):] for lead in ('ccbase', 'bip001', 'bip01', 'bip')
                    if key.startswith(lead)), key)
        if RIGS.get(key, name) not in joints:
            joints[RIGS[key]] = joints[name]
    if up == 2:
        joints = {k: np.stack([p[:, 0], p[:, 2], -p[:, 1]], 1) for k, p in joints.items()}
    return times, joints


class Long(int):
    """An int written 64 bits wide whatever its size: an id, a time."""


def _prop(p):
    """A property's bytes: its type's letter, then it - an array deflated."""
    if isinstance(p, bool):
        return b'C' + struct.pack('<?', p)
    if isinstance(p, int):
        wide = isinstance(p, Long) or not -2 ** 31 <= p < 2 ** 31
        return (b'L' + struct.pack('<q', p)) if wide else (b'I' + struct.pack('<i', p))
    if isinstance(p, float):
        return b'D' + struct.pack('<d', p)
    if isinstance(p, (str, bytes)):
        raw = p.encode() if isinstance(p, str) else p
        return (b'S' if isinstance(p, str) else b'R') + struct.pack('<I', len(raw)) + raw
    code = {'int64': b'l', 'float32': b'f', 'float64': b'd', 'int32': b'i'}[p.dtype.name]
    packed = zlib.compress(p.tobytes())
    return code + struct.pack('<III', len(p), 1, len(packed)) + packed


def _packed(at, name, props=(), kids=()):
    """A node's record as it lies at offset `at`: where it ends, its properties, its children
    and, after them - or with nothing in it - an empty record."""
    body = b''.join(_prop(p) for p in props)
    out, pos = [], at + 13 + len(name) + len(body)
    for kid in kids:
        out.append(_packed(pos, *kid))
        pos += len(out[-1])
    if kids or not props or name in ('AnimationStack', 'AnimationLayer'):
        out.append(bytes(13))
        pos += 13
    return (struct.pack('<IIIB', pos, len(props), len(body), len(name)) + name.encode() + body
            + b''.join(out))


def _p(name, kind, sub, *values):
    return ('P', (name, kind, sub, 'A' if kind.startswith('Lcl') or kind == 'Number' else '')
            + values)


def write(path, times, bones):
    """A take written: `bones` [(name, parent's name or None, at (x, y, z) m in its parent's
    frame, deg (n, 3) Euler x first or None, places (n, 3) m in place of `at` or None)],
    parents first, a key each of `times` s."""
    times = np.asarray(times, float)
    stamps = np.round((times - times[0]) * KTIME).astype(np.int64)
    end, n = Long(int(stamps[-1])), len(times)
    rate = (n - 1) / (times[-1] - times[0]) if n > 1 else 30.0
    ids = {name: Long(3000000000 + k) for k, (name, *_rest) in enumerate(bones)}
    stack, layer, free = Long(3100000000), Long(3100000001), [3200000000]
    objects, links = [], []

    def curves(model, what, values, scale):
        """`model`'s `what` keyed: a curve node, a curve an axis."""
        node = Long(free[0])
        free[0] += 4
        objects.append(('AnimationCurveNode', (node, what[4] + '\x00\x01AnimCurveNode', ''), [
            ('Properties70', (), [_p('d|' + a, 'Number', '', float(values[0][k] * scale))
                                  for k, a in enumerate('XYZ')])]))
        links.extend([('C', ('OO', node, layer)), ('C', ('OP', node, model, what))])
        for k, a in enumerate('XYZ'):
            if np.ptp(values[:, k]) * scale < 1e-9:           # an axis at rest: its default
                continue
            objects.append(('AnimationCurve', (Long(node + 1 + k), '\x00\x01AnimCurve', ''), [
                ('Default', (0.0,)), ('KeyVer', (4008,)), ('KeyTime', (stamps,)),
                ('KeyValueFloat', ((values[:, k] * scale).astype(np.float32),)),
                ('KeyAttrFlags', (np.array([24836], np.int32),)),
                ('KeyAttrDataFloat', (np.array([0.0, 0.0, 9.419963346924634e-30, 0.0],
                                               np.float32),)),
                ('KeyAttrRefCount', (np.array([n], np.int32),))]))
            links.append(('C', ('OP', Long(node + 1 + k), node, 'd|' + a)))
    for name, above, at, deg, places in bones:
        here = places[0] if places is not None else at
        turn = deg[0] if deg is not None else (0.0, 0.0, 0.0)
        objects.append(('NodeAttribute', (Long(ids[name] + 50000), name + '\x00\x01NodeAttribute',
                                          'LimbNode'), [('TypeFlags', ('Skeleton',))]))
        objects.append(('Model', (ids[name], name + '\x00\x01Model', 'LimbNode'), [
            ('Version', (232,)),
            ('Properties70', (), [
                _p('Lcl Translation', 'Lcl Translation', '', *(100.0 * float(v) for v in here)),
                _p('Lcl Rotation', 'Lcl Rotation', '', *(float(v) for v in turn)),
                _p('InheritType', 'enum', '', 1)]),
            ('Shading', (True,)), ('Culling', ('CullingOff',))]))
        links.extend([('C', ('OO', ids[name], ids[above] if above else Long(0))),
                      ('C', ('OO', Long(ids[name] + 50000), ids[name]))])
        if places is not None:
            curves(ids[name], 'Lcl Translation', np.asarray(places, float), 100.0)
        if deg is not None:
            curves(ids[name], 'Lcl Rotation', np.asarray(deg, float), 1.0)
    span = [_p(k, 'KTime', 'Time', v) for k, v in (
        ('LocalStart', Long(0)), ('LocalStop', end), ('ReferenceStart', Long(0)),
        ('ReferenceStop', end))]
    objects += [('AnimationStack', (stack, 'Take 001\x00\x01AnimStack', ''),
                 [('Properties70', (), span)]),
                ('AnimationLayer', (layer, 'BaseLayer\x00\x01AnimLayer', ''), [])]
    links.append(('C', ('OO', layer, stack)))
    counts = {}
    for kind, _props_, _kids in objects:
        counts[kind] = counts.get(kind, 0) + 1
    mode = MODES.get(int(round(rate)), 14)
    top = [
        ('FBXHeaderExtension', (), [
            ('FBXHeaderVersion', (1003,)), ('FBXVersion', (7400,)), ('EncryptionType', (0,)),
            ('CreationTimeStamp', (), [('Version', (1000,)), ('Year', (1970,)), ('Month', (1,)),
                                       ('Day', (1,)), ('Hour', (10,)), ('Minute', (0,)),
                                       ('Second', (0,)), ('Millisecond', (0,))]),
            ('Creator', ('coaxial_63100 tools/sim/fbx.py',))]),
        ('FileId', (FILE_ID,)), ('CreationTime', (STAMP,)),
        ('Creator', ('coaxial_63100 tools/sim/fbx.py',)),
        ('GlobalSettings', (), [('Version', (1000,)), ('Properties70', (), [
            _p('UpAxis', 'int', 'Integer', 1), _p('UpAxisSign', 'int', 'Integer', 1),
            _p('FrontAxis', 'int', 'Integer', 2), _p('FrontAxisSign', 'int', 'Integer', 1),
            _p('CoordAxis', 'int', 'Integer', 0), _p('CoordAxisSign', 'int', 'Integer', 1),
            _p('UnitScaleFactor', 'double', 'Number', 1.0), _p('TimeMode', 'enum', '', mode),
            _p('TimeSpanStart', 'KTime', 'Time', Long(0)), _p('TimeSpanStop', 'KTime', 'Time', end),
            _p('CustomFrameRate', 'double', 'Number', float(rate) if mode == 14 else -1.0)])]),
        ('Documents', (), [('Count', (1,)), ('Document', (Long(3300000000), 'Scene', 'Scene'), [
            ('RootNode', (Long(0),))])]),
        ('References', (), []),
        ('Definitions', (), [('Version', (100,)), ('Count', (1 + sum(counts.values()),)),
                             ('ObjectType', ('GlobalSettings',), [('Count', (1,))])]
         + [('ObjectType', (kind,), [('Count', (count,))]) for kind, count in counts.items()]),
        ('Objects', (), objects), ('Connections', (), links),
        ('Takes', (), [('Current', ('Take 001',)), ('Take', ('Take 001',), [
            ('FileName', ('Take_001.tak',)), ('LocalTime', (Long(0), end)),
            ('ReferenceTime', (Long(0), end))])])]
    out = b'Kaydara FBX Binary  \x00\x1a\x00' + struct.pack('<I', 7400)
    for node in top:
        out += _packed(len(out), *node)
    out += bytes(13) + FOOT + bytes(4)
    out += bytes(16 - len(out) % 16) + struct.pack('<I', 7400) + bytes(120) + TAIL
    with open(path, 'wb') as f:
        f.write(out)


def bone(segment):
    """HumanIK's name of her `segment`."""
    side, _bar, rest = segment.partition('_')
    return (side.capitalize() + BONES[rest]) if side in ('left', 'right') else BONES[segment]


def joints(rows):
    """(times s, {joint: (n, 3) places, m}) of her look's rows, as `take` has a take's."""
    from machine.figure import SEGMENTS
    return (np.array([float(r['t']) for r in rows]),
            {bone(seg[0]): np.array([[float(r['%s_%s' % (seg[0], axis)]) for axis in 'xyz']
                                     for r in rows]) for seg in SEGMENTS})


def wrote(path, rows):
    """Her look's rows written a take: a bone a segment, the pelvis's place and every bone's
    turn in its parent's frame keyed a row."""
    from machine import figure
    turns = {seg[0]: [] for seg in figure.SEGMENTS}
    for r in rows:
        placed = figure.frames({j: float(r[j]) for j in figure.JOINTS},
                               tuple(float(r[k]) for k in 'xyz'),
                               figure.quat(*(float(r['q' + k]) for k in 'wxyz')))
        for name, above, *_rest in figure.SEGMENTS:
            here = np.array(placed[name][1])
            turns[name].append(here if above is None else np.array(placed[above][1]).T @ here)
    bones = []
    for name, above, _joints, offset, *_rest in figure.SEGMENTS:
        m = np.array(turns[name])
        # x first: R = Rz Ry Rx
        deg = np.degrees(np.unwrap(np.stack([
            np.arctan2(m[:, 2, 1], m[:, 2, 2]), -np.arcsin(np.clip(m[:, 2, 0], -1.0, 1.0)),
            np.arctan2(m[:, 1, 0], m[:, 0, 0])], 1), axis=0))
        bones.append((bone(name), bone(above) if above else None, offset, deg,
                      np.array([[float(r[k]) for k in 'xyz'] for r in rows])
                      if above is None else None))
    write(path, [float(r['t']) for r in rows], bones)
