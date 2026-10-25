"""A traced program's layout (`tools.cores.emit`): its elementwise temps folded, its buffers laid.

An elementwise temp written once and read by one later elementwise statement alone folded into its
reader (`fused`); each buffer's place in one workspace by when it lives (`lay`); the blocks its
inputs and outputs are laid in (`blocks`).
"""
import numpy as np

from tools.cores.trace import Ref


def blocks(shapes):
    """([(start, stop)] of each array of `shapes` in one block laid in order, the block's doubles)."""
    spans, at = [], 0
    for shape in shapes:
        size = int(np.prod(shape))
        spans.append((at, at + size))
        at += size
    return spans, at


def cstrides(shape):
    out, step = [], 1
    for n in reversed(shape):
        out.append(step)
        step *= n
    return list(reversed(out))


def leaves(node):
    """An elementwise tree's ('ref', Ref) leaves."""
    if node[0] == 'ref':
        yield node
    elif node[0] not in ('scalar', 'const'):
        for k in node[1:]:
            yield from leaves(k)


def same_ref(a, b):
    return (a.buffer is b.buffer and a.offset == b.offset and a.strides == b.strides and a.shape == b.shape)


def _put(node, target, tree):
    """`node` with each leaf reading `target` replaced by `tree`."""
    if node[0] == 'ref':
        return tree if same_ref(node[1], target) else node
    if node[0] in ('scalar', 'const'):
        return node
    return (node[0],) + tuple(_put(k, target, tree) for k in node[1:])


def fused(prog):
    """`prog`'s statements with each elementwise temp written once and read by one later elementwise
    statement alone, element for element, folded into that statement's tree - none of what it read
    written between - a MuJoCo call writes every extern. The per-element arithmetic numpy's, its
    temps gone."""
    sts = list(prog.statements)
    writers, readers = {}, {}
    for k, st in enumerate(sts):
        for b in writes(st):
            writers.setdefault(b.id, []).append(k)
        for b in reads(st):
            readers.setdefault(b.id, []).append(k)
    outs = {r.buffer.id for r in prog.outs}
    gone = set()
    for k, st in enumerate(sts):
        if st.kind != 'ew':
            continue
        b = st.out.buffer
        if (b.role != 'temp' or b.id in outs or writers.get(b.id) != [k]
                or len(set(readers.get(b.id, []))) != 1 or st.out.offset != 0 or b.size != int(np.prod(st.out.shape))):
            continue
        j = readers[b.id][0]
        use = sts[j]
        if use.kind != 'ew' or use.out.shape != st.out.shape:
            continue
        hits = [leaf for leaf in leaves(use.args) if leaf[1].buffer is b]
        if not hits or any(not same_ref(leaf[1], st.out) for leaf in hits):
            continue
        read = {leaf[1].buffer.id for leaf in leaves(st.args)}
        theirs = any(prog.buffers[i].role == 'extern' for i in read)
        if any((theirs and sts[i].kind == 'mj') or read & {x.id for x in writes(sts[i])}
               for i in range(k + 1, j) if i not in gone):
            continue
        sts[j] = type(use)(use.kind, use.out, _put(use.args, st.out, st.args), use.value, use.where)
        for leaf in leaves(st.args):
            readers.setdefault(leaf[1].buffer.id, []).append(j)
        gone.add(k)
    return [st for k, st in enumerate(sts) if k not in gone]


def lay(prog, statements=None):
    """Each buffer's place: inputs by their order, externs by theirs; an output that is its whole
    buffer in C order the caller's own array; the rest in one workspace, a place free once the
    statement after its last has run; (places, the workspace's doubles)."""
    last, first = {}, {}
    statements = prog.statements if statements is None else statements
    for k, st in enumerate(statements):
        for b in touched(st):
            first.setdefault(b.id, k)
            last[b.id] = k
    end = len(statements)
    outs = {}
    for k, r in enumerate(prog.outs):
        last[r.buffer.id] = end
        b = r.buffer
        if (b.role not in ('in', 'extern') and r.offset == 0 and r.strides == tuple(cstrides(r.shape))
                and b.size == int(np.prod(r.shape))):
            outs.setdefault(b.id, k)
    starts = {}
    for bid, k in first.items():
        starts.setdefault(k, []).append(bid)
    at, free, top, ending = {}, [], 0, {}
    for k in range(end + 1):
        for bid in ending.pop(k, ()):
            free = _merged(free + [(at[bid], prog.buffers[bid].size)])
        for bid in starts.get(k, ()):
            b = prog.buffers[bid]
            if b.role in ('in', 'extern') or bid in outs:
                continue
            spot = next((s for s in free if s[1] >= b.size), None)
            if spot is not None:
                free.remove(spot)
                at[bid] = spot[0]
                if spot[1] > b.size:
                    free = _merged(free + [(spot[0] + b.size, spot[1] - b.size)])
            else:
                at[bid] = top
                top += b.size
            ending.setdefault(last[bid] + 1, []).append(bid)
    return {'at': at, 'in': {b.id: k for k, b in enumerate(prog.ins)}, 'out': outs,
            'ext': {b.id: k for k, b in enumerate(prog.externs)}}, top


def _merged(free):
    """Free places sorted, the touching ones one."""
    out = []
    for start, size in sorted(free):
        if out and out[-1][0] + out[-1][1] == start:
            out[-1] = (out[-1][0], out[-1][1] + size)
        else:
            out.append((start, size))
    return out


def writes(st):
    """The buffers a statement writes: its own, a MuJoCo call's buffers not const, a solve's scratch."""
    if st.kind == 'mj':
        return [k[1].buffer for k in st.args[1] if k[0] == 'buf' and not k[2]]
    if st.kind == 'solve':
        return [st.out.buffer, st.args[4].buffer]
    return [st.out.buffer]


def reads(st):
    """The buffers a statement reads."""
    if st.kind == 'ew':
        return [leaf[1].buffer for leaf in leaves(st.args)]
    if st.kind == 'einsum':
        return [r.buffer for r in st.args[1] if not isinstance(r, tuple)]
    if st.kind in ('matmul', 'solve'):
        return [side[0].buffer for side in st.args[:2] if side is not None and isinstance(side[0], Ref)]
    if st.kind == 'reduce':
        return [st.args[1].buffer]
    if st.kind == 'mj':
        return [k[1].buffer for k in st.args[1] if k[0] == 'buf']
    if st.kind in ('gather', 'scatter') and st.args[1 if st.kind == 'scatter' else 0] is not None:
        return [st.args[1 if st.kind == 'scatter' else 0]]
    return []


def touched(st):
    return writes(st) + reads(st)
