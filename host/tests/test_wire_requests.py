"""The wire's requests agree: what the host sends is what each handler reads."""
import ast
import io
import os
import re
import sys

from tools.dev.focus import chosen
from structure_kit import HOST, OP_CLASSES, REPO, Report, _DISPATCH, _c_defines, _c_reads, _command_files


def _c_requests():
    """{(PREFIX, OP): reads} for every op a command file dispatches."""
    found = {}
    for rel in _command_files():
        text = io.open(os.path.join(REPO, *rel.split('/')), encoding='utf-8').read()
        text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
        defines = _c_defines(text)
        for prefix, op, handler, first in _DISPATCH.findall(text):
            found[(prefix, op)] = _c_reads(text, handler, defines) if first == 'in' else []
    return found


def _py_requests():
    """{(EnumName, OP): widths} for every request the library sends whose
    payload is a literal `pack(...)`, nothing, or a name bound to one in
    the same function; None where it is built some other way."""
    found = {}
    folder = os.path.join(HOST, 'coaxial')

    def widths_of(node, scope):
        if node is None:
            return []
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'pack'):
            out = []
            for arg in node.args:
                if not (isinstance(arg, ast.Tuple) and arg.elts
                        and isinstance(arg.elts[0], ast.Constant)):
                    return None
                out.append(arg.elts[0].value)
            return out
        if isinstance(node, ast.Name) and node.id in scope:
            return widths_of(scope[node.id], {})
        return None

    for name in sorted(os.listdir(folder)):
        if not name.endswith('.py'):
            continue
        tree = ast.parse(io.open(os.path.join(folder, name), encoding='utf-8').read())
        for fn in [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]:
            scope = {t.id: s.value for s in fn.body if isinstance(s, ast.Assign)
                     for t in s.targets if isinstance(t, ast.Name)}
            for call in ast.walk(fn):
                if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                        and call.func.attr in ('_op', '_ack') and call.args):
                    continue
                op = call.args[0]
                if not (isinstance(op, ast.Attribute) and isinstance(op.value, ast.Name)
                        and op.value.id.endswith('Op')):
                    continue
                payload = call.args[1] if len(call.args) > 1 else None
                key = (op.value.id, op.attr)
                widths = widths_of(payload, scope)
                if key not in found or found[key] is None:
                    found[key] = widths
    return found


def _request_agrees(reads, widths):
    """Whether packed `widths` are what the handler reads: an optional field
    taken when it is there, the rest whatever is left, a repeat once or
    more."""
    def go(i, j):
        if i == len(reads):
            return j == len(widths)
        a = reads[i]
        if isinstance(a, tuple) and a[0] == 'rest':
            return True
        if isinstance(a, tuple) and a[0] == '?':
            return (j < len(widths) and widths[j] == a[1] and go(i + 1, j + 1)) or go(i + 1, j)
        if isinstance(a, tuple) and a[0] == '*':
            body = list(a[1])
            n = 0
            while body and widths[j + n * len(body):j + (n + 1) * len(body)] == body:
                n += 1
            return any(go(i + 1, j + k * len(body)) for k in range(n, 0, -1))
        return j < len(widths) and widths[j] == a and go(i + 1, j + 1)
    return go(0, 0)


def test_wire_requests_agree(r):
    """What the library packs into a request is what the handler reads."""
    enum_of = {prefix: cls for prefix, cls in OP_CLASSES.items()}
    reads = _c_requests()
    packs = _py_requests()
    for prefix in sorted({p for p, _ in reads}):
        cls = enum_of.get(prefix)
        wrong, compared, skipped = [], 0, []
        for (p, op), sequence in sorted(reads.items()):
            if p != prefix:
                continue
            widths = packs.get((cls, op), 'absent')
            if widths == 'absent':
                continue                      # no request from the library
            if widths is None:
                skipped.append(op)
                continue
            compared += 1
            if not _request_agrees(sequence, widths):
                wrong.append('%s: C reads %s, Python packs %s' % (
                    op, ' '.join(str(w) for w in sequence) or 'nothing',
                    ' '.join(widths) or 'nothing'))
        r.check("%s's requests are packed as its handlers read them" % prefix,
                not wrong, '; '.join(wrong[:3]) or '%d ops%s' % (
                    compared, (', not literal: ' + ', '.join(skipped)) if skipped else ''))


ROSTER = (test_wire_requests_agree,)

def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
