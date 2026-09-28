"""The wire's shapes agree: what each handler writes is what the host reads, field for field."""
import ast
import importlib
import inspect
import sys
import textwrap

from tools.dev.focus import chosen
from structure_kit import Report, _c_defines, _c_function, _c_source, _c_writes_in, _shapes_agree, depth


#: A fixed-shape reply written field by field in C and read field by field
#: in Python, and the two sequences held to each other: the C file and
#: handler, the Python module, class and method. PROTOCOL.md describes the
#: same shape in prose, which no parser holds; these two are what a wire
#: crosses, and the C's own comments say what one moved offset
#: costs every decoder.
WIRE_SHAPES = (
    ('comms/src/cmd_gate_drivers.c', 'h_gate_drivers_state',
     'coaxial.devices.gate_drivers', 'GateDrivers', 'state'),
    ('comms/src/cmd_drive.c', 'h_drive_state', 'coaxial.devices.drive', 'Drive', 'state'),
    ('comms/src/cmd_ctrl.c', 'h_ctrl_state', 'coaxial.devices.ctrl', 'Ctrl', 'state'),
    ('comms/src/cmd_daq.c', 'h_daq_state', 'coaxial.acquire.daq', 'Daq', 'state'),
    ('comms/src/cmd_drive.c', 'h_drive_setpoints', 'coaxial.devices.drive', 'Drive', 'setpoints'),
    ('comms/src/cmd_drive.c', 'h_drive_window', 'coaxial.devices.drive', 'Drive', '_take_window'),
    ('comms/src/cmd_drive.c', 'h_drive_moments', 'coaxial.devices.drive', 'Drive', '_read_moments'),
    ('comms/src/cmd_drive.c', 'h_drive_model', 'coaxial.devices.drive', 'Drive', '_read_model'),
    ('comms/src/cmd_drive.c', 'h_drive_observers', 'coaxial.devices.drive', 'Drive', '_read_observers'),
    ('comms/src/cmd_log.c', 'h_log_state', 'coaxial.acquire.capture', 'Capture', 'state'),
    ('comms/src/cmd_power.c', 'h_power_state', 'coaxial.devices.power', 'Power', 'state'),
    ('comms/src/cmd_thermal.c', 'h_thermal_state',
     'coaxial.devices.thermal', 'Thermal', 'state'),
    ('comms/src/cmd_thermal.c', 'h_thermal_budget',
     'coaxial.devices.thermal', 'Thermal', 'budget'),
    ('comms/src/cmd_thermal.c', 'h_thermal_edges',
     'coaxial.devices.thermal', 'Thermal', 'network'),
    ('comms/src/cmd_time.c', 'h_time_read', 'coaxial.acquire.clock', 'Clock', 'read'),
)


#: What each Reader method takes off the wire; the scaled ones take a width
#: as their first argument when it is not the default.
_READS = {'u8': 'u8', 'i8': 'i8', 'u16': 'u16', 'i16': 'i16', 'u32': 'u32',
          'i32': 'i32', 'q16': 'u32', 'flags': 'u8', 'fraction': 'u8',
          'centi': 'i32', 'milli': 'i32', 'micro': 'i32', 'nano': 'u32'}


_WIDTH_ARG = ('centi', 'milli', 'micro', 'nano')


def _c_writes(rel, name):
    text = _c_source(rel)
    body = _c_function(text, name)
    assert body is not None, '%s has no %s taking out first' % (rel, name)
    return _c_writes_in(text, body, _c_defines(text))


def _py_reads(module, cls, method):
    """The widths a decoder reads, in order, the same way: a loop over a
    module's tuple or a constant is its body that many times, a loop over
    a count read off the wire is `('*', body)`, a helper handed the
    reader is followed.
    """
    mod = importlib.import_module(module)
    owner = getattr(mod, cls)

    def count(node):
        """A loop's repeat when a constant or the module knows it, else None."""
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'range'):
            arg = node.args[-1]
            value = (arg.value if isinstance(arg, ast.Constant) else
                     getattr(mod, arg.id, None) if isinstance(arg, ast.Name) else None)
            return value if isinstance(value, int) else None
        if isinstance(node, (ast.Tuple, ast.List)):
            return len(node.elts)
        if isinstance(node, ast.Name) and hasattr(mod, node.id):
            return len(getattr(mod, node.id))
        return None

    def helper(node):
        """The function a call hands the reader to, or None."""
        if isinstance(node.func, ast.Name):
            return getattr(mod, node.func.id, None)
        if (isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name)
                and node.func.value.id == 'self'):
            return getattr(owner, node.func.attr, None)
        return None

    def function_reads(fn, reader_at, out, depth):
        fn_def = ast.parse(textwrap.dedent(inspect.getsource(fn))).body[0]
        if not isinstance(fn_def, (ast.FunctionDef, ast.AsyncFunctionDef)):
            raise ValueError('a reader this check cannot follow: %s' % fn)
        params = [a.arg for a in fn_def.args.args]
        if params and params[0] == 'self':
            params = params[1:]
        visit(fn_def, out, params[reader_at], depth + 1)

    def visit(node, out, reader, depth=0):
        if isinstance(node, (ast.For, ast.GeneratorExp, ast.ListComp, ast.DictComp)):
            source_ = node.iter if isinstance(node, ast.For) else node.generators[0].iter
            body = node.body if isinstance(node, ast.For) else (
                [node.key, node.value] if isinstance(node, ast.DictComp) else [node.elt])
            inner = []
            for child in body:
                visit(child, inner, reader, depth)
            heads = []
            visit(source_, heads, reader, depth)
            out.extend(heads)
            repeat = None if heads else count(source_)
            if inner:
                out.extend(inner * repeat if repeat else [('*', tuple(inner))])
            return
        if isinstance(node, ast.Call):
            if (isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name)
                    and node.func.value.id == reader):
                name = node.func.attr
                if name == 'maybe':
                    width = node.args[0]
                    if not isinstance(width, ast.Constant):
                        raise ValueError('a maybe() this check cannot size: %s' % reader)
                    out.append(width.value)
                elif name in _READS:
                    width = _READS[name]
                    if name in _WIDTH_ARG and node.args and isinstance(node.args[0], ast.Constant):
                        width = node.args[0].value
                    out.append(width)
                else:
                    raise ValueError('a read this check cannot size: %s.%s' % (reader, name))
            handed = [k for k, a in enumerate(node.args)
                      if isinstance(a, ast.Name) and a.id == reader]
            fn = helper(node) if handed else None
            if fn is not None and depth < 4:
                function_reads(fn, handed[0], out, depth)
                return
        for child in ast.iter_child_nodes(node):
            visit(child, out, reader, depth)

    out = []
    tree = ast.parse(textwrap.dedent(inspect.getsource(getattr(owner, method))))
    visit(tree, out, 'r')
    return out


def test_wire_shapes_agree(r):
    """A reply's shape is one sequence of widths, written in C and read in
    Python, and the two are held to each other field by field.
    """
    for rel, func, module, cls, method in WIRE_SHAPES:
        wrote = _c_writes(rel, func)
        read = _py_reads(module, cls, method)
        same, at = _shapes_agree(wrote, read)
        where = ('%d fields both ways' % len(wrote) if same else
                 'from field %d: C %s, Python %s' % (
                     at, ' '.join(str(w) for w in wrote[at:at + 4]) or 'nothing',
                     ' '.join(str(w) for w in read[at:at + 4]) or 'nothing'))
        r.check('%s writes what %s.%s reads, field for field' % (func, cls, method),
                same, where)


ROSTER = (test_wire_shapes_agree,)

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
