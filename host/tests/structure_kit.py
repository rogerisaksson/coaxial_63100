"""The structure suites' kit: the report, host/'s modules and sources, the names they bind."""
import ast
import io
import os
import re


HERE = os.path.dirname(os.path.abspath(__file__))


HOST = os.path.dirname(HERE)


REPO = os.path.dirname(HOST)


# Packages this suite walks.
PACKAGES = ('coaxial', 'coaxial_mcp', 'coaxial_ollama', 'machine', 'motor', 'testline',
            'terminal')


SCRIPTS = ('tools', 'examples')


#: Outside host/, and judged the same: a reader copies from these.
#: Notebooks included - their code cells are concatenated and parsed as
#: one module, so a rename that leaves `print(daq)` behind in a cell
#: fails here exactly as it did when the examples were .py files.
BESIDE = ('../notebook_examples',)


NESTS = (ast.If, ast.For, ast.While, ast.With, ast.Try, ast.ExceptHandler)


class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, what, ok, detail=''):
        self.passed += bool(ok)
        self.failed += not ok
        print('  %s  %-52s %s' % ('PASS' if ok else 'FAIL', what,
                                  '' if ok else detail))


def sources(beside=True):
    """(path, tree) for everything this suite judges, scripts included."""
    out = []
    for where in PACKAGES + SCRIPTS + (BESIDE if beside else ()):
        root = (os.path.join(REPO, where[3:]) if where.startswith('../')
                else os.path.join(HOST, where))
        if not os.path.isdir(root):
            continue
        for here, dirs, names in os.walk(root):
            dirs[:] = [d for d in dirs if d != '__pycache__']
            for name in sorted(names):
                path = os.path.join(here, name)
                if name.endswith('.py'):
                    text = io.open(path, encoding='utf-8').read()
                elif name.endswith('.ipynb'):
                    text = notebook_source(path)
                else:
                    continue
                rel = os.path.relpath(path, root)
                out.append((os.path.join(where, rel), text, ast.parse(text)))
    return out


def notebook_source(path):
    """A notebook's code cells as one module, for the same AST checks."""
    import json
    with io.open(path, encoding='utf-8') as handle:
        cells = json.load(handle)['cells']
    return '\n\n'.join(''.join(c['source']) for c in cells
                       if c['cell_type'] == 'code')


def depth(node, at=0):
    worst = at
    for child in ast.iter_child_nodes(node):
        worst = max(worst, depth(child, at + isinstance(child, NESTS)))
    return worst


#: The command header's op prefixes and the protocol module's enums. The
#: rails have one op and no enum on the C side.
OP_CLASSES = {'IMU': 'ImuOp', 'ANGLE': 'AngleOp', 'LINK': 'LinkOp',
              'CAL': 'CalOp', 'GATEDRIVERS': 'GateOp', 'LOG': 'LogOp',
              'DAQ': 'DaqOp', 'TIME': 'TimeOp', 'THERMAL': 'ThermalOp',
              'DRIVE': 'DriveOp', 'POWER': 'PowerOp', 'BOOT': 'BootOp',
              'CTRL': 'CtrlOp'}


_C_TOKENS = re.compile(
    r'(?P<open>\{)|(?P<close>\})'
    r'|(?P<leave>\b(?:return|continue)\b[^;]*;)'
    r'|for\s*\([^;]*;[^<]*<=?\s*(?:\([^)]*\)\s*)?(?P<bound>\w+)[^)]*\)'
    r'|(?P<refuse>\bwr_took\s*\(\s*out\s*,\s*")'
    r'|\b(?P<callee>\w+)\s*\(\s*out\b')


#: The command files whose helpers any handler may call - `wr_took` in
#: the wire writes the byte every acknowledging op answers with.
_SHARED = ('comms/src/cmd_device.c', 'comms/src/cmd.c', 'comms/src/wire.c')


#: Every file that dispatches ops the way a command file does: `case
#: PREFIX_OP_NAME: return handler(in, out)`. The bootloader's core is one
#: outside comms/, served by a node in its bootloader.
def _command_files():
    folder = os.path.join(REPO, 'comms', 'src')
    files = ['comms/src/' + name for name in sorted(os.listdir(folder))
             if name.startswith('cmd_') and name != 'cmd_length.c']
    return files + ['boot/src/boot_core.c']


_HEADERS = ('comms/inc', 'board/inc', 'drive/inc', 'thermal/inc', 'daq/inc', 'boot/inc',
            'filter/inc', 'shtp/inc', 'modbus/inc', 'ctrl/inc')


def _c_defines(extra_text=''):
    """Every `#define NAME <int>` in the tree's headers, and in the text
    given: what a loop bound may be."""
    found = {}
    texts = [extra_text]
    for rel in _HEADERS:
        folder = os.path.join(REPO, *rel.split('/'))
        for name in sorted(os.listdir(folder)) if os.path.isdir(folder) else ():
            if name.endswith('.h'):
                texts.append(io.open(os.path.join(folder, name), encoding='utf-8').read())
    for text in texts:
        for m in re.finditer(r'#define\s+(\w+)\s+\(?(\d+)U?\)?\s*(?:/|$)', text, re.M):
            found[m.group(1)] = int(m.group(2))
    return found


def _c_function(text, name):
    """The braces of `name` in `text`, or None when it is not defined there."""
    m = re.search(r'^(?:static\s+)?\w+\s+%s\s*\(\s*wr_t\s*\*\s*out' % re.escape(name), text, re.M)
    return None if m is None else text[text.index('{', m.end()):]


def _c_writes_in(text, body, defines, depth_limit=4):
    """The widths a body writes, in order. A block that refuses in words and leaves is a
    refusal, not the reply: a data op's own (MINOR 21)."""
    stack = [[[], None, False]]      # [writes, repeat-or-'*'-or-None, ends-in-leave]
    depth, bound, refusing = 0, None, None
    for m in _C_TOKENS.finditer(body):
        if m.group('open'):
            depth += 1
            if bound is not None:
                repeat = (int(bound.rstrip('U')) if bound.rstrip('U').isdigit()
                          else defines.get(bound, '*'))
                stack.append([[], repeat, False])
                bound = None
            elif depth > 1:
                stack.append([[], None, False])
            continue
        if m.group('close'):
            depth -= 1
            if depth == 0:
                break
            writes, repeat, left = stack.pop()
            if repeat == '*':
                if writes:
                    stack[-1][0].append(('*', tuple(writes)))
            elif repeat is not None:
                stack[-1][0].extend(writes * repeat)
            elif not left:
                stack[-1][0].extend(writes)
            stack[-1][2] = False
            continue
        if m.group('leave'):
            if refusing == depth:
                stack[-1][0], stack[-1][2], refusing = [], True, None
                continue
            if depth > 1 and stack[-1][0] and re.search(r'\breturn\b(?![^;]*CMD_ERR)', m.group('leave')):
                return [w for level in stack for w in level[0]]
            stack[-1][2] = True
            continue
        stack[-1][2] = False
        if m.group('bound'):
            bound = m.group('bound')
            continue
        if m.group('refuse'):
            if depth > 1:
                refusing = depth
            else:
                stack[-1][0].append('u8')
            continue
        callee = m.group('callee')
        w = re.fullmatch(r'wr_(u8|i8|u16|i16|u32|i32|str|bytes)', callee)
        if w and w.group(1) in ('str', 'bytes'):
            stack[-1][0].append(('rest',))
        elif w:
            stack[-1][0].append(w.group(1))
        elif depth_limit and _c_function(text, callee) is not None:
            stack[-1][0].extend(_c_writes_in(text, _c_function(text, callee), defines,
                                             depth_limit - 1))
    return stack[0][0]


def _c_source(rel):
    """One command file's text with the shared files' after it, comments
    stripped, so a helper is found whichever file defines it."""
    texts = []
    for each in (rel,) + _SHARED:
        path = os.path.join(REPO, *each.split('/'))
        if os.path.exists(path):
            texts.append(io.open(path, encoding='utf-8').read())
    return re.sub(r'/\*.*?\*/', ' ', '\n'.join(texts), flags=re.S)


def _shapes_agree(wrote, read, i=0, j=0):
    """Whether the reader's sequence is the writer's."""
    if i == len(wrote) and j == len(read):
        return True, i
    if i >= len(wrote) or j >= len(read):
        rest = wrote[i:] if i < len(wrote) else read[j:]
        return all(isinstance(x, tuple) and x[0] in ('?', 'rest') for x in rest), i
    a, b = wrote[i], read[j]
    if (isinstance(a, tuple) and a[0] == 'rest') or (isinstance(b, tuple) and b[0] == 'rest'):
        return True, len(wrote)
    if isinstance(a, tuple) and a[0] == '?':
        if isinstance(b, tuple) and b[0] == '?' and a[1] == b[1]:
            return _shapes_agree(wrote, read, i + 1, j + 1)
        taken = _shapes_agree(wrote, read, i + 1, j + 1) if b == a[1] else (False, i)
        return taken if taken[0] else _shapes_agree(wrote, read, i + 1, j)
    if isinstance(b, tuple) and b[0] == '?':
        taken = _shapes_agree(wrote, read, i + 1, j + 1) if a == b[1] else (False, i)
        return taken if taken[0] else _shapes_agree(wrote, read, i, j + 1)
    if isinstance(a, tuple) and isinstance(b, tuple):
        return _shapes_agree(wrote, read, i + 1, j + 1) if a[1] == b[1] else (False, i)
    if isinstance(a, tuple) or isinstance(b, tuple):
        # the side with the repeat, and the side laying the run out
        laid, body = ((read, list(a[1])) if isinstance(a, tuple)
                      else (wrote, list(b[1])))
        at = j if isinstance(a, tuple) else i
        n = 0
        while body and laid[at + n * len(body):at + (n + 1) * len(body)] == body:
            n += 1
        furthest = i
        for k in range(n, 0, -1):
            ni, nj = ((i + 1, j + k * len(body)) if isinstance(a, tuple)
                      else (i + k * len(body), j + 1))
            ok, reached = _shapes_agree(wrote, read, ni, nj)
            if ok:
                return True, reached
            furthest = max(furthest, reached)
        return False, furthest
    if a != b:
        return False, i
    return _shapes_agree(wrote, read, i + 1, j + 1)


#: The requests' side of the same check. Every `case PREFIX_OP_NAME: return
#: handler(in, out);` names the handler that reads an op's request, and
#: every `self._op(XOp.NAME, pack(...))` names what the decoder sends; the
#: enums are already held to cmd.h by name, so the two pair up by the op's
#: name and the reads are held to the widths.
_RD_TOKENS = re.compile(
    r'(?P<open>\{)|(?P<close>\})|(?P<semi>;)'
    r'|(?P<leave>\b(?:return|continue)\b[^;]*;)'
    r'|(?P<guard>\brd_left\s*\(\s*in\s*\)|\?)'
    r'|for\s*\([^;]*;[^<]*<=?\s*(?:\([^)]*\)\s*)?(?P<bound>\w+)[^)]*\)'
    r'|\brd_(?P<width>u8|i8|u16|i16|u32|i32|bytes)\s*\(\s*in\b'
    r'|\b(?P<call>\w+)\s*\(\s*in\s*[,)]')


_DISPATCH = re.compile(r'case\s+(\w+?)_OP_(\w+)\s*:\s*return\s+(\w+)\s*\(\s*(in|out)')


def _c_reads(text, name, defines):
    """The widths a handler reads off its request, in order: a field read
    under an `rd_left` guard - a ternary, or a block the guard opens - is
    `('?', width)`, taken when it is there; a loop over a count is its
    body that many times or `('*', body)`; `rd_bytes` is `('rest',)`,
    whatever is left; a helper handed `in` reads in its caller's place.
    """
    m = re.search(r'^static\s+\w+\s+%s\s*\(\s*rd_t\s*\*\s*in' % re.escape(name), text, re.M)
    if m is None:
        return []
    body = text[text.index('{', m.end()):]
    stack = [[[], None, False, False]]   # [reads, repeat, ends-in-leave, optional]
    depth, bound, armed = 0, None, False
    for t in _RD_TOKENS.finditer(body):
        if t.group('open'):
            depth += 1
            optional = armed or stack[-1][3]
            if bound is not None:
                repeat = (int(bound.rstrip('U')) if bound.rstrip('U').isdigit()
                          else defines.get(bound, '*'))
                stack.append([[], repeat, False, optional])
                bound = None
            elif depth > 1:
                stack.append([[], None, False, optional])
            armed = False
            continue
        if t.group('close'):
            depth -= 1
            if depth == 0:
                break
            reads, repeat, left, _ = stack.pop()
            if repeat == '*':
                if reads:
                    stack[-1][0].append(('*', tuple(reads)))
            elif repeat is not None:
                stack[-1][0].extend(reads * repeat)
            elif not left:
                stack[-1][0].extend(reads)
            stack[-1][2] = False
            continue
        if t.group('semi'):
            armed = False
            continue
        if t.group('leave'):
            stack[-1][2] = True
            armed = False
            continue
        stack[-1][2] = False
        if t.group('guard'):
            armed = True
            continue
        if t.group('bound'):
            bound = t.group('bound')
            continue
        if t.group('call'):
            stack[-1][0].extend(_c_reads(text, t.group('call'), defines))
            continue
        width = t.group('width')
        if width == 'bytes':
            stack[-1][0].append(('rest',))
        elif armed or stack[-1][3]:
            stack[-1][0].append(('?', width))
        else:
            stack[-1][0].append(width)
    return stack[0][0]
