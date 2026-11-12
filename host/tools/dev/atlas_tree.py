"""The atlas's tree: the trunk's layers as a table, every tracked file on one, its symbols and ties.

A path's layer is its longest prefix in LAYERS; the rest is read from the files as they are: synopses by
docs/structure.py, modules and imports by host_map.py, prototypes, includes and the wire's rows by
target_map.py. A tie joins two files: a module's import, a C file's include, a text naming a file by a
name only that file has, or a command the wire names on both ends - CMD_X's number in a header, X bound
to it in a host module. tools/dev/atlas.py shows it, a node at a time or as a page. A build keeps what it
read off each file in build/atlas.cache, by the file's size and time and the code that read it: the next
reads only the files changed since.
"""
import ast
import collections
import contextlib
import csv
import hashlib
import importlib.util
import marshal
import os
import re
import subprocess
import sys

from tools import REPO
from tools.dev import host_map, target_map

#: The trunk from its foot, the terminal, up to the shunts; then what stands in for its upper half and
#: what is about it. A row: its key, its role, what it holds, its path prefixes (a path's is its longest).
LAYERS = (
    ('terminal', 'trunk', 'What the user sees and presses: the front page, the live views, their drawings, '
                          'the launchers.',
     ('host/terminal/', 'host/coaxial/draw/', 'host/coaxial/graphics/', 'terminal/', 'render/',
      'system_terminal.ps1')),
    ('agents', 'trunk', "What speaks for the user: the local model's runner, the MCP server, the board's "
                        'chat, the test plan.',
     ('host/coaxial_ollama/', 'host/coaxial_mcp/', 'host/board_chat', 'host/testline/')),
    ('texts', 'trunk', "What she is told: the language's texts, her G-code, their reading and voicing.",
     ('host/lisp/', 'host/gcode/', 'host/machine/texts/')),
    ('executive', 'trunk', "Her loop and the crafts': processes, way, moves, the latent law, the whole-body "
                           'step, senses, eyes, body.',
     ('host/machine/',)),
    ('rig', 'trunk', "Machines over IO nodes: boards and their loops, the inverter's library, the motors.",
     ('host/machine/rig/', 'host/machine/library/', 'host/machine/__init__.py', 'host/machine/errors.py',
      'host/motor/', 'host/coaxial/')),
    ('wire', 'trunk', "The wire: Modbus RTU framing and the board's commands, the host's end and the "
                      "firmware's.",
     ('host/coaxial/comm/', 'comms/', 'modbus/')),
    ('cores', 'trunk', 'The portable C11 cores, a step a pass, built for the MCU and for this host.',
     ('wbc/', 'aero/', 'mppi/', 'drive/', 'ctrl/', 'thermal/', 'daq/', 'filter/')),
    ('board', 'trunk', 'The MCU and its edge: boot, main and interrupts, the board layer, the build.',
     ('board/', 'core/', 'boot/', 'shtp/', 'cmake/', 'CMakeLists.txt', 'CMakePresets.json',
      'STM32H753xx_FLASH.ld', 'startup_stm32h753xx.s', 'coaxial_63100.ioc')),
    ('sensing', 'trunk', 'The power stage and the analog front end up to the phase shunts: schematic, BOM, '
                        'SPICE models, datasheets.',
     ('electronics/', 'electronic_simulations', 'datasheets/')),
    ('twin', 'stand-in', 'What stands in for the hardware: her world in MuJoCo, the simulated board, the C '
                         "plant, the board's fakes and emulator.",
     ('host/machine/world/', 'host/coaxial/simulated/', 'world/', 'board/fake/', 'board/native/',
      'board/emu/')),
    ('workshop', 'about', 'Tools and tests: the developer loop, the bench, the cores built here, the suites, '
                          'setup.',
     ('host/tools/', 'host/tests/', 'host/examples/', 'host/', 'notebook_examples/', 'setup/', 'setup.ps1',
      'env.ps1')),
    ('docs', 'about', 'What is written down: the architecture, the findings, the protocol, the hardware, the '
                      'README.',
     ('docs/', 'README.md', 'AGENTS.md', 'CLAUDE.md', 'LICENSE')),
)
#: A node's kinds in table order, and a tie's.
LAYER, FOLDER, FILE, SYMBOL = range(4)
KINDS = ('layer', 'folder', 'file', 'symbol')
TIES = ('imports', 'includes', 'names', 'wire')
#: (prefix, layer), the longest first: a path's layer is its first match.
ROWS = sorted(((prefix, i) for i, row in enumerate(LAYERS) for prefix in row[3]), key=lambda r: -len(r[0]))
#: The kinds read as text, and those of them read for the files they name.
TEXT = ('.py', '.c', '.h', '.s', '.lisp', '.md', '.ps1', '.cs', '.asc', '.asy', '.lib', '.csv', '.json',
        '.toml', '.txt', '.yml', '.yaml', '.repl', '.resc', '.ld', '.cmake', '.gcode', '.ioc')
NAMING = ('.py', '.c', '.h', '.lisp', '.md', '.ps1', '.cs', '.repl', '.resc', '.toml', '.json', '.yml',
          '.cmake')
#: A file a text names; a command's number where a header defines it; a Lisp definition; a comment line.
NAMED = re.compile(r'(?<![\w./-])[\w./-]+\.[A-Za-z]{1,5}\b')
COMMAND = re.compile(r'^\s*#\s*define\s+CMD_(\w+)\s+(0x[0-9A-Fa-f]+|\d+)', re.M)
DEF = re.compile(r'^\((def[\w-]*)\s+\(?([^\s()]+)', re.M)
LEAD = re.compile(r'^\s*(?:#+|//+|;+|\*+|/\*+)\s?(.*?)\s*(?:\*/)?$')
#: Where a build keeps what it read off each file (git leaves build/ out), and a file's rows as kept: its
#: stamp (size, time) when read, its brief, its symbols, a module's import statements and top-level
#: numbers, the file names it says, the headers it includes, a header's command numbers, a C file's wire
#: rows - None where its kind has none.
CACHE = os.path.join(REPO, 'build', 'atlas.cache')
STAMP, BRIEF, DEFS, IMPORTS, NUMBERS, NAMES, INCLUDES, COMMANDS, CODES = range(9)


def _load(path):
    """A module from its file, kept out of sys.modules."""
    spec = importlib.util.spec_from_file_location('atlas_' + os.path.basename(path)[:-3], path)
    if spec is None or spec.loader is None:
        raise ImportError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


#: docs/structure.py: a file's synopsis, a folder's purpose, the trees it leaves out.
structure = _load(os.path.join(REPO, 'docs', 'structure.py'))


def _git(where, *args):
    run = subprocess.run(('git',) + args + ('-z',), cwd=where, capture_output=True, check=True)
    return [p for p in run.stdout.decode('utf-8').split('\0') if p]


def tracked():
    """Every path git tracks, a checked-out submodule's files in its place, outside the trees left out."""
    out = []
    for entry in _git(REPO, 'ls-files', '-s'):
        mode, path = entry.split(' ', 1)[0], entry.split('\t', 1)[1]
        inner = os.path.join(REPO, path)
        held = mode == '160000' and os.path.exists(os.path.join(inner, '.git'))
        out += [path + '/' + p for p in _git(inner, 'ls-files')] if held else [path]
    return [p for p in out
            if not p.startswith(structure.SKIP) and not any(s.startswith('.') for s in p.split('/'))]


def layer_of(path):
    """The index in LAYERS of the layer holding `path`, None where no row does."""
    return next((i for prefix, i in ROWS if path.startswith(prefix)), None)


class Atlas:
    """The atlas as columns: its nodes in table order - the layers, then each folder, its files and their
    symbols - and the ties between files."""

    def __init__(self):
        self.id, self.up, self.kind, self.label, self.brief, self.line = [], [], [], [], [], []
        self.src, self.dst, self.tie = [], [], []
        self.at, self.low, self.seen = {}, {}, set()

    def lowered(self, path):
        """A file's text in lower case, read and kept once asked - a word looked for in every text: the
        build keeps no texts, 19.6 MB a query read back for a word few queries look for."""
        if path not in self.low:
            self.low[path] = _read(path).lower()
        return self.low[path]

    def add(self, key, up, kind, label, brief='', line=0):
        """A node's row and its index; a key met before takes '#2', '#3'."""
        own, n = key, 1
        while own in self.at:
            n += 1
            own = '%s#%d' % (key, n)
        self.at[own] = len(self.id)
        for column, value in zip((self.id, self.up, self.kind, self.label, self.brief, self.line),
                                 (own, up, kind, label, brief, line)):
            column.append(value)
        return self.at[own]

    def join(self, a, b, tie):
        """A tie from file `a` to file `b`: once a kind, none to itself or to a file not here."""
        i, j = self.at.get(a), self.at.get(b)
        if i is not None and j is not None and i != j and (i, j, tie) not in self.seen:
            self.seen.add((i, j, tie))
            self.src.append(i)
            self.dst.append(j)
            self.tie.append(tie)


def _read(path):
    """A file's text, '' for one not read as text; UTF-16 where its mark says so, else UTF-8."""
    full = os.path.join(REPO, path)
    if os.path.splitext(path)[1].lower() not in TEXT or not os.path.isfile(full):
        return ''
    with open(full, 'rb') as f:
        raw = f.read()
    if raw[:2] in (b'\xff\xfe', b'\xfe\xff'):
        return raw.decode('utf-16', errors='replace')
    return raw.decode('utf-8', errors='replace').lstrip('﻿')


def _parse(text):
    try:
        return ast.parse(text)
    except SyntaxError:
        return ast.Module(body=[], type_ignores=[])


def _brief(path, text, tree=None):
    """A file's first sentence: structure.py's synopsis (a module's off the tree parsed), a document's
    title, a script's synopsis, else its opening comment; a folder tracked is a submodule not checked out."""
    folder, name = path.rpartition('/')[::2]
    kind = os.path.splitext(name)[1].lower()
    if tree is not None:
        return structure.sentence((ast.get_docstring(tree) or '').split('\n\n')[0])
    if kind in structure.KINDS:
        return structure.synopsis(folder or '.', name, text)
    if os.path.isdir(os.path.join(REPO, path)):
        return 'A submodule, not checked out here.'
    said = re.search(r'^#+\s+(.+)$' if kind == '.md' else r'\.SYNOPSIS[ \t]*\n\s*(.+)', text, re.M)
    if said:
        return structure.sentence(said.group(1))
    lead = []
    for line in text.splitlines()[:16]:
        m = LEAD.match(line)
        if not m:
            break
        lead.append(m.group(1))
    return structure.sentence(' '.join(w for w in lead if w.strip(' *-=')))


def _purpose(folder, names, said):
    """What a folder's files are for: structure.py's purpose of its sources, else its README's title -
    off `said`, each file's brief as the build read it, no file read again."""
    sources = [n for n in names if n.endswith(structure.KINDS)]
    if sources:
        return structure.purpose(folder or '.', sources, said)
    return said.get('README.md', '')


def _line(text, at):
    return text.count('\n', 0, at) + 1


def _py(tree):
    """A module's public defs and classes, a class's public methods by its name."""
    out, owner = [], ''
    for indent, node in host_map.public(tree):
        owner = owner if indent else node.name
        name = owner + '.' + node.name if indent else node.name
        signature = (owner + '.' if indent else '') + host_map.signature(node)
        out.append((name, signature, host_map.brief(ast.get_docstring(node)), node.lineno))
    return out


def _h(text):
    """A header's prototypes."""
    out = []
    for proto in target_map.prototypes(text):
        name = re.search(r'(\w+)\s*\(', proto)
        at = name and re.search(r'\b%s\s*\(' % name.group(1), text)
        if name:
            out.append((name.group(1), proto, '', _line(text, at.start()) if at else 0))
    return out


def _c(text):
    """A C file's public functions, and the commands its rows and device cases hand out."""
    out = [(m.group(1), '%s "%s" -> %s' % m.groups(), '', _line(text, m.start()))
           for m in target_map.ROW.finditer(text)]
    out += [('%s.%s' % m.group(1, 2), '%s.%s -> %s' % m.groups(), '', _line(text, m.start()))
            for m in target_map.CASE.finditer(text)]
    return out + [(m.group(1), ' '.join(m.group(0)[:-1].split()), '', _line(text, m.start()))
                  for m in target_map.DEFINITION.finditer(text) if not m.group(0).startswith('static')]


def _lisp(text):
    """A text's top-level definitions."""
    return [(m.group(2), '(%s %s' % m.groups(), '', _line(text, m.start())) for m in DEF.finditer(text)]


def _md(text):
    """A document's headings to the third level, its fenced code passed over."""
    out, fenced = [], False
    for n, line in enumerate(text.splitlines(), 1):
        fenced ^= line.lstrip().startswith('```')
        m = None if fenced else re.match(r'(#{1,3})\s+(.+?)\s*#*$', line)
        if m:
            out.append((m.group(2), line.strip(), '', n))
    return out


def _asc(text):
    """A schematic's parts: each symbol's instance name, its value, its model."""
    parts = []
    for n, line in enumerate(text.splitlines(), 1):
        word = line.split()
        if word[:1] == ['SYMBOL']:
            parts.append([word[1] if len(word) > 1 else '', '', '', n])
        elif parts and word[:2] == ['SYMATTR', 'InstName']:
            parts[-1][1] = ' '.join(word[2:])
        elif parts and word[:2] == ['SYMATTR', 'Value']:
            parts[-1][2] = ' '.join(word[2:])
    return [(name, (name + ' ' + value).strip(), model, n) for model, name, value, n in parts if name]


def _csv(text):
    """A table's rows by their first cell, read below the first row that heads three columns or more."""
    out, headed = [], False
    for n, row in enumerate(csv.reader(text.splitlines()), 1):
        first = row[0].strip() if row else ''
        if headed and first:
            out.append((first, first, '', n))
        headed = headed or sum(bool(c.strip()) for c in row) >= 3
    return out


def _words(pattern):
    found = re.compile(pattern, re.M)
    return lambda text: [(m.group(1), m.group(0).strip(), '', _line(text, m.start()))
                         for m in found.finditer(text)]


#: A kind's symbols from its text (a module's from its tree, in `build`).
SYMBOLS = {'.h': _h, '.c': _c, '.lisp': _lisp, '.md': _md, '.asc': _asc, '.csv': _csv,
           '.ps1': _words(r'^\s*[Ff]unction\s+([\w-]+)'),
           '.cs': _words(r'^\s*(?:(?:public|internal|private|sealed|static|partial|abstract)\s+)*'
                         r'(?:class|struct|interface|enum)\s+(\w+)')}


def build(cache: str | None = CACHE):
    """The atlas of the tree as it stands: its nodes, then its ties. A file's rows - all the build reads off
    it alone (`_file`) - are taken from `cache` while its stamp holds, else read afresh, and the cache laid
    anew when any was; the ties are joined from them all (`_ties`). None: no cache."""
    atlas, parts, files, fresh = Atlas(), collections.defaultdict(list), {}, False
    kept = _kept(cache) if cache else {}
    for key, _, brief, _ in LAYERS:
        atlas.add(key, -1, LAYER, key, brief)
    for path in tracked():
        layer = layer_of(path)
        if layer is not None:
            parts[(path.rpartition('/')[0], layer)].append(path)
    split = {folder for folder, n in collections.Counter(f for f, _ in parts).items() if n > 1}
    for (folder, layer), paths in sorted(parts.items()):
        names = [p.rpartition('/')[2] for p in paths]
        here = (folder or '.') + '/'
        key = here + '@' + LAYERS[layer][0] if folder in split else here
        for path in paths:
            stamp, told = _stamp(path), kept.get(path)
            if told is None or stamp is None or told[STAMP] != stamp:
                told, fresh = _file(path, stamp), True
            files[path] = told
        up = atlas.add(key, layer, FOLDER, here, _purpose(folder, names, {n: files[p][BRIEF] for n, p in zip(names, paths)}))
        for path, name in zip(paths, names):
            node = atlas.add(path, up, FILE, name, files[path][BRIEF])
            for symbol, signature, told, line in files[path][DEFS]:
                atlas.add(path + '::' + symbol, node, SYMBOL, signature, told, line)
    _ties(atlas, files)
    if cache and (fresh or kept.keys() != files.keys()):
        _keep(cache, files)
    return atlas


def _file(path, stamp):
    """A file's rows (STAMP to CODES), read off it alone: read once, parsed once where it is a module and
    its tree let go - 843 trees held to the end had the collector walk them."""
    text = _read(path)
    kind = os.path.splitext(path)[1].lower()
    tree = _parse(text) if kind == '.py' else None
    return (stamp, _brief(path, text, tree), _py(tree) if tree is not None else SYMBOLS[kind](text) if kind in SYMBOLS else [],
            [_row(node) for node in _imported(tree)] if tree is not None else None,
            [(node.targets[0].id, node.value.value) for node in tree.body
             if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)
             and isinstance(node.value, ast.Constant) and type(node.value.value) is int] if tree is not None else None,
            sorted(set(NAMED.findall(text))) if path.lower().endswith(NAMING) else None,
            target_map.INCLUDE.findall(text) if path.endswith(('.c', '.h')) else None,
            COMMAND.findall(text) if path.endswith('.h') else None,
            target_map.ROW.findall(text) if path.endswith('.c') else None)


def _imported(tree):
    """A module's import statements alone, wherever they stand - a function's, a branch's, a handler's -
    in the order ast.walk meets them: its statements walked breadth first, not its expressions, where
    nine in ten of its nodes are (6.0 s of walking the build's trees whole, 0.38 s)."""
    found, todo = [], collections.deque([tree])
    while todo:
        node = todo.popleft()
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            found.append(node)
            continue
        for field in node._fields:
            value = getattr(node, field, None)
            if isinstance(value, list):
                todo.extend(item for item in value if isinstance(item, (ast.stmt, ast.excepthandler, ast.match_case)))
    return found


def _row(node):
    """An import statement as a row kept: (None, names, 0) an import, (module, names, level) a from - a name
    (name, as)."""
    names = tuple((a.name, a.asname) for a in node.names)
    return (None, names, 0) if isinstance(node, ast.Import) else (node.module, names, node.level)


def _statements(rows):
    """A module's import statements again off their rows, as host_map.imports reads them."""
    def aliases(names):
        return [ast.alias(name=n, asname=a) for n, a in names]
    return ast.Module(body=[ast.Import(names=aliases(names)) if module is None and level == 0
                            else ast.ImportFrom(module=module, names=aliases(names), level=level)
                            for module, names, level in rows], type_ignores=[])


def _ties(atlas, files):
    """The ties between the atlas's files, joined from their rows (`_file`)."""
    known = {path[5:-3].replace('/', '.').replace('.__init__', ''): path
             for path, told in files.items() if told[IMPORTS] is not None and path.startswith('host/')}
    for dotted, path in known.items():
        imports = _statements(files[path][IMPORTS])
        for dep in host_map.imports(path[5:], dotted, imports, known):
            atlas.join(path, known[dep], TIES.index('imports'))
        # A script's own folder is on its path, so a bare name there is its sibling (tests' kits); in a
        # package an import is absolute and no sibling's.
        folder = path.rpartition('/')[0]
        for node in imports.body if folder + '/__init__.py' not in atlas.at else ():
            for name in ([a.name for a in node.names] if isinstance(node, ast.Import)
                         else [node.module] if isinstance(node, ast.ImportFrom) and node.module and not node.level
                         else []):
                atlas.join(path, '%s/%s.py' % (folder, name.partition('.')[0]), TIES.index('imports'))
    resolve = target_map.resolver(sorted({p.split('/')[0] for p in files if p.endswith(('.c', '.h'))}))
    for path, told in files.items():
        for header in target_map.resolved(path, told[INCLUDES], resolve) if told[INCLUDES] is not None else ():
            atlas.join(path, header, TIES.index('includes'))
    tails = collections.defaultdict(list)
    for path in files:
        tails[path.rpartition('/')[2]].append(path)
    for path, told in files.items():
        for said in told[NAMES] or ():
            said = said.strip('./')
            ends = [p for p in tails.get(said.rpartition('/')[2], ()) if p.endswith(said)]
            if len(ends) == 1:
                atlas.join(path, ends[0], TIES.index('names'))
    _wire(atlas, files)


def _wire(atlas, files):
    """A command named on both ends of the wire: CMD_X in a handler's row, X bound to its number in a module
    (its top-level numbers)."""
    number, bound = {}, collections.defaultdict(list)
    for told in files.values():
        number.update((name, int(value, 0)) for name, value in told[COMMANDS] or ())
    for path, told in files.items():
        for name, value in told[NUMBERS] or ():
            bound[(name, value)].append(path)
    for path, told in files.items():
        for code in told[CODES] or ():
            for module in bound.get((code[0][4:], number.get(code[0][4:])), ()):
                atlas.join(module, path, TIES.index('wire'))


def _stamp(path):
    """A file's size and time as it stands; None where there is none to read."""
    try:
        st = os.stat(os.path.join(REPO, path))
    except OSError:
        return None
    return st.st_size, st.st_mtime_ns


def _code():
    """What a file's rows hang on beside the file: the code that reads it - this module, host_map,
    target_map, structure.py - and the interpreter."""
    digest = hashlib.sha256(sys.version.encode())
    for module in [__file__] + [m.__file__ or '' for m in (host_map, target_map, structure)]:
        with open(module, 'rb') as f:
            digest.update(f.read())
    return digest.hexdigest()


def _kept(cache):
    """{path: its rows} the cache keeps for this code; none where it keeps none, another code's, or less."""
    try:
        with open(cache, 'rb') as f:
            code, files = marshal.loads(f.read())
    except (OSError, EOFError, ValueError, TypeError):
        return {}
    return files if code == _code() and isinstance(files, dict) else {}


def _keep(cache, files):
    """The cache laid anew, whole or not at all: written beside it, then put in its place."""
    beside = '%s.%d' % (cache, os.getpid())
    try:
        os.makedirs(os.path.dirname(cache), exist_ok=True)
        with open(beside, 'wb') as f:
            f.write(marshal.dumps((_code(), files)))
        os.replace(beside, cache)
    except OSError:
        with contextlib.suppress(OSError):
            os.remove(beside)
