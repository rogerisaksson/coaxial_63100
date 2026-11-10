#!/usr/bin/env python3
"""README.md out of its source and the tree: its lists read off the code, its claims held to it.

    python tools/dev/readme.py            # README.md laid where behind; its faults said, 1 where any
    python tools/dev/readme.py --behind   # 'README.md' where it is behind its sources, nothing where not

The README is written in docs/README.src.md. A line there that is `<!-- name -->` alone is laid as what
the tree says now (FACTS): a page, a package, a machine type, a core, a paper, a suite or a stage of
setup is in the README at the commit that adds it - the pre-commit lays it into that commit. What stays
written by hand is held to the tree (`unheld`): a path in backticks, a link, a picture or a command
names what git tracks; an example parses, and what it imports from host/'s packages is there by that
name; every top-level folder, every git hook and every document of docs/ is named. One that is not
stops the commit. Written by hand alone, the README's one generated section stood 1 392 commits old,
it linked a document that was gone and told of a folder that was.
"""
import ast
import os
import re
import sys
import textwrap

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.dev import depends

REPO = depends.REPO

#: The README's source, the README, the page tools/dev/cover.py writes and the marks about its tables.
SOURCE, LAID, RESULTS = 'docs/README.src.md', 'README.md', 'docs/RESULTS.md'
MARKS = ('<!-- coverage -->', '<!-- /coverage -->')

#: The README's last line: where it is written.
NOTE = '<!-- Laid by host/tools/dev/readme.py from docs/README.src.md: write that. -->'


class Tree:
    """A tree as the README is laid from it: `paths` what git tracks there - a commit's index, or `root`'s
    own -, its files read under `root`."""

    def __init__(self, root=REPO, paths=None):
        self.root = root
        self.paths = set(depends.files(root) if paths is None else paths)
        self.folders = {p[:k] for p in self.paths for k, c in enumerate(p) if c == '/'}

    def read(self, path):
        with open(os.path.join(self.root, *path.split('/')), encoding='utf-8') as f:
            return f.read()

    def under(self, folder, end=''):
        """The tracked paths that begin `folder` and end `end`, in order."""
        return sorted(p for p in self.paths if p.startswith(folder) and p.endswith(end))


def _first(text):
    """The first sentence of a docstring or a paragraph on one line, its full stop cut."""
    flat = ' '.join((text or '').strip().split('\n\n')[0].split())
    end = re.search(r'\.(?=\s|$)', flat)
    return flat[:end.start()] if end else flat


def _told(tree, path):
    """What a module says it is: its docstring's first sentence."""
    return _first(ast.get_docstring(ast.parse(tree.read(path))))


def _keys(tree, path, name):
    """The keys of the dict a module assigns `name`, in its order."""
    for node in ast.parse(tree.read(path)).body:
        if (isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', None) == name
                and isinstance(node.value, ast.Dict)):
            return [str(key.value) for key in node.value.keys if isinstance(key, ast.Constant)]
    raise ValueError('%s assigns no dict %s' % (path, name))


def _suites(tree):
    return [p for p in tree.under('host/tests/test_', '.py') if p.count('/') == 2]


def pages(tree, _source):
    """The front page's pages: a module each in host/terminal/pages/, its title and what it shows its own."""
    rows = []
    for path in tree.under('host/terminal/pages/', '.py'):
        name = path.rsplit('/', 1)[1][:-3]
        if not name.startswith('_'):
            title, _, shows = _told(tree, path).partition(': ')
            rows.append('| `%s` | %s | %s |' % (name, title, shows))
    return ['%d pages on the front page, each drawn in braille on the terminal:' % len(rows), '',
            '| page | titled | what it shows |', '| --- | --- | --- |'] + rows


def packages(tree, _source):
    """host/'s packages, each by what its own __init__ says it is."""
    rows = ['| `%s/` | %s |' % (path.rsplit('/', 1)[0], _told(tree, path))
            for path in tree.under('host/', '/__init__.py') if path.count('/') == 2]
    return ['| package | what it is |', '| --- | --- |'] + rows


def types(tree, _source):
    """The machine types `Machine.discover` knows: the keys of machine.rig.routines.TYPES."""
    names = _keys(tree, 'host/machine/rig/routines.py', 'TYPES')
    return textwrap.wrap('%d machine types, each its limbs and its routines (`Machine.discover`): %s.'
                         % (len(names), ', '.join('`%s`' % n for n in names)), 79)


def cores(tree, source):
    """The portable cores: the top-level folders of inc/ and src/ the source does not name itself - the
    layers above them are written there."""
    named = set(re.findall(r'`([\w.-]+)/`', source))
    found = sorted(f for f in tree.folders if '/' not in f and f not in named
                   and f + '/inc' in tree.folders and f + '/src' in tree.folders)
    return textwrap.wrap('The %d portable cores: %s.' % (len(found), ' '.join('`%s/`' % f for f in found)), 79)


def notebooks(tree, _source):
    """The papers: the keys of tools.notebooks.MODULES, in their reading order."""
    names = _keys(tree, 'host/tools/notebooks/__init__.py', 'MODULES')
    return textwrap.wrap('%d executed papers in `notebook_examples/`, each written from its module in '
                         '`host/tools/notebooks/`: %s.' % (len(names), ', '.join(names)), 79)


def suites(tree, _source):
    """How many suites host/tests/ holds."""
    return ['%d suites in `host/tests/`, each a plain script with its own tally.' % len(_suites(tree))]


def setup(tree, _source):
    """Setup's stages, as setup.ps1's own description lists them: a name, what it does, where."""
    listed = re.search(r'\.DESCRIPTION\r?\n(.*?)\r?\n\s*Areas:', tree.read('setup.ps1'), re.S)
    if listed is None:
        raise ValueError('setup.ps1 describes no stages before its Areas')
    rows = []
    for line in listed.group(1).splitlines():
        name, rest = line.split(None, 1)
        does, where = rest.rsplit(None, 1)
        rows.append('| `%s` | %s | `%s` |' % (name, does.strip(), where.replace('\\', '/')))
    return ['| stage | what it does | where |', '| --- | --- | --- |'] + rows


def python(tree, _source):
    """The Pythons CI runs the host on, and the host's package and its version."""
    gate = re.search(r'python:[ \t]*\[([^\]]*)\]', tree.read('.github/workflows/host.yml'))
    version = re.search(r'^version\s*=\s*"([^"]+)"', tree.read('host/pyproject.toml'), re.M)
    if gate is None or version is None:
        raise ValueError('host.yml names no pythons, or pyproject.toml no version')
    runs = re.findall(r'\d+\.\d+', gate.group(1))
    return textwrap.wrap('Python %s to %s is what CI runs the host on. The host is one package, '
                         '`coaxial63100` %s: `pip install -e host/`, which setup does.'
                         % (runs[0], runs[-1], version.group(1)), 79)


def results(tree, _source):
    """The last measured run: what tools/dev/cover.py wrote above its tables, and where they are - where it
    wrote any."""
    page = tree.read(RESULTS).split(MARKS[0], 1)[1].split(MARKS[1], 1)[0]
    tables = ['', 'Each suite and each part: [docs/RESULTS.md](docs/RESULTS.md).'] if '<details>' in page else []
    return page.split('<details>', 1)[0].strip().splitlines() + tables


#: What a line `<!-- name -->` of the source is laid as: (the tree, the source's text) -> its lines.
FACTS = {'pages': pages, 'packages': packages, 'types': types, 'cores': cores, 'notebooks': notebooks,
         'suites': suites, 'setup': setup, 'python': python, 'results': results}


def _named(text):
    """The words of `text`, a README laid, that may name a file: what stands in backticks, a link's and a
    picture's target, and each word of a block that is not Python."""
    blocks = re.findall(r'^```(\w*)\n(.*?)^```', text, re.M | re.S)
    prose = re.sub(r'^```.*?^```', '', text, flags=re.M | re.S)
    words = set(re.findall(r'`([^`\s]+)`', prose)) | set(re.findall(r'\]\(([^)#\s]+)\)', prose))
    words |= set(re.findall(r'src="([^"]+)"', prose))
    for kind, body in blocks:
        if kind != 'python':
            words |= set(body.split())
    return words, [body for kind, body in blocks if kind == 'python']


def unheld(tree, text):
    """What `text`, a README laid, says that the tree does not hold: a path no file or folder of it; an
    example that does not parse, or imports from host/ what is not there; a top-level folder, a git hook
    or a document of docs/ it never names."""
    found = []
    tops = {p.split('/')[0] for p in tree.paths}
    words, examples = _named(text)
    for word in sorted(words):
        path = word.replace('\\', '/').rstrip(':,;)')
        path = (path[2:] if path.startswith('./') else path).rstrip('/')
        if (re.fullmatch(r'[\w.-]+(/[\w.-]+)+', path) and path.split('/')[0] in tops
                and path not in tree.paths and path not in tree.folders):
            found.append('`%s` is named, and the tree holds no such file or folder' % word)
    for body in examples:
        try:
            nodes = list(ast.walk(ast.parse(body)))
        except SyntaxError as fault:
            found.append('an example does not parse (%s): %s' % (fault.msg, body.strip().split('\n')[0]))
            continue
        for node in nodes:
            if (not isinstance(node, ast.ImportFrom) or not node.module
                    or 'host/' + node.module.split('.')[0] not in tree.folders):
                continue
            module = node.module
            base = 'host/' + module.replace('.', '/')
            home = next((p for p in (base + '.py', base + '/__init__.py') if p in tree.paths), None)
            if home is None:
                found.append('an example imports from %s, which host/ does not hold' % module)
                continue
            held = tree.read(home)
            found += ['an example imports %s from %s, which names none' % (alias.name, module)
                      for alias in node.names if not re.search(r'\b%s\b' % re.escape(alias.name), held)]
    found += ['the tree holds %s/, which the README names nowhere' % folder
              for folder in sorted(f for f in tree.folders if '/' not in f and not f.startswith('.'))
              if '`%s/' % folder not in text]
    found += ['git runs .githooks/%s, which the README names nowhere' % path.split('/')[1]
              for path in tree.under('.githooks/') if '`%s`' % path.split('/')[1] not in text]
    found += ['%s is linked nowhere in the README' % path for path in tree.under('docs/', '.md')
              if path.count('/') == 1 and not path.startswith('docs/DONE_') and path != SOURCE
              and '(%s)' % path not in text]
    return found


def made(tree=None):
    """(the README's text, its faults) off `tree`: the source with each `<!-- name -->` line laid as the
    tree's fact; a fact not read, a name that is none, and what `unheld` finds are the faults."""
    tree = Tree() if tree is None else tree
    source = tree.read(SOURCE)
    lines, faults = [], []
    for line in source.rstrip('\n').split('\n'):
        name = re.fullmatch(r'<!-- (\w+) -->', line)
        if name is None:
            lines.append(line)
        elif name.group(1) not in FACTS:
            faults.append('%s asks `%s`, which is no fact (%s)' % (SOURCE, name.group(1), ', '.join(FACTS)))
        else:
            try:
                lines += FACTS[name.group(1)](tree, source)
            except (OSError, ValueError, SyntaxError, IndexError) as fault:
                faults.append('`%s` could not be read off the tree: %s' % (name.group(1), fault))
    text = '\n'.join(lines) + '\n\n' + NOTE + '\n'
    return text, faults + unheld(tree, text)


def stands(root=REPO):
    """The README as it stands under `root`; None where none does."""
    try:
        with open(os.path.join(root, LAID), encoding='utf-8', newline='') as f:
            return f.read()
    except OSError:
        return None


def main(argv=None):
    """`--behind`: 'README.md' where it is not its sources' text, 0; else the README laid where it is
    behind, its faults said, 1 where any."""
    argv = sys.argv[1:] if argv is None else argv
    text, faults = made()
    behind = stands() != text
    if argv[:1] == ['--behind']:
        print(LAID if behind else '')
        return 0
    if behind:
        with open(os.path.join(REPO, LAID), 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
        print('readme: %s laid' % LAID)
    for fault in faults:
        print(fault)
    return 1 if faults else 0


if __name__ == '__main__':
    sys.exit(main())
