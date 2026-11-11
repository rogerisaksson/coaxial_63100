#!/usr/bin/env python3
"""A new machine's needs written where setup reads them (`tools.dev.depends`), on trees and machines laid here.

A distribution laid in a folder - its module of another name, one extra met, one not - is asked by the name a
requirements file writes; a tree laid in a folder imports it by a statement, by a name and in a notebook, and
looks two programs up; and the tree itself, as it stands: every module written, every requirements file read
by setup, every program named there. The same pre-commit removes every recording (`pre_commit.purged`)."""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The distribution depends-fake 1.4 as an installer leaves it: an extra that asks itself, one that asks an absent.
METADATA = ('Metadata-Version: 2.1\nName: depends-fake\nVersion: 1.4\nProvides-Extra: less\nProvides-Extra: more\n'
            'Requires-Dist: depends-fake>=1; extra == "less"\nRequires-Dist: depends-fake-absent>=1; extra == "more"\n')

#: A script of a tree: the standard library's, another's module by a statement, its own, a name, two programs.
SCRIPT = ('import os, json\nimport depends_fake_mod.part\nfrom own import thing\n'
          'import importlib, shutil, subprocess\n'
          "importlib.import_module('depends_never_mod.sub')\n"
          "shutil.which('depends-tool')\nsubprocess.run(['depends-other', '-v'])\nos.path.join(['depends-not'])\n")

#: A module of its own: an import nested in a function's try, in a file that names no sought call.
DEEP = 'def f():\n    try:\n        import depends_fake_mod\n    except ImportError:\n        pass\n'

#: A notebook of it: a cell an editor's own line keeps from parsing, and a cell that is no code.
BOOK = {'cells': [{'cell_type': 'code', 'source': ['%matplotlib inline\n', 'from depends_fake_mod import thing\n']},
                  {'cell_type': 'markdown', 'source': ['import depends_prose_mod']}]}


def _written(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(text)


class Laid:
    """The distribution depends-fake on the path while its block runs: its module is `depends_fake_mod`."""

    def __init__(self, top):
        self.site = os.path.join(top, 'site')
        for name, text in (('METADATA', METADATA), ('top_level.txt', 'depends_fake_mod\n')):
            _written(os.path.join(self.site, 'depends_fake-1.4.dist-info', name), text)

    def __enter__(self):
        sys.path.insert(0, self.site)

    def __exit__(self, *_exc):
        sys.path.remove(self.site)


def _tree(top, requirements, setup, more=None):
    """A tree under `top`: its requirements file, its setup script, a module of its own, SCRIPT and BOOK."""
    root = os.path.join(top, 'tree')
    for path, text in (('host/requirements.txt', requirements), ('setup.ps1', setup), ('host/own/__init__.py', ''),
                       ('host/own/deep.py', DEEP),
                       ('host/script.py', SCRIPT), ('book.ipynb', json.dumps(BOOK))) + (
                           (('host/requirements-more.txt', more),) if more is not None else ()):
        _written(os.path.join(root, *path.split('/')), text)
    return root


def test_a_requirement_is_asked_by_the_name_it_writes(report):
    """`depends.asked`, `met` and `absent` on a requirements file and a distribution laid here."""
    from tools.dev import depends
    with tempfile.TemporaryDirectory() as top:
        path = os.path.join(top, 'requirements.txt')
        _written(path, '# a comment\n\n--extra-index-url https://example.invalid/whl\n'
                       'depends-fake>=1.2,<2   # its module is depends_fake_mod\nDepends_Fake[less]==1.4\n'
                       'depends-fake[more]\ndepends-fake>=2\ndepends-fake-absent\n')
        asked = depends.asked(path)
        with Laid(top):
            absent = depends.absent(path)
    report.check("a requirements file read: comments, blank lines and pip's options left out, extras and versions kept",
                 asked == [('depends-fake', (), '>=1.2,<2'), ('Depends_Fake', ('less',), '==1.4'),
                           ('depends-fake', ('more',), ''), ('depends-fake', (), '>=2'), ('depends-fake-absent', (), '')],
                 asked)
    report.check('a distribution asked by the name written, its module of another: here; one short of an extra, '
                 'older than asked or absent: said', absent == ['depends-fake[more]', 'depends-fake', 'depends-fake-absent'],
                 absent)
    held = [depends.met(have, spec) for have, spec in (
        ('3.14', '==3.14.0'), ('2.14.1+cu130', '>=2.14'), ('13.4.3', '==13.*'), ('1.9', '>=1.2,<2'), ('2.0', '>=1.2,<2'),
        ('1.4.2', '~=1.4'), ('1.5', '~=1.4.2'), ('1.4', '!=1.4'), ('14.2.0', ''))]
    report.check('a version against what is asked of it: a release padded, a local label cut, a wildcard a prefix',
                 held == [True, True, True, True, False, True, False, False, True], held)


def test_a_module_s_distribution_is_written(report):
    """`depends.told` and `faults` on a tree laid here: what it imports and seeks, written and named or not."""
    from tools.dev import depends
    with tempfile.TemporaryDirectory() as top, Laid(top):
        root = _tree(top, 'depends-fake>=1\n', '# reads requirements.txt, checks depends-tool and Depends-Other.exe\n')
        modules, programs, unparsed = depends.told(depends.sources(root))
        clean, counted = depends.faults(root)
    report.check('a tree read: another\'s modules by a statement - a nested one too -, by a name and in a '
                 'notebook\'s code; its programs',
                 modules == {'depends_fake_mod': {'host/script.py', 'host/own/deep.py', 'book.ipynb'},
                             'depends_never_mod': {'host/script.py'}}
                 and programs == {'depends-tool': {'host/script.py'}, 'depends-other': {'host/script.py'}}
                 and unparsed == [], '%s %s %s' % (modules, programs, unparsed))
    report.check('all written and named: no fault, a module no distribution here holds said and not judged',
                 clean == [] and 'not here, not judged: depends_never_mod' in counted, '%s | %s' % (clean, counted))
    with tempfile.TemporaryDirectory() as top, Laid(top):
        found, _counted = depends.faults(_tree(top, '', '# nothing of it\n', more='depends-fake\n# of no setup\n'))
        short, _counted = depends.faults(_tree(os.path.join(top, 'other'), '# none\n', 'requirements.txt depends-tool\n'))
    report.check('a requirements file no setup reads, programs it never names: each a fault, the module written',
                 len(found) == 4 and found[0].startswith('host/requirements-more.txt')
                 and found[1].startswith('host/requirements.txt') and found[2].startswith('depends-other (host/script.py)')
                 and found[3].startswith('depends-tool (host/script.py)'), found)
    report.check('a module in no requirements file: a fault that names its distribution and its files',
                 len(short) == 2 and short[0].startswith('depends_fake_mod (book.ipynb and 2 more) comes with depends-fake')
                 and short[1].startswith('depends-other'), short)


def test_the_tree_writes_what_it_asks(report):
    """The tree as it stands: nothing a new machine would lack unwritten."""
    from tools.dev import depends
    found, counted = depends.faults()
    report.check('the tree: every module in a requirements file, each of those read by setup, every program named '
                 'there', found == [], '; '.join(found) or counted)


def test_a_commit_removes_every_recording(report):
    """`pre_commit.purged` on a folder laid here: each recording, in it and in a folder of it, removed; the
    folder kept, emptied."""
    import shutil
    from tools.dev import pre_commit
    folder = tempfile.mkdtemp()
    os.mkdir(os.path.join(folder, 'sub'))
    for name in ('page.csv', 'gate.jsonl', os.path.join('sub', 'gate.jsonl')):
        with open(os.path.join(folder, name), 'w', encoding='utf-8') as f:
            f.write('{}\n')
    got = pre_commit.purged(folder)
    left = os.listdir(folder)
    shutil.rmtree(folder, ignore_errors=True)
    report.check('three recordings removed, none kept, the folder left empty', got == (3, []) and left == [],
                 (got, left))


ROSTER = (test_a_requirement_is_asked_by_the_name_it_writes, test_a_module_s_distribution_is_written,
          test_the_tree_writes_what_it_asks, test_a_commit_removes_every_recording)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
