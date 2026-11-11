#!/usr/bin/env python3
"""Every source folder, what it is for, and its files' synopses: the project's map before any search.

    python -X utf8 docs/structure.py               # every folder: its files' count, its purpose
    python -X utf8 docs/structure.py host/machine  # that folder and those under it, each file's synopsis
    python -X utf8 docs/structure.py --files       # the whole tree, every file's synopsis

A folder's purpose is its lead file's synopsis: a Python package's __init__.py, else the file named after the
folder (its component's, for an inc/src/test folder); a folder with none is the heads of its files' synopses,
WIDTH characters of them. A file's synopsis is its first sentence: a module's docstring, a C file's opening comment
(a vendor's @brief), a Lisp text's SYNOPSIS or DOMAIN line or its first comment, else what its package exports.
Read from the files as they are, nothing kept.
"""
import argparse
import ast
import io
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KINDS = ('.py', '.c', '.h', '.lisp')
#: Trees left out: the vendor's drivers, what nothing reaches, build output.
SKIP = ('drivers/', 'deprecated/', 'build/')
ROLES = {'inc': 'headers', 'src': 'sources', 'test': 'tests'}
#: A folder's purpose compiled from its files' synopses is cut at WIDTH characters, each file's head at HEAD.
WIDTH, HEAD = 200, 64


def tracked():
    """{folder: [its source files]} of every source the repo tracks outside SKIP."""
    out = subprocess.run(['git', 'ls-files', '-z'], cwd=REPO, capture_output=True, check=True).stdout
    folders = {}
    for path in out.decode('utf-8').split('\0'):
        if path.endswith(KINDS) and not path.startswith(SKIP) and os.path.isfile(os.path.join(REPO, path)):
            folder, name = path.rpartition('/')[::2]
            folders.setdefault(folder or '.', []).append(name)
    return {f: sorted(n) for f, n in folders.items()}


def sentence(text):
    """`text`'s first sentence on one line."""
    flat = ' '.join(text.split())
    end = re.search(r'[.!?](\s|$)', flat)
    return flat[:end.end()].strip() if end else flat


def _python(text):
    try:
        doc = ast.get_docstring(ast.parse(text)) or ''
    except SyntaxError:
        return ''
    return sentence(doc.split('\n\n')[0])


def _c(name, text):
    comments = [m.group(1) for m in re.finditer(r'/\*(.*?)\*/', text[:4000], re.S) if 'USER CODE' not in m.group(1)]
    if not comments:
        return ''
    body = next((c for c in comments if '@brief' in c), comments[0])
    said = re.search(r'@brief\s*:?\s*(.+)', body)
    lines = [said.group(1)] if said else [re.sub(r'^[\s*]+', '', line) for line in body.splitlines()]
    words = ' '.join(line for line in lines if line.strip() and not set(line.strip()) <= set('*-='))
    return sentence(re.sub(r'^\S*%s\s+-\s+' % re.escape(name), '', words.strip()))


def _banner(line):
    """Whether a Lisp header's line is a title in capitals (its own path in parentheses aside)."""
    title = re.sub(r'\([^)]*\)', '', line)
    return title == title.upper()


def _lisp(text):
    head = []
    for line in text.splitlines():
        if not line.startswith(';'):
            break
        said = line.lstrip(';').strip()
        if said and not set(said) <= set('-=') and not said.startswith(('FILE:', 'REQUIRES:', 'MODULE')):
            head.append(said)
    tagged = next((h for tag in ('SYNOPSIS:', 'DOMAIN:') for h in head if h.startswith(tag)), None)
    told = [h for h in head if not _banner(h)] or head
    said = (tagged.partition(':')[2] if tagged else ' '.join(told)).strip()
    if said:
        return sentence(said)
    exports = re.search(r'\(:export(.*?)\)\)', text, re.S)
    return 'exports ' + ', '.join(re.findall(r'#:([\w*+-]+)', exports.group(1))) + '.' if exports else ''


def synopsis(folder, name, text=None):
    """`name`'s first sentence, read from it - from `text` where its caller has read it already."""
    if text is None:
        with open(os.path.join(REPO, folder, name), encoding='utf-8', errors='replace') as f:
            text = f.read()
    kind = os.path.splitext(name)[1]
    return _python(text) if kind == '.py' else _lisp(text) if kind == '.lisp' else _c(name, text)


def head(said):
    """A synopsis to its first colon or semicolon, HEAD characters of it at most: what the file is, not how."""
    first = re.split(r'[:;]\s', said, maxsplit=1)[0].rstrip('.')
    return first if len(first) <= HEAD else first[:HEAD].rpartition(' ')[0].rstrip(',')


def purpose(folder, names, synopses=None):
    """What `folder` is for: its lead file's synopsis, else its files' synopses' heads, WIDTH characters of
    them - `synopses` {name: synopsis} where its caller has them, else each read."""
    def told(name):
        return synopses[name] if synopses is not None else synopsis(folder, name)
    base = folder.rpartition('/')[2]
    component = folder.rpartition('/')[0].rpartition('/')[2] if base in ROLES else base
    lead = next((n for n in ('__init__.py',) + tuple(component + k for k in KINDS) if n in names), None)
    if lead:
        said = told(lead)
    else:
        heads = '; '.join(dict.fromkeys(filter(None, (head(told(n)) for n in names))))
        said = heads if len(heads) <= WIDTH else (heads[:WIDTH].rpartition('; ')[0] or heads[:WIDTH]) + '; ...'
    return ('%s %s: %s' % (component, ROLES[base], said)) if base in ROLES else said


def show(folders, under=None, files=False):
    """The lines for every folder (under `under`), each file's synopsis where `files`."""
    out = []
    for folder in sorted(folders):
        if under and not (folder == under or folder.startswith(under + '/')):
            continue
        names = folders[folder]
        out.append('%s/ (%d) - %s' % (folder, len(names), purpose(folder, names)))
        if files:
            out += ['    %s - %s' % (n, synopsis(folder, n)) for n in names]
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('under', nargs='?', help="a folder from the repo's root: it and those under it, each file")
    parser.add_argument('--files', action='store_true', help='every file, the whole tree')
    args = parser.parse_args(argv)
    under = args.under and args.under.replace('\\', '/').strip('/')
    lines = show(tracked(), under, files=args.files or bool(under))
    if not lines:
        print('no source folder %s' % under, file=sys.stderr)
        return 1
    if isinstance(sys.stdout, io.TextIOWrapper):   # a cp1252 pipe cannot carry every synopsis
        sys.stdout.reconfigure(encoding='utf-8')
    print('\n'.join(lines))
    return 0


if __name__ == '__main__':
    sys.exit(main())
