#!/usr/bin/env python3
"""Machines named, not "it": each prose sentence that opens on It or Its, sieved by test_layout.

    python -X utf8 tools/dev/named.py               # every such sentence in the tree, file:line
    python -X utf8 tools/dev/named.py FILE ...      # in these files alone

Prose is a comment, a docstring or other string, a Markdown paragraph, list item, heading or table
cell (`regions`); a sentence ends at a stop before a space or at a blank line (`sentences`). A
sentence that opens on It or Its leaves its reader to guess the thing; the robot, the quad, the
helicopter, the ornithopter, the aerobat or whatever else it speaks of is named instead (CLAUDE.md,
rule 8). Generated files, the vendor's code and deprecated/ are not read.
"""
import ast
import bisect
import io
import os
import re
import subprocess
import sys
import tokenize

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO
from tools.dev.token_budget import GENERATED

#: The suffixes whose comments open on a hash; Python and Markdown are parsed apart.
HASHED = ('.toml', '.yml', '.yaml', '.ps1', '.sh', '.cmake')
#: The suffixes read: C's and C#'s comments alike, Lisp's comments and strings.
READ = ('.py', '.md', '.c', '.h', '.cs', '.lisp') + HASHED
#: Paths never read: the vendor's code and its generated HAL, code kept for reference.
SKIPPED = ('drivers/', 'cmake/', 'core/', 'deprecated/')
#: What opens a sentence on the pronoun: marks and brackets before It or Its.
OPENS = re.compile(r"\s*[(\[\"'*_~-]*(It|Its|It's)(?![A-Za-z])")
#: A stop before a space, or a blank line.
STOP = re.compile(r"[.!?][\"')\]]*(?=\s)|\n[ \t]*\n")
#: Words a stop follows inside a sentence.
ABBREVIATIONS = {'e.g', 'i.e', 'etc', 'vs', 'cf', 'approx', 'incl', 'resp', 'viz', 'al'}
_QUOTES = re.compile(r'(?<![A-Za-z0-9])[rRbBuUfFtT]{0,2}(?:"""|\'\'\'|"|\')')
#: Each kind's marks, blanked before its sentences are read: comment marks, quotes, list marks.
MARKS = {
    'comment': re.compile(r'^[ \t]*#[:!]?', re.M),
    'doc': _QUOTES,
    'string': _QUOTES,
    'cblock': re.compile(r'/\*+!?<?|\*+/|^[ \t]*\*+', re.M),
    'cline': re.compile(r'^[ \t]*//[/!]?<?', re.M),
    'lcomment': re.compile(r'^[ \t]*;+', re.M),
    'ldoc': re.compile(r'"'),
    'para': re.compile(r'^[ \t]*>?', re.M),
    'item': re.compile(r'^[ \t]*(?:[-*+]|\d+[.)])[ \t]+|^[ \t]*>', re.M),
    'heading': re.compile(r'^[ \t]*#+'),
    'cell': re.compile(r'^$'),
}
#: Inline code, a word of its own; a link's target, no words.
CODE, TARGET = re.compile(r'`[^`\n]*`'), re.compile(r'\]\([^)\s]*\)')
_C = re.compile(r'"(?:[^"\\\n]|\\.)*"|\'(?:[^\'\\\n]|\\.)*\'|/\*.*?\*/|//[^\n]*', re.S)
_LISP = re.compile(r'#\\.|"(?:[^"\\]|\\.)*"|;[^\n]*', re.S)
_HASH = re.compile(r'"(?:[^"\\\n]|\\.)*"|\'[^\'\n]*\'|#[^\n]*')
_ITEM = re.compile(r'\s*(?:[-*+]|\d+[.)])\s+')
_LINK = re.compile(r'\s*\[[^\]]+\]:\s')


def generated(text):
    """Whether a tool writes the file: its first line says so (token_budget.GENERATED)."""
    return GENERATED in text.split('\n', 1)[0]


def rows(text):
    """Each line's first character's offset."""
    return [0] + [m.end() for m in re.finditer('\n', text)]


def _alone(text, start, end):
    """Whether [start, end) stands alone on its lines: nothing but blanks before or after it."""
    head, tail = text.rfind('\n', 0, start) + 1, text.find('\n', end)
    return not text[head:start].strip() and not text[end:len(text) if tail < 0 else tail].strip()


def _blocked(found, text, kind, start, end, mark):
    """A comment joined to the block above it when both stand alone, on consecutive lines, at one
    column, under one mark; else a region of its own, flowing only if it stands alone."""
    alone = _alone(text, start, end)
    key = (start - text.rfind('\n', 0, start) - 1, mark)
    last = found[-1] if found else None
    if alone and last and last[0] == kind and last[3] and last[4] == key and \
            text.count('\n', last[2], start) == 1:
        last[2] = end
    else:
        found.append([kind, start, end, alone, key])


def _python(text):
    """Python's prose: comment blocks, docstrings and strings, as [kind, start, end, flows, key]."""
    try:
        tokens = list(tokenize.generate_tokens(io.StringIO(text).readline))
        tree = ast.parse(text)
    except (SyntaxError, tokenize.TokenError, ValueError):
        return []
    at = rows(text)
    docs = {(n.body[0].lineno, n.body[0].col_offset): isinstance(n, ast.Module)
            for n in ast.walk(tree)
            if isinstance(n, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
            and n.body and isinstance(n.body[0], ast.Expr)
            and isinstance(n.body[0].value, ast.Constant) and isinstance(n.body[0].value.value, str)}
    opening = {getattr(tokenize, name, -1) for name in ('FSTRING_START', 'TSTRING_START')}
    closing = {getattr(tokenize, name, -1) for name in ('FSTRING_END', 'TSTRING_END')}
    found, pieces, depth, begun = [], [], 0, (0, (1, 0))
    for i, t in enumerate(tokens):
        if t.type in opening:
            begun = (i, t.start) if depth == 0 else begun
            depth += 1
        elif t.type in closing:
            depth -= 1
            if depth == 0:
                pieces.append([begun[0], i, begun[1], t.end])
        elif depth == 0 and t.type == tokenize.STRING:
            pieces.append([i, i, t.start, t.end])
        elif depth == 0 and t.type == tokenize.COMMENT and not t.string.startswith('#!'):
            start = at[t.start[0] - 1] + t.start[1]
            _blocked(found, text, 'comment', start, start + len(t.string), t.string[:2] == '#:')
    runs = []
    for first, last, start, end in pieces:
        if runs and all(tokens[k].type == tokenize.NL for k in range(runs[-1][1] + 1, first)):
            runs[-1][1], runs[-1][3] = last, end
        else:
            runs.append([first, last, start, end])
    for _first, _last, start, end in runs:
        kind = 'doc' if start in docs else 'string'
        found.append([kind, at[start[0] - 1] + start[1], at[end[0] - 1] + end[1], kind == 'doc',
                      docs.get(start, False)])
    return found


def _c(text):
    """C's and C#'s comments: blocks and runs of line comments, as [kind, start, end, flows, key]."""
    found = []
    for m in _C.finditer(text):
        if m.group().startswith('/*'):
            found.append(['cblock', m.start(), m.end(), _alone(text, m.start(), m.end()), None])
        elif m.group().startswith('//'):
            _blocked(found, text, 'cline', m.start(), m.end(), m.group()[:3])
    return found


def _lisp(text):
    """Lisp's comments and strings, as [kind, start, end, flows, key]."""
    found = []
    for m in _LISP.finditer(text):
        if m.group().startswith('"'):
            found.append(['ldoc', m.start(), m.end(), True, None])
        elif m.group().startswith(';'):
            marks = len(m.group()) - len(m.group().lstrip(';'))
            _blocked(found, text, 'lcomment', m.start(), m.end(), marks)
    return found


def _hashed(text):
    """A configuration's or a script's comments, as [kind, start, end, flows, key]."""
    found = []
    for m in _HASH.finditer(text):
        if m.group().startswith('#'):
            _blocked(found, text, 'comment', m.start(), m.end(), False)
    return found


def _markdown(text):
    """Markdown's prose: paragraphs and list items flowing, headings and table cells not."""
    found, at, fence, open_ = [], rows(text), None, None
    for i, line in enumerate(text.split('\n')):
        start, stripped = at[i], line.strip()
        fenced = re.match(r'\s*(```|~~~)', line)
        if fence:
            fence = None if stripped.startswith(fence) else fence
        elif fenced:
            fence, open_ = fenced.group(1), None
        elif not stripped or _LINK.match(line):
            open_ = None
        elif stripped[0] in '#|':
            found += [['heading', start, start + len(line), False, None]] if stripped[0] == '#' \
                else [['cell', start + c.start(), start + c.end(), False, None]
                      for c in re.finditer(r'[^|]+', line)]
            open_ = None
        elif _ITEM.match(line) or open_ is None:
            open_ = ['item' if _ITEM.match(line) else 'para', start, start + len(line), True, None]
            found.append(open_)
        else:
            open_[2] = start + len(line)
    return found


def regions(path, text):
    """The file's prose, in order: [kind, start, end, flows, key] - a comment's key its (column,
    mark), a docstring's whether it is the module's."""
    suffix = os.path.splitext(path)[1]
    parse = {'.py': _python, '.md': _markdown, '.c': _c, '.h': _c, '.cs': _c, '.lisp': _lisp}
    found = parse.get(suffix, _hashed if suffix in HASHED else lambda _text: [])(text)
    return sorted(found, key=lambda region: region[1])


def masked(text, region):
    """The region's text with its marks blanked, inline code a word of x and links' targets blank:
    as long as the region, so its offsets hold."""
    blank = lambda m: ' ' * len(m.group())
    chunk = MARKS[region[0]].sub(blank, text[region[1]:region[2]])
    return TARGET.sub(blank, CODE.sub(lambda m: 'x' * len(m.group()), chunk))


def sentences(clean):
    """(start, end) of each sentence in a region's masked text: ended by a stop before a space,
    an abbreviation's excepted, or by a blank line."""
    found, start = [], 0
    for m in STOP.finditer(clean):
        word = re.search(r'([A-Za-z.]+)$', clean[:m.start()]) if m.group()[0] == '.' else None
        if word and word.group(1).lower().strip('.') in ABBREVIATIONS:
            continue
        found.append((start, m.end()))
        start = m.end()
    return found + [(start, len(clean))]


def opened(path, text):
    """[(offset, sentence)] of each sentence in the file's prose that opens on It or Its."""
    found = []
    for region in regions(path, text):
        clean = masked(text, region)
        for start, end in sentences(clean):
            m = OPENS.match(clean, start, end)
            if m:
                found.append((region[1] + m.start(1), ' '.join(clean[start:end].split())))
    return found


def read(paths=None):
    """{path: text} of the files read - the tree's tracked ones, or those given - generated files,
    skipped paths and other suffixes left out."""
    if paths is None:
        listed = subprocess.run(('git', '-C', REPO, 'ls-files'), capture_output=True, text=True,
                                encoding='utf-8').stdout.split('\n')
        paths = [p for p in listed if p.endswith(READ) and not p.startswith(SKIPPED)]
    texts = {}
    for path in paths:
        with open(os.path.join(REPO, path), encoding='utf-8', errors='replace', newline='') as f:
            text = f.read()
        if not generated(text):
            texts[path] = text
    return texts


def main(argv):
    """Print each sentence opening on It or Its as file:line; exit 1 on any."""
    paths = [os.path.relpath(os.path.abspath(p), REPO).replace('\\', '/') for p in argv] or None
    count = 0
    for path, text in sorted(read(paths).items()):
        at = rows(text)
        for offset, sentence in opened(path, text):
            count += 1
            print('%s:%d: %s' % (path, bisect.bisect_right(at, offset), sentence[:110]))
    print('%d sentences open on It or Its' % count)
    return 1 if count else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
