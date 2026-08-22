"""The repository's own documents, reachable from a prompt.

The documents in `docs/` exist so that nobody re-derives what took real
measurements to establish - and until this module, the one reader who could not
open them was the model standing at the bench. It has `run_python`, so it could
technically read a file; it had no way to know which files, or that they were
worth reading at all.

The shape here is index first, section second, and that is the whole design.
The tool list is re-read every turn (see ARCHITECTURE.md on the token budget),
and a tool that returned a whole document by default would cost more than it
saves - FINDINGS.md alone is about 3500 tokens. So:

    docs()                          every document, its headings, its size
    docs(doc='FINDINGS')            one document's headings
    docs(doc='MODELS', section='Threads')
    docs(find='25.00')              where a phrase appears, with its heading

Sections are clipped, and the clip says so rather than trailing off. A model
that needs the rest asks for the subsection by name.

Read-only by construction: one fixed directory, a name allowlist built from
what is on disk, and no path from the arguments to the filesystem.
"""
import os
import re

# The documents a bench question can reach. CLAUDE.md and README.md are in
# here too: they are where the invariants and the commands live, and a model
# asking "what is AFE_ON for" should find that answer in the same place as the
# rest.
NAMES = ('README', 'CLAUDE', 'ARCHITECTURE', 'PROTOCOL', 'HARDWARE',
         'FINDINGS', 'MODELS')

CLIP = 4000        # characters of one section, about a thousand tokens
FIND_HITS = 12     # lines reported for a search, before it is a document dump


def root():
    """The repository root, from this file rather than the shell's cwd.

    host/coaxial_mcp/docs.py -> host/coaxial_mcp -> host -> the repository.
    Resolved on each call: cheap, and a stale module-level constant survives a
    move of the tree in a way that is annoying to debug.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))


def paths():
    """Name -> path, for the documents that actually exist."""
    base = root()
    found = {}
    for name in NAMES:
        for candidate in (os.path.join(base, name + '.md'),
                          os.path.join(base, 'docs', name + '.md')):
            if os.path.isfile(candidate):
                found[name] = candidate
                break
    return found


def _read(path):
    with open(path, encoding='utf-8', errors='replace') as handle:
        return handle.read()


def _headings(text):
    """(level, title, line index) for every ## or ### heading."""
    out = []
    for index, line in enumerate(text.splitlines()):
        match = re.match(r'^(#{2,3})\s+(.*\S)\s*$', line)
        if match:
            out.append((len(match.group(1)), match.group(2), index))
    return out


def index():
    """Every document, its headings, and what a full read would cost."""
    lines = []
    for name, path in sorted(paths().items()):
        text = _read(path)
        heads = _headings(text)
        lines.append('%-12s %5d lines  ~%d tok' % (name, text.count('\n') + 1,
                                                   len(text) // 4))
        for level, title, _ in heads:
            lines.append('  %s%s' % ('  ' * (level - 2), title))
    if not lines:
        return 'no documents found under %s' % root()
    lines.append('')
    lines.append("docs(doc=NAME) for one document, "
                 "docs(doc=NAME, section=TITLE) for one section, "
                 "docs(find=TEXT) to search")
    return '\n'.join(lines)


def outline(name):
    """One document's headings."""
    found = paths()
    if name not in found:
        raise ValueError('no document %r; have %s'
                         % (name, ', '.join(sorted(found))))
    text = _read(found[name])
    # The instruction goes first, not last. A small model handed an outline
    # will otherwise answer out of a heading - measured here: asked why the NTC
    # reads exactly 25.00, qwen2.5:14b took the heading "The NTC channel is not
    # anomalous, it is quiet" and stopped, which is a different finding about a
    # different thing.
    lines = ["%s: headings only, %d lines. Titles, not answers - "
             "docs(doc='%s', section=TITLE) for the text, "
             "docs(find=TEXT) to search."
             % (name, text.count('\n') + 1, name)]
    for level, title, _ in _headings(text):
        lines.append('  %s%s' % ('  ' * (level - 2), title))
    return '\n'.join(lines)


def section(name, wanted):
    """One section, matched loosely on its heading, clipped.

    Loose matching because a model quoting a heading back gets the case or a
    trailing word wrong often enough to matter, and the alternative - an error
    for 'Threads' against '### Threads' - teaches it to stop asking.
    """
    found = paths()
    if name not in found:
        raise ValueError('no document %r; have %s'
                         % (name, ', '.join(sorted(found))))
    text = _read(found[name])
    lines = text.splitlines()
    heads = _headings(text)
    needle = wanted.strip().lower()

    hit = None
    for position, (level, title, line_no) in enumerate(heads):
        low = title.lower()
        if low == needle or needle in low:
            hit = (position, level, title, line_no)
            break
    if hit is None:
        raise ValueError('no section %r in %s; headings: %s'
                         % (wanted, name,
                            ', '.join(t for _, t, _ in heads) or '(none)'))

    position, level, title, start = hit
    end = len(lines)
    for other_level, _, other_line in heads[position + 1:]:
        # A subsection belongs to its parent; a sibling or an uncle ends it.
        if other_level <= level:
            end = other_line
            break
    body = '\n'.join(lines[start:end]).strip()
    if len(body) > CLIP:
        body = body[:CLIP].rstrip() + (
            '\n... clipped at %d characters. Ask for a subsection by name.' % CLIP)
    return '%s / %s\n%s' % (name, title, body)


def find(needle):
    """Where a phrase appears, with the heading it appears under."""
    needle = str(needle).strip()
    if not needle:
        raise ValueError('find needs something to look for')
    low = needle.lower()

    hits = []
    for name, path in sorted(paths().items()):
        text = _read(path)
        heads = _headings(text)
        for number, line in enumerate(text.splitlines()):
            if low not in line.lower():
                continue

            # BOTH ancestors, chapter and entry, because in FINDINGS the
            # chapter is the meaning. Measured here: asked what had been ruled
            # out about the phase V offset, qwen2.5:14b found the entry
            # '"PCSEL accumulation explains the Phase V offset"' and reported
            # it as the explanation - and that entry lives under "Refuted:
            # plausible, wrong, do not revisit". A hit without its chapter can
            # say the opposite of what the document says.
            chapter = entry = ''
            for level, title, head_line in heads:
                if head_line > number:
                    break
                if level == 2:
                    chapter, entry = title, ''
                else:
                    entry = title
            where = chapter if not entry else '%s / %s' % (chapter, entry)
            hits.append('%-10s %-46s %s' % (name, where[:46], line.strip()[:70]))
    if not hits:
        return 'no document mentions %r' % needle
    shown = hits[:FIND_HITS]
    if len(hits) > FIND_HITS:
        shown.append('... %d more, narrow the search' % (len(hits) - FIND_HITS))
    return '\n'.join(shown)


def docs(session=None, doc=None, section=None, find=None, **_):
    """The tool entry point. `session` is unused - documents are not the board.

    Named the same as the module's own functions on purpose: the tool argument
    is the noun the model uses, and shadowing inside this one function is
    cheaper to read than an argument called `section_name`.
    """
    if find:
        return globals()['find'](find)
    if doc and section:
        return globals()['section'](doc, section)
    if doc:
        return outline(doc)
    return index()
