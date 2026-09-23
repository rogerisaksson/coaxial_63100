"""The repository's own documents, reachable from a prompt."""
import os
import re

from .detail import TERSE

# The documents a bench question can reach.
NAMES = ('README', 'CLAUDE', 'ARCHITECTURE', 'PROTOCOL', 'HARDWARE',
         'FINDINGS', 'MODELS')

CLIP = 4000        # characters of one section, about a thousand tokens
FIND_HITS = 12     # lines reported for a search, before it is a document dump

# The same two numbers for a reader paying for them out of 8192 tokens shared
# with the conversation and the readings.
CLIP_TERSE = 1200
FIND_HITS_TERSE = 6


def _limits(level):
    """(section clip, search hits) for one detail level."""
    if level == TERSE:
        return CLIP_TERSE, FIND_HITS_TERSE
    return CLIP, FIND_HITS


def root():
    """The repository root, from this file rather than the shell's cwd."""
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


#: Headings one document may spend on the index. FINDINGS is a log and
#: grows without bound; the index is read on every turn that asks about
#: the documents at all, so it must not grow with it.
INDEX_HEADS = 12


def index(level=None):
    """Every document, its headings, and what a full read would cost."""
    terse = level == TERSE
    lines = []
    for name, path in sorted(paths().items()):
        text = _read(path)
        heads = _headings(text)
        if terse:
            lines.append(name)
        else:
            lines.append('%-12s %5d lines  ~%d tok'
                         % (name, text.count('\n') + 1, len(text) // 4))

        shown = [h for h in heads if not (terse and h[0] > 2)]
        # A LOG DOES NOT GET A LINE EACH.
        clipped = len(shown) - INDEX_HEADS
        if clipped > 0:
            shown = shown[-INDEX_HEADS:]

        for head_level, title, _ in shown:
            lines.append('  %s%s' % ('  ' * (head_level - 2), title))
        if clipped > 0:
            lines.append('  ... and %d older, by `doc=%s`' % (clipped, name))
    if not lines:
        return 'no documents found under %s' % root()
    lines.append('')
    lines.append("docs(doc=NAME) for one document, "
                 "docs(doc=NAME, section=TITLE) for one section, "
                 "docs(find=TEXT) to search")
    return '\n'.join(lines)


def outline(name, level=None):
    """One document's headings. The same at either level: this is already
    nothing but titles, and a shorter list of titles is a document the model
    cannot ask about by name.
    """
    found = paths()
    if name not in found:
        raise ValueError('no document %r; have %s'
                         % (name, ', '.join(sorted(found))))
    text = _read(found[name])
    # The instruction goes first, not last.
    lines = ["%s: headings only, %d lines. Titles, not answers - "
             "docs(doc='%s', section=TITLE) for the text, "
             "docs(find=TEXT) to search."
             % (name, text.count('\n') + 1, name)]
    for level, title, _ in _headings(text):
        lines.append('  %s%s' % ('  ' * (level - 2), title))
    return '\n'.join(lines)


def section(name, wanted, level=None):
    """One section, matched loosely on its heading, clipped."""
    found = paths()
    if name not in found:
        raise ValueError('no document %r; have %s'
                         % (name, ', '.join(sorted(found))))
    text = _read(found[name])
    lines = text.splitlines()
    heads = _headings(text)
    needle = wanted.strip().lower()

    # `depth` throughout, not `level`: the heading's depth and the detail
    # level are two different numbers and the second one is a parameter of
    # this function.
    hit = None
    for position, (depth, title, line_no) in enumerate(heads):
        low = title.lower()
        if low == needle or needle in low:
            hit = (position, depth, title, line_no)
            break
    if hit is None:
        raise ValueError('no section %r in %s; headings: %s'
                         % (wanted, name,
                            ', '.join(t for _, t, _ in heads) or '(none)'))

    position, depth, title, start = hit
    end = len(lines)
    for other_depth, _, other_line in heads[position + 1:]:
        # A subsection belongs to its parent; a sibling or an uncle ends it.
        if other_depth <= depth:
            end = other_line
            break
    clip, _ = _limits(level)
    body = '\n'.join(lines[start:end]).strip()
    if len(body) > clip:
        body = body[:clip].rstrip() + (
            '\n... clipped at %d characters. Ask for a subsection by name.'
            % clip)
    return '%s / %s\n%s' % (name, title, body)


def find(needle, level=None):
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
            # chapter is the meaning.
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
    _, keep = _limits(level)
    shown = hits[:keep]
    if len(hits) > keep:
        shown.append('... %d more, narrow the search' % (len(hits) - keep))
    return '\n'.join(shown)


def docs(session=None, doc=None, section=None, find=None, detail=None, **_):
    """The tool entry point. `session` is unused - documents are not the board.
    """
    if find:
        return globals()['find'](find, detail)
    if doc and section:
        return globals()['section'](doc, section, detail)
    if doc:
        return outline(doc, detail)
    return index(detail)
